#!/usr/bin/env python

# Copyright (c) 2018 Intel Labs.
# authors: Fabian Oboril (fabian.oboril@intel.com)
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Scenarios in which another (opposite) vehicle 'illegally' takes
priority, e.g. by running a red traffic light.
"""

from __future__ import print_function
import sys
import json
import argparse

import py_trees
import carla

from srunner.scenariomanager.atomic_scenario_behavior import *
from srunner.scenariomanager.atomic_scenario_criteria import *
from srunner.scenariomanager.timer import TimeOut
from srunner.scenarios.basic_scenario import *

SCENARIO_GENERATOR_SCENARIOS = [
    "ScenarioGenerator"
]

collision_location = None
collision_coordinates = None

def point_tf_to_carla(x, y, scale=5):
    # each unitary movement from the sequence is 5m in the simulator, x and y axis are inverted
    # collision happens at -4.5, -133 in the simulator (6,3 in the sequence)
    # TODO: hard coded collision location
    t = carla.Transform()
    t.location.x = scale*collision_coordinates.x + collision_location.x
    t.location.y = scale*collision_coordinates.y + collision_location.y
    t.location.z = collision_coordinates.z + collision_location.z
    t.rotation.pitch = 0
    t.rotation.yaw = 1.5 # angle between the road lane and y axis in degrees
    t.rotation.roll = 0 
    location = carla.Location(x=-scale*x, y=-scale*y, z=1)
    t.transform(location)

    return location


class AutoPilot(BasicAgent):
    def __init__(self, vehicle, target_speed):
        super(AutoPilot, self).__init__(vehicle, target_speed)
        
    """Set autopilot destination: carla.Location"""
    def set_destination(self, destination):
        super(AutoPilot, self).set_destination([destination.x, destination.y, destination.z])   
        self._destination = destination

    def tick(self, debug=False):
        if self.done():
            self.brake()
        else:
            control = super(AutoPilot, self).run_step(debug)
            self._vehicle.apply_control(control)

    def distance(self):
        return self._destination.distance(self._vehicle.get_location())

    def brake(self, brake_value=1):
        control = carla.VehicleControl()
        control.brake = brake_value
        self._vehicle.apply_control(control)

        
class SequenceStep(object):
    """
    A step of the sequence.
    Only movements of an actor are supported as step
    """
    def __init__(self, dict_step):
        self.actor = dict_step["actor"]
        self.id = dict_step["id"] if "id" in dict_step else 0
        self.required_by = dict_step["required_by"] if "required_by" in dict_step else 1
        self.action = dict_step["action"]
        if self.actor == "walker": # TODO: Harcoded
            self.loc = point_tf_to_carla(dict_step["next_location"]["x"], dict_step["next_location"]["y"], scale=2.5)
        else:
            self.loc = point_tf_to_carla(dict_step["next_location"]["x"], dict_step["next_location"]["y"])

    def __str__(self):
        return "{}: action {} of id {} to {}, required by {}".format(self.actor, self.action, self.id, self.loc, self.required_by)

"""
A class to manage and process the test case sequence
The class sort the steps of the sequence by actor 
then create the complete test case sequence by merging the actor's sequences 
"""
class RawSequence(object):
    """Raw sequence from the generator"""
    def __init__(self, file_name):
        with open(file_name, 'r') as f:
            self.original_sequence = json.load(f)
        self.sequence = [] # 

    def parallelization_by_dependency(self):
        self.sort_by_actor()
        self.parallelize_sequence()

    def parallelization_by_TICK(self):
        self.sequence = []
        parallel_steps = []
        for step in self.original_sequence["sequence"]:
            if step["action"] == "tick":
                self.sequence += [parallel_steps]
                parallel_steps = []
            elif (step["action"] == "move"):
                parallel_steps += [SequenceStep(step)]
            # elif step["action"] == "idle": # idle steps are ignored 

    # create a dictionnary a with actor as key and a list for each actor containing the actions of the actor
    def sort_by_actor(self):
        self.actor_sorted_sequence = {} # {"actor": [[carla.Location n0], ...], ...}
        for actor in self.original_sequence["actors"]:
            # loc = point_tf_to_carla(actor["start_location"]["x"], actor["start_location"]["y"])
            self.actor_sorted_sequence[actor["name"]] =[] 
        for step in self.original_sequence["sequence"]:
            if step["action"] == "move":
                self.actor_sorted_sequence[step["actor"]] += [SequenceStep(step)]
            # elif step["action"] == "idle": # idle steps are ignored 
    
    def is_sorted_sequence_empty(self):
        for seq in self.actor_sorted_sequence.values():
            if seq:
                return False
        return True

    def dependencies_satisfied(self, step):
        for step_id in step.required_by:
            for seq_ in self.actor_sorted_sequence.values():
                for step_ in seq_:
                    if step_id == step_.id:
                        return False
        return True

    # create a parallelized sequence from actor_sorted_sequence, contructs the sequence by startig from the last action and adding actions whose dependencies are satisfied
    def parallelize_sequence(self):
        self.sequence = []
        self.actor_sorted_sequence_copy = {actor: [] for actor in self.actor_sorted_sequence}
        while not self.is_sorted_sequence_empty():
            parallel_steps = []
            for actor, seq in self.actor_sorted_sequence.iteritems():
                if seq:
                    step = seq.pop()
                    if self.dependencies_satisfied(step):
                        parallel_steps += [step]
                        self.actor_sorted_sequence_copy[actor] += [step]
                    else:
                        seq += [step]

            if not parallel_steps:
                self.display_actor_seq()
                raise Exception("Parallelization blocked, can't resolve dependencies")
            else:
                self.sequence += [parallel_steps]
        for value in self.actor_sorted_sequence_copy.values():
            value.reverse()
        self.actor_sorted_sequence = self.actor_sorted_sequence_copy
        self.sequence.reverse()

    def get_initial_pose(self, actor_name):
        for actor in self.original_sequence["actors"]:
            if actor["name"] == actor_name:
                x = actor["start_location"]["x"]
                y = actor["start_location"]["y"]
                yaw = actor["start_location"]["yaw"]
                return x, y, yaw

    def display_actor_seq(self):
        print("{:^30}".format("ACTOR SEQUENCE"))
        for actor in self.actor_sorted_sequence:
            print("{}".format(actor))
            for step in self.actor_sorted_sequence[actor]:
                print("    {}".format(step))
    
    def display_seq(self):
        print("{:^30}".format("SEQUENCE"))
        for i, steps in enumerate(self.sequence):
            print("Seq step {}".format(i))
            for step in steps:
                print("    {}".format(step))

"""
A class to share ressource between nodes of the behaviour tree
"""
class SharedRessources(object):
    def __init__(self):
        self.speeds = {} 
    

class ScenarioGenerator(BasicScenario):
    """
    This class holds everything required for a scenario,
    in which a pedestrian cross the road while the car traffic light is green
    The pedestrian tries to cross between the leading and the ego vehicle
    The result is a collision between the ego vehicle and the pedestrian
    The actors wait for the traffic light to turn green

    This scenario is different from PedestrianCrossingRedLight, the behaviour tree
    is generated from a .json file (./srunner/configs/seq?.json, name is hard coded)
    3 different methods generates the tree
    Location: Town03
    """

    timeout = 5000 # Timeout of scenario in seconds

    # moving vehicles paramters
    _expected_driven_distance = 40
    _security_distance = 10

    _cell_distance = 5 # meters
    _cell_distance_walker = 2.5 # meters
    _vehicle_speed = 5
    _target_velocity = _vehicle_speed
    _walker_speed = 0.5
    _distance_to_location = 2
    _distance_to_location_walker = 0.3
    _cross_start_distance = 10
   
    _intersection_location = carla.Location(x=-3.5, y=-150, z=0)
    _traffic_light_location = carla.Location(x=-11.5, y=-125.0, z=0.15)
    _traffic_light = None
    _location_of_collision = carla.Location(x=-3.5, y=-133, z=1)

    _lidar_location = carla.Transform(
        carla.Location(x=0, y=0, z=2.32),carla.Rotation(roll=0.74484513, pitch=0.85943669))
    _camera_location = carla.Transform(
        carla.Location(x= -4, y=0, z=3.5), carla.Rotation(pitch=-15))

    def __init__(self, world, ego_vehicle, other_actors, town, randomize=False, debug_mode=False, sequence_file=""):
        global collision_coordinates, collision_location
        """
        Setup all relevant parameters and create scenario
        and instantiate scenario manager
        """
        
        # self.lidar = setup_sensor(
        #     world,
        #     "lidar",
        #     self._lidar_location,
        #     self.ego_vehicle) 
        # self.camera_rgb = setup_sensor(
        #     world,
        #     "camera.rgb",
        #     self._camera_location,
        #     self.ego_vehicle)
        self.world_debug = world.debug
        self.ego_vehicle = ego_vehicle
        self.leading_vehicle = other_actors[0]
        stop_control = carla.VehicleControl()
        stop_control.brake = 1
        self.ego_vehicle.apply_control(stop_control)
        self.leading_vehicle.apply_control(stop_control)
        # self.stopped_vehicle = other_actors[1]
        
        self.ego_autopilot = AutoPilot(self.ego_vehicle, 30)
        self.leading_autopilot = AutoPilot(self.leading_vehicle, 30)
        
        self.walker = other_actors[1]

        self.walker_control = carla.WalkerControl()
        self.walker_control_direction = 180 # yaw in degrees
        self.walker_control_speed = 1.8 # m/s

        self.shared_ressources = SharedRessources()
        
        for actor in world.get_actors().filter('traffic.traffic_light'):
            if actor.get_location().distance(self._traffic_light_location) < 1.0:
                self._traffic_light = actor

        if self._traffic_light is None:
            print("No traffic light for the given location found")
            sys.exit(-1)
        self._traffic_light.set_state(carla.TrafficLightState.Green)
        self._traffic_light.freeze(True)
        
        testcase_args = argparse.ArgumentParser(description="Parser to retrieve the Name of the file of the test case", usage='%(prog)s [options] --test-case')
        testcase_args.add_argument('--test-case', required=True,    help='Name of the file of the test case')
        testcase_args = testcase_args.parse_known_args()[0]
        
        if testcase_args.test_case not in ["seq1", "seq2", "seq3", "seq4", "seq5", "seq6"]:
            print("Unknown testcase named {}, testcase available: 'seq1' to 'seq6'".format(testcase_args.test_case))
            sys.exit(-1)

        self.sequence = RawSequence("./srunner/configs/{}.json".format(testcase_args.test_case)) # TODO: hard coded file name
        
        #TODO: improve configuration
        if testcase_args.test_case in ["seq3", "seq4"]:
            collision_location = carla.Location(x=-4.5, y=-141, z=0)
            collision_coordinates = carla.Location(x=7, y=5, z=0)
        else: # seq 1, 2, 5, 6
            collision_location = carla.Location(x=-4.5, y=-133, z=0)
            collision_coordinates = carla.Location(x=6, y=3, z=0)

        if testcase_args.test_case == "seq1":
            self.sequence.parallelization_by_dependency()
        else: # seq 3
            self.sequence.parallelization_by_TICK()
        self.sequence.display_seq()

        
        x, y, yaw = self.sequence.get_initial_pose("ego")
        ego_vehicle_start = carla.Transform(point_tf_to_carla(x, y), carla.Rotation(yaw=yaw+1.5))
        ego_vehicle_start.location.z = 2
        self.ego_vehicle.set_transform(ego_vehicle_start)

        x, y, yaw = self.sequence.get_initial_pose("leading")
        leading_vehicle_start = carla.Transform(point_tf_to_carla(x, y), carla.Rotation(yaw=yaw+1.5))
        self.leading_vehicle.set_transform(leading_vehicle_start)

        x, y, yaw = self.sequence.get_initial_pose("walker")
        walker_vehicle_start = carla.Transform(point_tf_to_carla(x, y, scale=2.5), carla.Rotation(yaw=yaw+1.5))
        self.walker.set_transform(walker_vehicle_start)
        self.walk_orientation = yaw
        print("yaw", self.walk_orientation)

        super(ScenarioGenerator, self).__init__("ScenarioGenerator",
                                                ego_vehicle,
                                                other_actors,
                                                town,
                                                world,
                                                debug_mode)
    


    def _create_node_drive_distance(self, vehicle, distance, speed, vehicle_name):
        travel_distance = py_trees.composites.Parallel("Keep velocity {} m/s for {} meters".format(speed, distance), policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        keep_velocity = KeepVelocity(vehicle, speed, name="Velocity {} m/s".format(speed))
        check_distance = DriveDistance(vehicle, distance, name="Distance {} meters".format(distance))
        travel_distance.add_child(keep_velocity)
        travel_distance.add_child(check_distance)
        
        stop = StopVehicle(vehicle, 1, name="Stop {}".format(vehicle_name))
        
        travel_distance_then_stop = py_trees.composites.Sequence("Vehicle {} drives for {} meters at {} m/s ".format(vehicle_name, distance, speed))
        travel_distance_then_stop.add_child(travel_distance)
        travel_distance_then_stop.add_child(stop)

        return travel_distance_then_stop
        
    def _create_node_walk_distance(self, walker, distance, speed):
        travel_distance = py_trees.composites.Parallel("Keep velocity {} m/s for {} meters".format(speed, distance), policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        keep_velocity = KeepVelocity_pedestrian(walker, speed, self.walk_orientation, name="Velocity {} m/s".format(speed))
        check_distance = DriveDistance(walker, distance, name="Distance {} meters".format(distance))
        travel_distance.add_child(keep_velocity)
        travel_distance.add_child(check_distance)
        
        # stop = StopVehicle(walker, 1, name="Stop {}".format(vehicle_name))
        
        travel_distance_then_stop = py_trees.composites.Sequence("Walker walks for {} meters at {} m/s ".format(distance, speed))
        travel_distance_then_stop.add_child(travel_distance)
        # travel_distance_then_stop.add_child(stop)

        return travel_distance_then_stop


    """
    First method, the behaviours tree is written by hand is the script (the json file is not read)
    Actors actions are sequencial, they move one after each other like in a turn-by-turn board game
    """
    def _create_simple_behavior(self):
        sequence = py_trees.composites.Sequence("Sequence Behavior")
        sequence.add_child(self._create_node_drive_distance(self.ego_vehicle, self._cell_distance, 4, "Ego vehicle"))
        sequence.add_child(self._create_node_drive_distance(self.ego_vehicle, self._cell_distance, 4, "Ego vehicle"))
        # ego vehicle blocked because the leading vehicle has not moved yet
        sequence.add_child(IDLE(1, name="Ego vehicle idle"))
        sequence.add_child(IDLE(1, name="Ego vehicle idle"))
        sequence.add_child(IDLE(1, name="Ego vehicle idle"))
        sequence.add_child(self._create_node_drive_distance(self.leading_vehicle, self._cell_distance, 4, "Leading vehicle"))
        sequence.add_child(IDLE(1, name="Ego vehicle idle"))
        sequence.add_child(self._create_node_drive_distance(self.leading_vehicle, self._cell_distance, 4, "Leading vehicle"))
        sequence.add_child(self._create_node_drive_distance(self.ego_vehicle, self._cell_distance, 4, "Ego vehicle"))
        sequence.add_child(self._create_node_drive_distance(self.leading_vehicle, self._cell_distance, 4, "Leading vehicle"))
        sequence.add_child(self._create_node_drive_distance(self.ego_vehicle, self._cell_distance, 4, "Ego vehicle"))
        sequence.add_child(self._create_node_drive_distance(self.leading_vehicle, self._cell_distance, 4, "Leading vehicle"))
        sequence.add_child(self._create_node_drive_distance(self.ego_vehicle, self._cell_distance, 4, "Ego vehicle"))
        sequence.add_child(self._create_node_drive_distance(self.leading_vehicle, self._cell_distance, 4, "Leading vehicle"))
        sequence.add_child(self._create_node_drive_distance(self.leading_vehicle, self._cell_distance, 4, "Leading vehicle"))
        sequence.add_child(self._create_node_drive_distance(self.leading_vehicle, self._cell_distance, 4, "Leading vehicle"))
        # Leading vechicle passed the intersection
        sequence.add_child(self._create_node_walk_distance(self.walker, self._cell_distance_walker, 0.5))
        sequence.add_child(self._create_node_walk_distance(self.walker, self._cell_distance_walker, 0.5))
        sequence.add_child(self._create_node_walk_distance(self.walker, self._cell_distance_walker, 0.5))
        sequence.add_child(self._create_node_drive_distance(self.ego_vehicle, self._cell_distance, 4, "Ego vehicle"))
        sequence.add_child(self._create_node_walk_distance(self.walker, self._cell_distance_walker, 0.5))
        sequence.add_child(self._create_node_drive_distance(self.ego_vehicle, self._cell_distance, 4, "Ego vehicle"))
        # collision: ego vehicle into walker !!
        # 10 sec wait until terminate
        sequence.add_child(IDLE(10, name="10 sec wait until terminate"))
        return sequence


    class AutoPilot_LocalPlanner(object):
        class _Agent(object):
            def __init__(self, vehicle):
                self.vehicle = vehicle
                
        class _Waypoint(object):
            def __init__(self):
                self.transform = carla.Transform()
            
        def __init__(self, vehicle, target_speed):
            self.local_planner = LocalPlanner(self._Agent(vehicle))
            self._vehicle = vehicle
            self.target_speed = target_speed
            
        """Set AutoPilot_LocalPlanner destination: carla.Location"""
        def set_destination(self, destination):
            self._destination = destination
            self.wp = self._Waypoint()
            self.wp.transform.location = destination
            self.local_planner._waypoint_buffer.clear()
            self.local_planner.waypoints_queue.clear()
        
        def set_speed(self, target_speed):
            self.target_speed = target_speed

        def tick(self, debug=False):
            self.local_planner.set_global_plan([(self.wp, RoadOption.VOID)])
            control = self.local_planner.run_step(self.target_speed, True)
            self._vehicle.apply_control(control)

        def distance(self):
            loc = self._vehicle.get_location()
            loc.z = self._destination.z
            return self._destination.distance(loc)

        def brake(self, brake_value=1):
            control = carla.VehicleControl()
            control.brake = brake_value
            self._vehicle.apply_control(control)

        def done(self):
           return self.distance() < 1


    """
    Second method, the behaviour tree is a sequence of parallel actions, at each step of the sequence several actors may be moving at the same time
    All actions of step must be accomplished before going to the next step and velocities are not synchronised so the fastest actor may be stopped, waiting for others to finish their action
    """
    def _create_auto_pilot_behavior(self):
        behaviour_tree = py_trees.composites.Sequence("Sequence Behavior")
        self.ego_agent = self.AutoPilot_LocalPlanner(self.ego_vehicle, self._vehicle_speed)
        self.leading_agent = self.AutoPilot_LocalPlanner(self.leading_vehicle, self._vehicle_speed)
        behaviour_tree = py_trees.composites.Sequence("Sequence Behavior")
        
        for i, steps in enumerate(self.sequence.sequence):
            state = py_trees.composites.Parallel("State {}".format(i), policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
            for step in steps:
                # TODO: hard coded vehicle selection
                if step.actor == "ego":
                    state.add_child(LocalPlannerBehaviour(self.ego_agent, step.loc, "ego_{}".format(i)))
                    # state.add_child(BasicAgentBehavior(self.ego_vehicle, step.loc, "ego_{}".format(i)))
                    # state.add_child(self._create_node_drive_distance(self.ego_vehicle, self._cell_distance, self._vehicle_speed, "Ego vehicle"))
                elif step.actor == "leading":
                    state.add_child(LocalPlannerBehaviour(self.leading_agent, step.loc, "leading_{}".format(i)))
                    # state.add_child(BasicAgentBehavior(self.leading_vehicle, step.loc, "leading_{}".format(i)))
                    # state.add_child(self._create_node_drive_distance(self.leading_vehicle, self._cell_distance, self._vehicle_speed, "Leading vehicle"))
                elif step.actor == "walker":
                    state.add_child(self._create_node_walk_distance(self.walker, self._cell_distance_walker, self._walker_speed))
                else:
                    raise Exception("Unknown acto '{}' in sequence, can't associate actor".format(step.actor))
            behaviour_tree.add_child(state)
        return behaviour_tree

    def _node_drive_to_point(self, vehicle, vehicle_name, location):
        drive_to_loc = py_trees.composites.Parallel("{} drives to {}".format(vehicle_name, location), policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        keep_velocity = KeepSharedVelocityPID(vehicle, vehicle_name, self.shared_ressources, name="Dynamic velocity")
        check_location = InTriggerDistanceToLocation(vehicle, location, self._distance_to_location, name="Destination {}".format(location))
        drive_to_loc.add_children([keep_velocity, check_location])
        return drive_to_loc

    def _node_walk_to_point(self, walker, walker_name, location):
        walk_to_loc = py_trees.composites.Parallel("{} walks to {}".format(walker_name, location), policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        keep_velocity = KeepSharedVelocity_pedestrian(walker, walker_name, self.shared_ressources, self.walk_orientation, name="Dynamic velocity") # TODO; hard coded orientation
        check_location = InTriggerDistanceToLocation(walker, location, self._distance_to_location_walker, name="Destination {}".format(location))
        walk_to_loc.add_children([keep_velocity, check_location])
        return walk_to_loc

    """
    Third method, the behaviour tree is a sequence of parallel actions, at each step of the sequence several actors may be moving at the same time
    All actions of step must be accomplished before going to the next step but velocities are synchronised the actors finish their actions simultaneously
    """
    def _create_synchronized_parallelized_behavior(self):
        behaviour_tree = py_trees.composites.Sequence("Sequence Behavior")
        
        for i, steps in enumerate(self.sequence.sequence):
            
            apply_speed = py_trees.composites.Parallel("Apply speed to actors", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE) #TODO: maybe set to SUCCESS_ON_ONE ?
            actors_dict = []
            for step in steps:
                # TODO: hard coded vehicle selection
                if step.actor == "ego":
                    apply_speed.add_child(self._node_drive_to_point(self.ego_vehicle, "ego", step.loc))
                    actors_dict += [{"name": "ego", "actor": self.ego_vehicle, "dest": step.loc, "vmax": self._vehicle_speed}]
                    pass
                elif step.actor == "leading":
                    apply_speed.add_child(self._node_drive_to_point(self.leading_vehicle, "leading", step.loc))
                    actors_dict += [{"name": "leading", "actor": self.leading_vehicle, "dest": step.loc, "vmax": self._vehicle_speed}]
                elif step.actor == "walker":
                    apply_speed.add_child(self._node_walk_to_point(self.walker, "walker", step.loc))
                    actors_dict += [{"name": "walker", "actor": self.walker, "dest": step.loc, "vmax": self._walker_speed}]
                    pass
                else:
                    raise Exception("Unknown acto '{}' in sequence, can't associate actor".format(step.actor))
            state = py_trees.composites.Parallel("State {}".format(i), policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
            state.add_child(ComputeSpeedFactor(actors_dict, self.shared_ressources))
            state.add_child(apply_speed)

            behaviour_tree.add_child(state)
        behaviour_tree.add_child(IDLE(10, name="10 sec wait until terminate"))
            
        return behaviour_tree
        
    def _create_behavior(self):
        """
        Scenario behavior:
        The other vehicle waits until the ego vehicle is close enough to the
        intersection and that its own traffic light is red. Then, it will start
        driving and 'illegally' cross the intersection. After a short distance
        it should stop again, outside of the intersection. The ego vehicle has
        to avoid the crash, but continue driving after the intersection is clear.

        If this does not happen within 120 seconds, a timeout stops the scenario
        """
        node_seq_inversed = []

        # sequence automaticaly generated 
        # Build behavior tree


        # behaviour_tree = self._create_synchronized_parallelized_behavior()
        behaviour_tree = self._create_auto_pilot_behavior()
        behaviour_tree_with_debug = py_trees.composites.Parallel("Behaviour tree", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        behaviour_tree_with_debug.add_child(behaviour_tree)
        loc_list = [step.loc if step.actor == "leading" else carla.Location() for steps in self.sequence.sequence for step in steps ]
        behaviour_tree_with_debug.add_child(DebugBehaviour(self.world_debug, loc_list))
        for e in loc_list:
            print(e)
        return behaviour_tree
        # return behaviour_tree_with_debug

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        # velocity_criterion = AverageVelocityTest(self.ego_vehicle, self._target_velocity)
        # collision_criterion = CollisionTest(self.ego_vehicle)
        # driven_distance_criterion = DrivenDistanceTest(self.ego_vehicle, self._expected_driven_distance)
        # criteria.append(velocity_criterion)
        # # criteria.append(collision_criterion)
        # criteria.append(driven_distance_criterion)

        velocity_criterion = AverageVelocityTest(self.leading_vehicle, self._target_velocity)
        # collision_criterion = CollisionTest(self.leading_vehicle)
        # driven_distance_criterion = DrivenDistanceTest(self.leading_vehicle, self._expected_driven_distance)
        criteria.append(velocity_criterion)
        # criteria.append(collision_criterion)
        # criteria.append(driven_distance_criterion)

        # velocity_criterion = AverageVelocityTest(self.other_vehicles[1], 0)
        # collision_criterion = CollisionTest(self.other_vehicles[1])
        # criteria.append(velocity_criterion)
        # criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        self._traffic_light = None
        # super(ScenarioGenerator, self).__del__()