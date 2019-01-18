#!/usr/bin/env python

#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Non-signalized junctions: crossing negotiation:

The hero vehicle is passing through a junction without traffic lights
And encounters another vehicle passing across the junction.
"""

import py_trees
import carla

from ScenarioManager.atomic_scenario_behavior import *
from ScenarioManager.atomic_scenario_criteria import *
from ScenarioManager.scenario_manager import Scenario
from ScenarioManager.timer import TimeOut
from Scenarios.basic_scenario import *


class DynamicObjectDetect(BasicScenario):

    """
    Implementation class for
    'Non-signalized junctions: crossing negotiation' scenario,
    Traffic Scenario 10.

    Location    :   Town03
    """

    # ego vehicle parameters
    _ego_vehicle_model = 'vehicle.nissan.micra'
    _ego_vehicle_start = carla.Transform(
        # carla.Location(x=6.1, y= -124, z=0.5), carla.Rotation(yaw=271.5))
        carla.Location(x=-14.2, y= -133, z=0.5), carla.Rotation(yaw=1.5))
    _ego_vehicle_max_velocity = 15
    _ego_vehicle_driven_distance = 105
    _ego_vehicle_max_brake = 1.0


    _lidar_location = carla.Transform(
        carla.Location(x=0, y=0, z=2.4),carla.Rotation(roll=0.74484513, pitch=0.85943669))

    _camera_location = carla.Transform(
        carla.Location(x= -4, y=0, z=3.5), carla.Rotation(pitch=-15))



    # other vehicle
    _other_vehicle_model = 'vehicle.tesla.model3'
    _other_vehicle_start = carla.Transform(
        carla.Location(x=4.5, y= -60, z=0.5), carla.Rotation(yaw=271.5))
    _other_vehicle_max_brake = 1.0
    _other_vehicle_target_velocity = 15

    def __init__(self, world, debug_mode=False, params = None):
        """
        Setup all relevant parameters and create scenario
        and instantiate scenario manager
        """

        self.dt = world.wait_for_tick().delta_seconds
        print("delta_seconds", self.dt)


        if params is not None:
            self._ego_vehicle_max_velocity = params.ego_vehicle_vel
            self._other_vehicle_target_velocity = params.other_vehicle_vel



        self.other_vehicles = [setup_vehicle(
            world,
            self._other_vehicle_model,
            self._other_vehicle_start)]
        self.ego_vehicle = setup_vehicle(
            world,
            self._ego_vehicle_model,
            self._ego_vehicle_start, zoe=True)

        self.lidar = setup_sensor(
            world,
            "lidar",
            self._lidar_location,
            self.ego_vehicle)

        self.camera_rgb = setup_sensor(
            world,
            "camera.rgb",
            self._camera_location,
            self.ego_vehicle)

        super(DynamicObjectDetect, self).__init__(
            name="DynamicObjectDetect",
            town="Town03",
            world=world,
            debug_mode=debug_mode)


        try:
            # Setup scenario
            if debug_mode:
                py_trees.logging.level = py_trees.logging.Level.DEBUG

            behavior = self._create_behavior()
            criteria = self._create_test_criteria()
            self.scenario = Scenario(
                behavior, criteria, self.name, self.timeout)
        except KeyboardInterrupt:
            print("Interrupted!")

    def _create_behavior(self):
        """
        After invoking this scenario, it will wait for the user
        controlled vehicle to enter the start region,
        then make a traffic participant to accelerate
        until it is going fast enough to reach an intersection point.
        at the same time as the user controlled vehicle at the junction.
        Once the user controlled vehicle comes close to the junction,
        the traffic participant accelerates and passes through the junction.
        After 60 seconds, a timeout stops the scenario.
        """

        # Creating leaf nodes

        keep_velocity_other = KeepVelocityPID(
            self.other_vehicles[0],
            self._other_vehicle_target_velocity)

        stop_other_trigger = InTriggerRegion(
            self.other_vehicles[0],
             3, 7,
            -123,-111)

        keep_stoping_other = StopVehicleForTime(
            self.other_vehicles[0],
            4/self.dt)

        stop_other_trigger_2 = InTriggerRegion(
            self.other_vehicles[0],
            5.5, 10.5,
            -164, -154)

        stop_other = StopVehicle(
            self.other_vehicles[0],
            self._other_vehicle_max_brake)

        end_condition = InTriggerRegion(
            self.other_vehicles[0],
             5.5, 10.5,
            -190, -155)


        root_timeout = TimeOut(self.timeout)

        # Creating non-leaf nodes
        root = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        scenario_sequence = py_trees.composites.Sequence()
        keep_velocity_other_parallel = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        keep_velocity_other_parallel_2 = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        # Building tree
        root.add_child(scenario_sequence)
        root.add_child(root_timeout)
        # scenario_sequence.add_child(start_other_trigger)
        scenario_sequence.add_child(keep_velocity_other_parallel)
        scenario_sequence.add_child(keep_stoping_other)
        scenario_sequence.add_child(keep_velocity_other_parallel_2)
        scenario_sequence.add_child(stop_other)
        scenario_sequence.add_child(end_condition)
        keep_velocity_other_parallel.add_child(keep_velocity_other)
        keep_velocity_other_parallel.add_child(stop_other_trigger)

        keep_velocity_other_parallel_2.add_child(keep_velocity_other)
        keep_velocity_other_parallel_2.add_child(stop_other_trigger_2)

        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        # Adding checks for ego vehicle
        collision_criterion_ego = CollisionTest(self.ego_vehicle)
        # driven_distance_criterion = DrivenDistanceTest(
        #     self.ego_vehicle, self._ego_vehicle_driven_distance)
        criteria.append(collision_criterion_ego)
        # criteria.append(driven_distance_criterion)

        # Add approriate checks for other vehicles
        for vehicle in self.other_vehicles:
            collision_criterion = CollisionTest(vehicle)
            criteria.append(collision_criterion)

        return criteria




class DynamicObjectDetect2(BasicScenario):

    """
    Implementation class for
    'Non-signalized junctions: crossing negotiation' scenario,
    Traffic Scenario 10.

    Location    :   Town03
    """

    # ego vehicle parameters
    _ego_vehicle_model = 'vehicle.nissan.micra'
    _ego_vehicle_start = carla.Transform(
        # carla.Location(x=6.1, y= -124, z=0.5), carla.Rotation(yaw=271.5))
        carla.Location(x=-14.2, y= -133, z=0.5), carla.Rotation(yaw=1.5))
    _ego_vehicle_max_velocity = 15
    _ego_vehicle_driven_distance = 105
    _ego_vehicle_max_brake = 1.0


    _lidar_location = carla.Transform(
        carla.Location(x=0, y=0, z=2.4),carla.Rotation(roll=0.74484513, pitch=0.85943669))

    _camera_location = carla.Transform(
        carla.Location(x= -4, y=0, z=3.5), carla.Rotation(pitch=-15))



    # other vehicle
    _other_vehicle_model = 'vehicle.tesla.model3'
    _other_vehicle_start = carla.Transform(
        carla.Location(x=-70, y=-134, z=0.5), carla.Rotation(yaw=1.5))
    _other_vehicle_max_brake = 1.0
    _other_vehicle_target_velocity = 15

    def __init__(self, world, debug_mode=False, params = None):
        """
        Setup all relevant parameters and create scenario
        and instantiate scenario manager
        """

        self.dt = world.wait_for_tick().delta_seconds
        print("delta_seconds", self.dt)


        if params is not None:
            self._ego_vehicle_max_velocity = params.ego_vehicle_vel
            self._other_vehicle_target_velocity = params.other_vehicle_vel



        self.other_vehicles = [setup_vehicle(
            world,
            self._other_vehicle_model,
            self._other_vehicle_start)]
        self.ego_vehicle = setup_vehicle(
            world,
            self._ego_vehicle_model,
            self._ego_vehicle_start, zoe=True)

        self.lidar = setup_sensor(
            world,
            "lidar",
            self._lidar_location,
            self.ego_vehicle)

        self.camera_rgb = setup_sensor(
            world,
            "camera.rgb",
            self._camera_location,
            self.ego_vehicle)

        super(DynamicObjectDetect, self).__init__(
            name="DynamicObjectDetect",
            town="Town03",
            world=world,
            debug_mode=debug_mode)


        try:
            # Setup scenario
            if debug_mode:
                py_trees.logging.level = py_trees.logging.Level.DEBUG

            behavior = self._create_behavior()
            criteria = self._create_test_criteria()
            self.scenario = Scenario(
                behavior, criteria, self.name, self.timeout)
        except KeyboardInterrupt:
            print("Interrupted!")

    def _create_behavior(self):
        """
        After invoking this scenario, it will wait for the user
        controlled vehicle to enter the start region,
        then make a traffic participant to accelerate
        until it is going fast enough to reach an intersection point.
        at the same time as the user controlled vehicle at the junction.
        Once the user controlled vehicle comes close to the junction,
        the traffic participant accelerates and passes through the junction.
        After 60 seconds, a timeout stops the scenario.
        """

        # Creating leaf nodes

        keep_velocity_other = KeepVelocityPID(
            self.other_vehicles[0],
            self._other_vehicle_target_velocity)

        stop_other_trigger = InTriggerRegion(
            self.other_vehicles[0],
            -24, -14,
            -137,-127.5)

        keep_stoping_other = StopVehicleForTime(
            self.other_vehicles[0],
            4/self.dt)

        stop_other_trigger_2 = InTriggerRegion(
            self.other_vehicles[0],
            31, 36,
            -135,-127.5)

        stop_other = StopVehicle(
            self.other_vehicles[0],
            self._other_vehicle_max_brake)

        end_condition = InTriggerRegion(
            self.other_vehicles[0],
            31, 45,
            -138,-127.5)


        root_timeout = TimeOut(self.timeout)

        # Creating non-leaf nodes
        root = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        scenario_sequence = py_trees.composites.Sequence()
        keep_velocity_other_parallel = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        keep_velocity_other_parallel_2 = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        # Building tree
        root.add_child(scenario_sequence)
        root.add_child(root_timeout)
        # scenario_sequence.add_child(start_other_trigger)
        scenario_sequence.add_child(keep_velocity_other_parallel)
        scenario_sequence.add_child(keep_stoping_other)
        scenario_sequence.add_child(keep_velocity_other_parallel_2)
        scenario_sequence.add_child(stop_other)
        scenario_sequence.add_child(end_condition)
        keep_velocity_other_parallel.add_child(keep_velocity_other)
        keep_velocity_other_parallel.add_child(stop_other_trigger)

        keep_velocity_other_parallel_2.add_child(keep_velocity_other)
        keep_velocity_other_parallel_2.add_child(stop_other_trigger_2)

        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        # Adding checks for ego vehicle
        collision_criterion_ego = CollisionTest(self.ego_vehicle)
        # driven_distance_criterion = DrivenDistanceTest(
        #     self.ego_vehicle, self._ego_vehicle_driven_distance)
        criteria.append(collision_criterion_ego)
        # criteria.append(driven_distance_criterion)

        # Add approriate checks for other vehicles
        for vehicle in self.other_vehicles:
            collision_criterion = CollisionTest(vehicle)
            criteria.append(collision_criterion)

        return criteria
