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

import py_trees
import carla

from srunner.scenariomanager.atomic_scenario_behavior import *
from srunner.scenariomanager.atomic_scenario_criteria import *
from srunner.scenariomanager.timer import TimeOut
from srunner.scenarios.basic_scenario import *

PEDESTRIAN_CROSSING_RED_LIGHT_SCENARIOS = [
    "PedestrianCrossingRedLight"
]

class PedestrianCrossingRedLight(BasicScenario):

    """
    This class holds everything required for a scenario,
    in which a pedestrian cross the road at red pedestrian light
    after an other vehicule passed the intersection (while the ego
    vehicle has green)

    Location: Town03
    """

    timeout = 180            # Timeout of scenario in seconds

    # moving vehicles paramters
    _expected_driven_distance = 40
    _target_velocity = 13.9
    _security_distance = 10

    _cross_start_distance = 10
    _ego_vehicle_start = carla.Transform(carla.Location(x=-3.5, y=-157, z=1), carla.Rotation(yaw=90))

    _leading_vehicle_start = carla.Transform(carla.Location(x=-3.5, y=-150, z=1), carla.Rotation(yaw=90))

    _stopped_vehicle_start = carla.Transform(carla.Location(x=-13.3, y=-133, z=1), carla.Rotation(yaw=0))
   
    _intersection_location = carla.Location(x=-3.5, y=-150, z=0)
    _traffic_light_location = carla.Location(x=-11.5, y=-125.0, z=0.15)
    _traffic_light = None
    _location_of_collision = carla.Location(x=-2.8, y=-153, z=1)

    _lidar_location = carla.Transform(
        carla.Location(x=0, y=0, z=2.32),carla.Rotation(roll=0.74484513, pitch=0.85943669))
    _camera_location = carla.Transform(
        carla.Location(x= -4, y=0, z=3.5), carla.Rotation(pitch=-15))

    def __init__(self, world, ego_vehicle, other_actors, town, randomize=False, debug_mode=False, config=None):
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
        self.ego_vehicle = ego_vehicle
        self.leading_vehicle = other_actors[0]
        self.stopped_vehicle = other_actors[1]
        
        self.walker = other_actors[2]

        walker_start_loc = carla.Transform(carla.Location(x=7, y=-133, z=1), carla.Rotation(yaw=180))
        self.walker_control = carla.WalkerControl()
        self.walker_control_direction = 180 # yaw iin degrees
        self.walker_control_speed = 1.8 # m/s

        
        for actor in world.get_actors().filter('traffic.traffic_light'):
            if actor.get_location().distance(self._traffic_light_location) < 1.0:
                self._traffic_light = actor

        if self._traffic_light is None:
            print("No traffic light for the given location found")
            sys.exit(-1)

        super(PedestrianCrossingRedLight, self).__init__("PedestrianCrossingRedLight",
                                                ego_vehicle,
                                                other_actors,
                                                town,
                                                world,
                                                debug_mode)

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



        # wait until traffic light for ego vehicle is green
        wait_for_green = WaitForTrafficLightState(self._traffic_light, "Green")

        # Move the cars forward and make the pedestrian cross the road
        intersection_crossing = py_trees.composites.Parallel("Makes the actors cross the instersection", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)

        # Leading vehicle drive for _expected_driven_distance +10 m
        leading_keep_velocity_for_distance = py_trees.composites.Parallel("Keep leading vehicle velocity for distance", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        leading_keep_velocity = KeepVelocityPID(self.leading_vehicle, self._target_velocity)
        leading_keep_velocity_distance = DriveDistance(self.leading_vehicle, self._expected_driven_distance+10, name="Distance")
        leading_keep_velocity_for_distance.add_child(leading_keep_velocity)
        leading_keep_velocity_for_distance.add_child(leading_keep_velocity_distance)
        
        stop_leading_vehicle = StopVehicle(self.leading_vehicle, 1)
        
        leading_crossing = py_trees.composites.Sequence("First vehicle sequence cross")
        leading_crossing.add_child(leading_keep_velocity_for_distance)
        leading_crossing.add_child(stop_leading_vehicle)

        # Wait for _security_distance meters bewteen ego vehicle and leading vehicle then drive for _expected_driven_distance m
        ego_wait_for_security_distance = InTriggerDistanceToVehicle(self.leading_vehicle, self.ego_vehicle, self._security_distance, greater_than_distance=True)
      
        ego_keep_velocity_for_distance = py_trees.composites.Parallel("Keep ego velocity for distance", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        ego_keep_velocity = KeepVelocityPID(self.ego_vehicle,self._target_velocity)
        ego_keep_velocity_distance = DriveDistance(self.ego_vehicle, self._expected_driven_distance, name="Distance")
        ego_keep_velocity_for_distance.add_child(ego_keep_velocity)
        ego_keep_velocity_for_distance.add_child(ego_keep_velocity_distance)
        
        stop_ego_vehicle = StopVehicle(self.ego_vehicle, 1)
        
        ego_crossing = py_trees.composites.Sequence("Ego vehicle sequence cross")
        ego_crossing.add_child(ego_wait_for_security_distance)
        ego_crossing.add_child(ego_keep_velocity_for_distance)
        ego_crossing.add_child(stop_ego_vehicle)


        pedestrian_sync_crossing = py_trees.composites.Sequence("Sequence cross")
        cross_start_condition = InTriggerDistanceToLocation(self.leading_vehicle, self._location_of_collision, self._cross_start_distance)
        walker_crossing = KeepVelocity_pedestrian(self.walker, self.walker_control_speed, self.walker_control_direction)
        pedestrian_sync_crossing.add_child(cross_start_condition)
        pedestrian_sync_crossing.add_child(walker_crossing)
        
        intersection_crossing.add_children([leading_crossing, ego_crossing, pedestrian_sync_crossing])

        # Build behavior tree
        sequence = py_trees.composites.Sequence("Sequence Behavior")
        sequence.add_child(wait_for_green)
        sequence.add_child(intersection_crossing)

        return sequence

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
        collision_criterion = CollisionTest(self.leading_vehicle)
        driven_distance_criterion = DrivenDistanceTest(self.leading_vehicle, self._expected_driven_distance)
        criteria.append(velocity_criterion)
        # criteria.append(collision_criterion)
        criteria.append(driven_distance_criterion)

        # velocity_criterion = AverageVelocityTest(self.other_vehicles[1], 0)
        # collision_criterion = CollisionTest(self.other_vehicles[1])
        # criteria.append(velocity_criterion)
        # criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        self._traffic_light = None
        super(PedestrianCrossingRedLight, self).__del__()