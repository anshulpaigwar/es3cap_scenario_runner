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

from ScenarioManager.atomic_scenario_behavior import *
from ScenarioManager.atomic_scenario_criteria import *
from Scenarios.basic_scenario import *

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
    _expected_driven_distance = 100
    _target_velocity = 10
    _cross_start_distance = 2
    # ego vehicle parameters
    _ego_vehicle_model = 'vehicle.nissan.micra'
    _ego_vehicle_start = carla.Transform(
        carla.Location(x=-2.8, y=-200, z=1), carla.Rotation(yaw=90))

    # other vehicle
    _vehicle_before_model = 'vehicle.tesla.model3'
    _vehicle_before_start = carla.Transform(
        carla.Location(x=-2.8, y=-175, z=1), carla.Rotation(yaw=90))

    _vehicle_stopped_model = 'vehicle.tesla.model3'
    _vehicle_stopped_start = carla.Transform(
        carla.Location(x=-13.3, y=-133, z=1), carla.Rotation(yaw=0))
   
    _intersection_location = carla.Location(x=-3, y=-150, z=0)
    _traffic_light_location = carla.Location(x=-11.5, y=-125.0, z=0.15)
    _traffic_light = None
    _location_of_collision = carla.Location(x=-2.8, y=-133, z=1)

    def __init__(self, world, debug_mode=False):
        """
        Setup all relevant parameters and create scenario
        and instantiate scenario manager
        """
        self.other_vehicles =  [setup_vehicle(world,
                                             self._vehicle_before_model,
                                             self._vehicle_before_start),
                                setup_vehicle(world,
                                             self._vehicle_stopped_model,
                                             self._vehicle_stopped_start)]

        self.ego_vehicle = setup_vehicle(world,
                                         self._ego_vehicle_model,
                                         self._ego_vehicle_start,
                                         hero=True)
        # setup_sensor(world, sensor, transform, vehicle):

        for actor in world.get_actors().filter('traffic.traffic_light'):
            if actor.get_location().distance(self._traffic_light_location) < 1.0:
                self._traffic_light = actor

        if self._traffic_light is None:
            print("No traffic light for the given location found")
            sys.exit(-1)

        super(PedestrianCrossingRedLight, self).__init__(
            name="PedestrianCrossingRedLight",
            town="Town03",
            world=world,
            debug_mode=debug_mode)

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

       
        first_keep_velocity_for_distance = py_trees.composites.Parallel("Keep before velocity for distance", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        first_keep_velocity = KeepVelocity(self.other_vehicles[0], self._target_velocity)
        first_keep_velocity_distance = DriveDistance(self.other_vehicles[0], self._expected_driven_distance, name="Distance")
        first_keep_velocity_for_distance.add_child(first_keep_velocity)
        first_keep_velocity_for_distance.add_child(first_keep_velocity_distance)
        
        stop_first_vehicle = StopVehicle(self.other_vehicles[0], 1)
        
        first_crossing = py_trees.composites.Sequence("First vehicle sequence cross")
        first_crossing.add_child(first_keep_velocity_for_distance)
        first_crossing.add_child(stop_first_vehicle)

        
        ego_keep_velocity_for_distance = py_trees.composites.Parallel("Keep ego velocity for distance", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        ego_keep_velocity = KeepVelocity(self.ego_vehicle,self._target_velocity)
        ego_keep_velocity_distance = DriveDistance(self.ego_vehicle, self._expected_driven_distance, name="Distance")
        ego_keep_velocity_for_distance.add_child(ego_keep_velocity)
        ego_keep_velocity_for_distance.add_child(ego_keep_velocity_distance)
        
        stop_ego_vehicle = StopVehicle(self.ego_vehicle, 1)
        
        ego_crossing = py_trees.composites.Sequence("Ego vehicle sequence cross")
        ego_crossing.add_child(ego_keep_velocity_for_distance)
        ego_crossing.add_child(stop_ego_vehicle)


        pedestrian_sync_crossing = py_trees.composites.Sequence("Sequence cross")
        cross_start_condition = InTriggerDistanceToLocation(self.other_vehicles[0], self._location_of_collision, self._cross_start_distance)
        sync_arrival = SyncArrival(self.other_vehicles[1], self.ego_vehicle, self._location_of_collision)
        pedestrian_sync_crossing.add_child(cross_start_condition)
        pedestrian_sync_crossing.add_child(sync_arrival)
        
        intersection_crossing.add_children([before_keep_velocity_for_distance, ego_keep_velocity_for_distance, pedestrian_sync_crossing])

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

        velocity_criterion = AverageVelocityTest(self.other_vehicles[0], self._target_velocity)
        collision_criterion = CollisionTest(self.other_vehicles[0])
        driven_distance_criterion = DrivenDistanceTest(self.other_vehicles[0], self._expected_driven_distance)
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
