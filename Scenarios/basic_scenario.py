#!/usr/bin/env python

# Copyright (c) 2018 Intel Labs.
# authors: Fabian Oboril (fabian.oboril@intel.com)
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provide the basic class for all user-defined scenarios.
"""

from __future__ import print_function
import random
import sys

import py_trees
from carla import ColorConverter as cc
from ScenarioManager.scenario_manager import Scenario


def setup_vehicle(world, model, spawn_point, hero = False, zoe=False):
    """
    Function to setup the most relevant vehicle parameters,
    incl. spawn point and vehicle model.
    """
    blueprint_library = world.get_blueprint_library()

    # Get vehicle by model
    blueprint = random.choice(blueprint_library.filter(model))
    if zoe:
        blueprint.set_attribute('role_name', 'zoe')
    elif hero:
        blueprint.set_attribute('role_name', 'hero')
    else:
        blueprint.set_attribute('role_name', 'test_vehicle')

    vehicle = world.try_spawn_actor(blueprint, spawn_point)

    if vehicle is None:
        sys.exit(
            "Error: Unable to spawn vehicle {} at {}".format(model, spawn_point))

    # Let's put the vehicle to drive around
    vehicle.set_autopilot(False)

    return vehicle



def setup_sensor(world, sensor, transform, vehicle):
    """
    Function to setup the sensor suit.
    """
    blueprint_library = world.get_blueprint_library()

    if sensor == "lidar":
        # add sensors
        lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('rotation_frequency', '10')
        lidar_bp.set_attribute('channels', '32')
        lidar_bp.set_attribute('range', '5000')
        lidar_bp.set_attribute('points_per_second', '422000')
        lidar_bp.set_attribute('upper_fov', '2')
        lidar_bp.set_attribute('lower_fov', '-24.8')
        lidar = world.spawn_actor(lidar_bp, transform, attach_to=vehicle)
        print('created %s' % lidar.type_id)
        return lidar
    if sensor == "camera.rgb":
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', '720')
        camera_bp.set_attribute('image_size_y', '480')
        camera_bp.set_attribute('fov', '110')
        camera_rgb = world.spawn_actor(camera_bp, transform, attach_to=vehicle)
        return camera_rgb
    else:
        sys.exit(
            "Error: Unable to spawn sensor {}".format(sensor))






class BasicScenario(object):

    """
    Base class for user-defined scenario
    """

    _town = None            # Name of the map that is used
    name = None             # Name of the scenario
    criteria_list = []      # List of evaluation criteria
    timeout = 60            # Timeout of scenario in seconds
    scenario = None

    ego_vehicle = None
    other_vehicles = []
    camera_rgb = None
    lidar = None

    def __init__(self, name, town, world, debug_mode=False):
        """
        Setup all relevant parameters and create scenario
        and instantiate scenario manager
        """

        # Check if the CARLA server uses the correct map
        self._town = town
        self._check_town(world)

        self.name = name

        # Setup scenario
        if debug_mode:
            py_trees.logging.level = py_trees.logging.Level.DEBUG

        behavior = self._create_behavior()
        criteria = self._create_test_criteria()
        self.scenario = Scenario(
            behavior, criteria, self.name, self.timeout)

    def _create_behavior(self):
        """
        Pure virtual function to setup user-defined scenario behavior
        """
        raise NotImplementedError(
            "This function is re-implemented by all scenarios"
            "If this error becomes visible the class hierarchy is somehow broken")

    def _create_test_criteria(self):
        """
        Pure virtual function to setup user-defined evaluation criteria for the
        scenario
        """
        raise NotImplementedError(
            "This function is re-implemented by all scenarios"
            "If this error becomes visible the class hierarchy is somehow broken")

    def _check_town(self, world):
        if world.map_name != self._town:
            print("The CARLA server uses the wrong map!")
            print("This scenario requires to use map {}".format(self._town))
            sys.exit(-1)

    def __del__(self):
        """
        Cleanup.
        - Removal of the vehicles
        """
        actors = [self.ego_vehicle] + self.other_vehicles + [self.lidar] +[self.camera_rgb]
        for actor in actors:
            if actor is not None:
                actor.destroy()
                actor = None
