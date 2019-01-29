#!/usr/bin/env python

# Copyright (c) 2018 Intel Labs.
# authors: Fabian Oboril (fabian.oboril@intel.com)
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Welcome to CARLA following vehicle scenario.

This is an example code on how to use a scenario, the scenario manager
and how to evaluate scenario results.
"""

from __future__ import print_function
import argparse
from argparse import RawTextHelpFormatter

import carla
import roslaunch
import rospy
import time


from Scenarios.follow_leading_vehicle import *
from Scenarios.opposite_vehicle_taking_priority import *
from Scenarios.object_crash_vehicle import *
from Scenarios.no_signal_junction_crossing import NoSignalJunctionCrossing
from Scenarios.junction_crossing_risk_estimation import JunctionCrossingRisk
from Scenarios.object_crash_intersection import *
from Scenarios.control_loss import *
from Scenarios.dynamic_obj_detection import DynamicObjectDetect
from ScenarioManager.scenario_manager import ScenarioManager

# from risk_grid_validation import *


# Version of scenario_runner
VERSION = 0.1



# List of all supported scenarios. IMPORTANT: String has to be class name
SCENARIOS = {
    "FollowLeadingVehicle",
    "FollowLeadingVehicleWithObstacle",
    "StationaryObjectCrossing",
    "DynamicObjectCrossing",
    "OppositeVehicleRunningRedLight",
    "NoSignalJunctionCrossing",
    "VehicleTurningRight",
    "VehicleTurningLeft",
    "ControlLoss",
    "JunctionCrossingRisk",
    "DynamicObjectDetect"
}




DESCRIPTION = (
    "CARLA Scenario Runner: Setup, Run and Evaluate scenarios using CARLA\n"
    "Current version: " + str(VERSION))

PARSER = argparse.ArgumentParser(description=DESCRIPTION,
                                 formatter_class=RawTextHelpFormatter)
PARSER.add_argument('--host', default='localhost',
                    help='IP of the host server (default: localhost)')
PARSER.add_argument('--port', default='2000',
                    help='TCP port to listen to (default: 2000)')
PARSER.add_argument(
    '--debug', action="store_true", help='Run with debug output')
PARSER.add_argument(
    '--output', action="store_true", help='Provide results on stdout')
PARSER.add_argument('--filename', help='Write results into given file')
PARSER.add_argument(
    '--junit', help='Write results into the given junit file')
PARSER.add_argument('--scenario',
                    help='Name of the scenario to be executed')
PARSER.add_argument(
    '--repetitions', default=1, help='Number of scenario executions')
PARSER.add_argument(
    '--list', action="store_true", help='List all supported scenarios and exit')
PARSER.add_argument(
    '-v', '--version', action='version', version='%(prog)s ' + str(VERSION))

PARSER.add_argument('-s', '--save', action='store_true',
                    help='Save the traces')
ARGUMENTS = PARSER.parse_args()






class ScenarioRunner(object):


    def __init__(self, args):

        if args.list:
            print("Currently the following scenarios are supported:")
            print(*SCENARIOS, sep='\n')
            sys.exit(0)

        if args.scenario is None:
            print("Please specify a scenario using '--scenario SCENARIONAME'\n\n")
            PARSER.print_help(sys.stdout)
            sys.exit(0)

        # Tunable parameters
        client_timeout = 2.0   # in seconds
        wait_for_world = 10.0  # in seconds

        # CARLA world and scenario handlers
        self.world = None
        self.scenario = None
        self.manager = None

        # First of all, we need to create the client that will send the requests
        # to the simulator. Here we'll assume the simulator is accepting
        # requests in the localhost at port 2000.
        self.client = carla.Client(args.host, int(args.port))
        self.client.set_timeout(client_timeout)

        # Once we have a client we can retrieve the world that is currently
        # running.
        self.world = self.client.get_world()

        # Wait for the world to be ready
        self.world.wait_for_tick(wait_for_world)

        # Create scenario manager
        self.manager = ScenarioManager(self.world, args.debug)

        # Setup and run the scenario for repetition times
        self.scenario_class = self.get_scenario_class_or_fail(args.scenario)

        # rospy.init_node('en_Mapping', anonymous=True)
        self.collision_check = False

        self.args = args



    def run(self, params = None):

        try:
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/anshul/catkin_ws/src/carla_ros-bridge/client_cmcdot.launch"])
            launch.start()
            rospy.loginfo("carla_ros_bridge started")
            time.sleep(3)
            # rospy.sleep(2)

            self.scenario = self.scenario_class(self.world,debug_mode=False, params = params)
            self.manager.load_scenario(self.scenario)
            self.manager.run_scenario()


            if not self.manager.analyze_scenario():
                self.collision_check = False
                print("Success!")
            else:
                self.collision_check = True
                print("Failure!")

            launch.shutdown()
            time.sleep(2)
            rospy.loginfo("carla_ros_bridge killed")


            self.manager.stop_scenario()
            del self.scenario

        except:

            if not self.manager.analyze_scenario():
                self.collision_check = False
                print("Success!")
            else:
                self.collision_check = True
                print("Failure!")

            launch.shutdown()
            time.sleep(2)
            rospy.loginfo("carla_ros_bridge killed")
            self.manager.stop_scenario()
            del self.scenario




    def get_scenario_class_or_fail(self, scenario):
        """
        Get scenario class by scenario name
        If scenario is not supported or not found, raise an exception
        """
        if scenario in SCENARIOS:
            if scenario in globals():
                return globals()[scenario]
            else:
                raise Exception("No class for scenario '{}'".format(scenario))
        else:
            raise Exception("Scenario '{}' not supported".format(scenario))

    def __del__(self):
        if self.manager is not None:
            del self.manager
        if self.world is not None:
            del self.world





if __name__ == '__main__':
    runner = ScenarioRunner(ARGUMENTS)
    for i in range(3):
        runner.run()






# ARGUMENTS.scenario = "JunctionCrossingRisk"
# ARGUMENTS.scenario = "DynamicObjectDetect"


# def run_scenario(params):
#     global runner
#     runner.run(params)
#     return runner.collision_check




# def main(args):
#     """
#     Main function starting a CARLA client and connecting to the world.
#     """
#
#
#
#
#         for i in range(int(args.repetitions)):



    # finally:
    #     if manager is not None:
    #         del manager
    #     if world is not None:
    #         del world



# def run_scenario(scenario = "JunctionCrossingRisk"):
#     ARGUMENTS.scenario = scenario
#
#     if ARGUMENTS.list:
#         print("Currently the following scenarios are supported:")
#         print(*SCENARIOS, sep='\n')
#         sys.exit(0)
#
#     if ARGUMENTS.scenario is None:
#         print("Please specify a scenario using '--scenario SCENARIONAME'\n\n")
#         PARSER.print_help(sys.stdout)
#         sys.exit(0)
#
#     main(ARGUMENTS)
