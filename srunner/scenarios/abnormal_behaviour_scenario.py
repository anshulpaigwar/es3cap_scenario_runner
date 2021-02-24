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

from srunner.scenariomanager.atomic_scenario_behavior import *
from srunner.scenariomanager.atomic_scenario_criteria import *
from srunner.scenarios.basic_scenario import *
from srunner.scenariomanager.timer import TimeOut

ABNORMAL_BEHAVIOUR_RISK = [
    "AbnormalBehaviourRisk",
    "AbnormalBraking"
]


class AbnormalBehaviourRisk(BasicScenario):

    """
    Implementation class for
    'Non-signalized junctions: crossing negotiation' scenario,
    Traffic Scenario 10.

    Location    :   Town03
    """
    category = "AbnormalBehaviourRisk"

    _ego_vehicle_max_velocity = 10
    _ego_vehicle_max_brake = 1.0

    _other_vehicle_max_brake = 1.0
    _other_vehicle_target_velocity = 10

    def __init__(self, world, ego_vehicle, other_actors, town, randomize=False, debug_mode=False, params=None):
        """
        Setup all relevant parameters and create scenario
        """

        if params is not None:
            self._ego_vehicle_max_velocity = params.ego_vehicle_vel
            self._other_vehicle_target_velocity = params.other_vehicle_vel

        super(AbnormalBehaviourRisk, self).__init__("AbnormalBehaviourRisk",
                                                       ego_vehicle,
                                                       other_actors,
                                                       "Town03",
                                                       world,
                                                       debug_mode)


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

        keep_velocity_ego = KeepVelocity_fixed(
            self.ego_vehicle,
            self._ego_vehicle_max_velocity)

        go_to_target_other = BasicAgentBehavior(
            actor=self.other_actors[0],
            target_location= carla.Location(x=15, y=200, z=3.5))
            # target_location= carla.Location(x=17.9, y=195.0, z=3.5))

        auto_pilot_other = UseAutoPilot(actor=self.other_actors[0])

        # keep_velocity_other = KeepVelocityPID(
        #     self.other_actors[0],
        #     self._other_vehicle_target_velocity)

        # self.other_actors[1].set_velocity(carla.Vector3D(-self._other_vehicle_target_velocity,0,0))
        keep_velocity_other_1 = KeepVelocityPID(
            self.other_actors[1],
            self._other_vehicle_target_velocity-1.5)



        stop_other = StopVehicle(
            self.other_actors[0],
            self._other_vehicle_max_brake)

        stop_ego_trigger = InTriggerRegion(
            self.ego_vehicle,
             8, 16,
            190, 207)

        end_condition = InTriggerRegion(
            self.ego_vehicle,
             -5, 16,
            190, 207)

        stop_ego = StopVehicle(
            self.ego_vehicle,
            self._ego_vehicle_max_brake)


        root_timeout = TimeOut(self.timeout)

        # Creating non-leaf nodes
        root = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        scenario_sequence = py_trees.composites.Sequence()
        keep_velocity_parallel = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        keep_velocity_ego_parallel = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        keep_velocity_other_parallel = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        end_condition_parallel = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        stop_ego_sequence = py_trees.composites.Sequence()
        stop_other_sequence = py_trees.composites.Sequence()

        # Building tree
        root.add_child(scenario_sequence)
        root.add_child(root_timeout)
        # scenario_sequence.add_child(start_other_trigger)
        scenario_sequence.add_child(keep_velocity_parallel)
        scenario_sequence.add_child(end_condition)
        keep_velocity_parallel.add_child(go_to_target_other)
        keep_velocity_parallel.add_child(keep_velocity_other_1)
        # keep_velocity_parallel.add_child(auto_pilot_other)
        keep_velocity_parallel.add_child(stop_ego_sequence)

        stop_ego_sequence.add_child(keep_velocity_ego_parallel)
        stop_ego_sequence.add_child(stop_ego)

        # stop_other_sequence.add_child(keep_velocity_other_parallel)
        # stop_other_sequence.add_child(stop_other)

        keep_velocity_ego_parallel.add_child(keep_velocity_ego)
        keep_velocity_ego_parallel.add_child(stop_ego_trigger)
        # keep_velocity_other_parallel.add_child(keep_velocity_other)
        # keep_velocity_other_parallel.add_child(stop_other_trigger)


        # end_condition_parallel.add_child(stop_ego_sequence)
        # end_condition_parallel.add_child(stop_other_sequence)


        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        # Adding checks for ego vehicle
        collision_criterion_ego = CollisionTest(self.ego_vehicle, terminate_on_failure=True)
        # driven_distance_criterion = DrivenDistanceTest(
        #     self.ego_vehicle, self._ego_vehicle_driven_distance)
        criteria.append(collision_criterion_ego)
        # criteria.append(driven_distance_criterion)

        # Add approriate checks for other vehicles
        for vehicle in self.other_actors:
            collision_criterion = CollisionTest(vehicle)
            criteria.append(collision_criterion)

        return criteria




class AbnormalBraking(BasicScenario):

    """
    Implementation class for
    'Non-signalized junctions: crossing negotiation' scenario,
    Traffic Scenario 10.

    Location    :   Town03
    """
    category = "AbnormalBehaviourRisk"

    _ego_vehicle_max_velocity = 10
    _ego_vehicle_max_brake = 1.0

    _other_vehicle_max_brake = 1.0
    _other_vehicle_target_velocity = 10

    def __init__(self, world, ego_vehicle, other_actors, town, randomize=False, debug_mode=False, params=None):
        """
        Setup all relevant parameters and create scenario
        """

        if params is not None:
            self._ego_vehicle_max_velocity = params.ego_vehicle_vel
            self._other_vehicle_target_velocity = params.other_vehicle_vel

        super(AbnormalBraking, self).__init__("AbnormalBraking",
                                                       ego_vehicle,
                                                       other_actors,
                                                       "Town03",
                                                       world,
                                                       debug_mode)


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

        keep_velocity_ego = KeepVelocity(
            self.ego_vehicle,
            self._ego_vehicle_max_velocity)

        go_to_target_other = BasicAgentBehavior(
            actor=self.other_actors[0],
            target_location= carla.Location(x=17.9, y=195.0, z=3.5))

        keep_velocity_other_1 = KeepVelocity(
            self.other_actors[1],
            self._other_vehicle_target_velocity - 3.5)


        stop_ego_trigger = InTriggerRegion(
            self.ego_vehicle,
             8, 16,
            190, 207)

        stop_other_1_trigger = InTriggerDistanceToVehicle(
            self.ego_vehicle,
            self.other_actors[1],
            15)

        end_condition = InTriggerRegion(
            self.ego_vehicle,
             -5, 16,
            190, 207)

        stop_ego = StopVehicle(
            self.ego_vehicle,
            self._ego_vehicle_max_brake)

        stop_other = StopVehicle(
            self.other_actors[1],
            self._other_vehicle_max_brake)

        root_timeout = TimeOut(self.timeout)

        # Creating non-leaf nodes
        root = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        scenario_sequence = py_trees.composites.Sequence()
        keep_velocity_parallel = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
        keep_velocity_ego_parallel = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        keep_velocity_other_1_parallel = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        end_condition_parallel = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        ego_sequence = py_trees.composites.Sequence()
        other_1_sequence = py_trees.composites.Sequence()

        # Building tree
        root.add_child(keep_velocity_parallel)
        root.add_child(end_condition)
        root.add_child(root_timeout)
        # scenario_sequence.add_child(start_other_trigger)

        # scenario_sequence.add_child(keep_velocity_parallel)
        # scenario_sequence.add_child(end_condition)

        keep_velocity_parallel.add_child(go_to_target_other)
        keep_velocity_parallel.add_child(ego_sequence)
        keep_velocity_parallel.add_child(other_1_sequence)

        ego_sequence.add_child(keep_velocity_ego_parallel)
        ego_sequence.add_child(stop_ego)

        other_1_sequence.add_child(keep_velocity_other_1_parallel)
        other_1_sequence.add_child(stop_other)

        keep_velocity_ego_parallel.add_child(keep_velocity_ego)
        keep_velocity_ego_parallel.add_child(stop_ego_trigger)

        keep_velocity_other_1_parallel.add_child(keep_velocity_other_1)
        keep_velocity_other_1_parallel.add_child(stop_other_1_trigger)

        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        # Adding checks for ego vehicle
        collision_criterion_ego = CollisionTest(self.ego_vehicle, terminate_on_failure=True)
        # driven_distance_criterion = DrivenDistanceTest(
        #     self.ego_vehicle, self._ego_vehicle_driven_distance)
        criteria.append(collision_criterion_ego)
        # criteria.append(driven_distance_criterion)

        # Add approriate checks for other vehicles
        for vehicle in self.other_actors:
            collision_criterion = CollisionTest(vehicle)
            criteria.append(collision_criterion)

        return criteria
