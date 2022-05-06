#!/usr/bin/env python3
#    Copyright 2020 Marian Begemann
#
#    Licensed under the Apache License, Version 2.0 (the "License");
#    you may not use this file except in compliance with the License.
#    You may obtain a copy of the License at
#
#        http://www.apache.org/licenses/LICENSE-2.0
#
#    Unless required by applicable law or agreed to in writing, software
#    distributed under the License is distributed on an "AS IS" BASIS,
#    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#    See the License for the specific language governing permissions and
#    limitations under the License.
import math

from geometry_msgs.msg import Twist
from ros2swarm.utils import setup_node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from ros2swarm.movement_pattern.movement_pattern import MovementPattern
from ros2swarm.utils.state import State
from ros2swarm.utils.scan_calculation_functions import ScanCalculationFunctions, ReductionOption


class AggregationPattern(MovementPattern):
    """
    Pattern to reach an aggregation of the participating robots in the available area.

    The aggregation pattern node creates drive commands based on the laser scan messages
    it receive.

    pattern_node >> publishing to the self.get_namespace()/drive_command topic
    """

    def __init__(self):
        """Initialize the aggregation pattern node."""
        super().__init__('aggregation_pattern')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('aggregation_max_range', None),
                ('aggregation_min_range', None),
                ('aggregation_front_attraction', None),
                ('aggregation_time_scale', None),
                ('aggregation_base_stay_time', None),
                ('aggregation_group_distance', None),
                ('aggregation_object_reduction', None),
                ('aggregation_group_reference_selection', None),
                ('aggregation_object_threshold', None),
                ('aggregation_object_min_width', None),
                ('aggregation_object_max_width', None),
                ('aggregation_stay_in_growing_groups', None),
                ('max_translational_velocity', None),
                ('max_rotational_velocity', None),
                ('lidar_config', None)
            ])
        self.state = State.INIT
        self.stay_counter_needs_init = True
        self.stay_counter = 0
        self.num_robots = 0

        self.scan_subscription = self.create_subscription(
            LaserScan,
            self.get_namespace() + '/scan',
            self.swarm_command_controlled(self.scan_callback),
            qos_profile=qos_profile_sensor_data
        )

        self.param_max_range = float(
            self.get_parameter("aggregation_max_range").get_parameter_value().double_value)
        self.param_min_range = self.get_parameter(
            "aggregation_min_range").get_parameter_value().double_value
        self.param_front_attraction = self.get_parameter(
            "aggregation_front_attraction").get_parameter_value().double_value
        self.param_time_scale = self.get_parameter(
            "aggregation_time_scale").get_parameter_value().double_value
        self.param_base_stay_time = self.get_parameter(
            "aggregation_base_stay_time").get_parameter_value().double_value
        self.param_group_distance = self.get_parameter(
            "aggregation_group_distance").get_parameter_value().double_value
        self.param_reduction = ReductionOption[self.get_parameter(
            "aggregation_object_reduction").get_parameter_value().string_value]
        self.param_group_reference_selection = ReductionOption[self.get_parameter(
            "aggregation_group_reference_selection").get_parameter_value().string_value]
        self.param_object_threshold = self.get_parameter(
            "aggregation_object_threshold").get_parameter_value().double_value
        self.param_object_min_width = self.get_parameter(
            "aggregation_object_min_width").get_parameter_value().integer_value
        self.param_object_max_width = self.get_parameter(
            "aggregation_object_max_width").get_parameter_value().integer_value
        self.param_stay_in_growing_groups = self.get_parameter(
            "aggregation_stay_in_growing_groups").get_parameter_value().bool_value
        self.param_max_translational_velocity = self.get_parameter(
            "max_translational_velocity").get_parameter_value().double_value
        self.param_max_rotational_velocity = self.get_parameter(
            "max_rotational_velocity").get_parameter_value().double_value
        # TODO replace magic number '3'
        self.lidar_config = self.get_parameter(
            "lidar_config").get_parameter_value().double_value if self.get_parameter(
            "lidar_config").get_parameter_value().type == 3 else None

    def scan_callback(self, incoming_msg):
        """Call back if a new scan msg is available."""
        direction = self.vector_calc(incoming_msg)
        self.command_publisher.publish(direction)

    def identify_robots(self, scan):
        """Identifies if a scan contains robots, based on the settings in the pattern parameters.

        :return: return the list of all found robots and a list in which each robot is represented by a single ray
         based on the chosen reduction method
        """
        robots, robots_center = ScanCalculationFunctions.identify_robots(laser_scan=scan,
                                                                         min_range=self.param_min_range,
                                                                         max_range=self.param_max_range,
                                                                         threshold=self.param_object_threshold,
                                                                         reduction=self.param_reduction,
                                                                         min_width=self.param_object_min_width,
                                                                         max_width=self.param_object_max_width)
        self.get_logger().debug('Found "{}" robots'.format(len(robots)))
        return robots, robots_center

    def explore(self, scan):
        """Explore State: Search for other robots by driving around."""
        robots, _ = self.identify_robots(scan)
        result = Twist()
        if robots:
            state = State.JOIN_GROUP
        else:
            state = State.EXPLORE
            result.linear.x = self.param_max_translational_velocity
        return result, state

    def join_group(self, scan):
        """Move towards a found group.
        JOIN_GROUP
        -> EXPLORE : no robots found anymore
        -> STAY_IN_GROUP : robots center within group distance
        -> JOIN_GROUP : else, moving towards group.
        """

        robots, robots_center = self.identify_robots(scan)
        result = Twist()
        if not robots:
            state = State.EXPLORE
        elif ScanCalculationFunctions.object_nearer_then(centered_object_list=robots_center,
                                                         distance=self.param_group_distance):
            state = State.STAY_IN_GROUP
        else:
            # TODO move towards group
            result = ScanCalculationFunctions.create_twist_towards_objects(
                centered_object_list=robots_center,
                max_rotational_velocity=self.param_max_rotational_velocity,
                max_translational_velocity=self.param_max_translational_velocity,
                max_range=self.param_max_range,
                front_attraction=self.param_front_attraction,
                select_target=self.param_group_reference_selection)
            state = State.JOIN_GROUP
        return result, state

    def stay_in_group(self, scan):
        """The robot stay in a group by not moving.
        In the first iteration a counter calculates how long to stay in place depending on the pattern parameters and
        the number of found robots.
        STAY_IN_GROUP
        -> STAY_IN_GROUP : counter greater zero
        -> LEAVE_GROUP : else / if param_stay_in_growing_groups is true and more robots than before,
                         reset counter instead
        """
        robots, _ = self.identify_robots(scan)
        result = Twist()
        self.get_logger().debug('Turtle "{}" counter {} {}'.format(self.get_namespace(), self.stay_counter, len(robots)))
        if self.stay_counter_needs_init:
            self.stay_counter = int(int(math.pow(2, len(robots)) * self.param_time_scale) + self.param_base_stay_time)
            self.get_logger().debug('Turtle "{}" math {} {} {}'.format(self.get_namespace(), math.pow(2, len(robots)), len(robots), self.param_time_scale))
            self.stay_counter_needs_init = False
            self.num_robots = len(robots)

        if self.stay_counter > 0:
            state = State.STAY_IN_GROUP
            self.stay_counter -= 1
        else:
            self.stay_counter_needs_init = True
            if self.param_stay_in_growing_groups and self.num_robots < len(robots):
                state = State.STAY_IN_GROUP
                self.get_logger().debug('Turtle "{}" re-set timer {} {}'.format(self.get_namespace(), self.num_robots, len(robots)))
            else:
                self.get_logger().debug('Turtle "{}" leave group {} {}'.format(self.get_namespace(), self.num_robots, len(robots)))
                state = State.LEAVE_GROUP

        return result, state

    def leave_group(self, scan):
        """The robot leaves a group by moving away from the robots in range.
        LEAVE_GROUP
        -> LEAVE_GROUP : at least one robot still in range
        -> EXPLORE : else """
        robots, robots_center = self.identify_robots(scan)
        result = Twist()
        if robots:
            # As log as robots are within detection range move way from them
            state = State.LEAVE_GROUP
            result = ScanCalculationFunctions.create_twist_from_objects(
                centered_object_list=robots_center,
                max_rotational_velocity=self.param_max_rotational_velocity,
                max_translational_velocity=self.param_max_translational_velocity,
                max_range=self.param_max_range,
                front_attraction=self.param_front_attraction,
                select_target=self.param_group_reference_selection)
        else:
            state = State.EXPLORE
        return result, state

    def vector_calc(self, current_scan):
        """Calculate the direction vector for the current scan.

        # INIT: determine if next state is EXPLORE or JOIN_GROUP
        # EXPLORE: -> move forward
        # STAY: -> num robots
        # JOIN_GROUP: -> robot moves towards center of mass [?? to nearest]
        # LEAVE_GROUP: -> robot moves away from center of mass # also has timer OR until no bots detected
        """
        if current_scan is None:
            return Twist()

        self.get_logger().debug('Turtle "{}" is in state "{}"'.format(self.get_namespace(), self.state))
        result = Twist()

        if self.state is State.INIT:
            robots, _ = self.identify_robots(current_scan)
            if robots:
                self.state = State.JOIN_GROUP
            else:
                self.state = State.EXPLORE
        elif self.state is State.EXPLORE:
            result, self.state = self.explore(current_scan)
        elif self.state is State.JOIN_GROUP:
            result, self.state = self.join_group(current_scan)
        elif self.state is State.STAY_IN_GROUP:
            result, self.state = self.stay_in_group(current_scan)
        elif self.state is State.LEAVE_GROUP:
            result, self.state = self.leave_group(current_scan)

        return result


def main(args=None):
    """Create a node for the aggregation pattern, spins it and handles the destruction."""
    setup_node.init_and_spin(args, AggregationPattern)


if __name__ == '__main__':
    main()
