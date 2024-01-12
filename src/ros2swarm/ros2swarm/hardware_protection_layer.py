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
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ros2swarm.abstract_pattern import AbstractPattern
from ros2swarm.utils import setup_node
from communication_interfaces.msg import RangeData
from rclpy.qos import qos_profile_sensor_data
from ros2swarm.utils.scan_calculation_functions import ScanCalculationFunctions


class HardwareProtectionLayer(AbstractPattern):
    """
    Protects the robot from crashing into obstacles.

    pattern_node >> HardwareProtectionLayer >> publish twist message

    This is performed by listening to the self.get_namespace()/drive_command topic and
    publishing the message or an adjusted one in the case of
    obstacles to self.get_namespace()/cmd_vel
    """

    def __init__(self):
        """Initialize the node."""
        super().__init__('hardware_protection_layer')

        self.angles = None
        self.declare_parameters(
            namespace='',
            parameters=[
                ('hardware_protection_layer_max_range', 0.0),
                ('hardware_protection_layer_min_range', 0.0),
                ('hardware_protection_layer_front_attraction', 0.0),
                ('hardware_protection_layer_threshold', 0),
                ('max_translational_velocity', 0.0),
                ('max_rotational_velocity', 0.0)
            ])

        self.command_subscription = self.create_subscription(
            Twist,
            self.get_namespace() + '/drive_command',
            self.swarm_command_controlled(self.command_callback),
            10)

        self.current_angles = None
        self.current_ranges = None

        self.range_data_subscription = self.create_subscription(
            RangeData,
            self.get_namespace() + '/range_data',
            self.swarm_command_controlled(self.range_data_callback),
            qos_profile=qos_profile_sensor_data
        )

        self.publisher_cmd_vel = self.create_publisher(Twist,
                                                       self.get_namespace() + '/cmd_vel',
                                                       10)

        self.param_max_range = float(self.get_parameter(
            "hardware_protection_layer_max_range").get_parameter_value().double_value)
        self.param_min_range = self.get_parameter(
            "hardware_protection_layer_min_range").get_parameter_value().double_value
        self.param_front_attraction = self.get_parameter(
            "hardware_protection_layer_front_attraction").get_parameter_value().double_value
        self.param_threshold = self.get_parameter(
            "hardware_protection_layer_threshold").get_parameter_value().integer_value
        self.param_max_translational_velocity = self.get_parameter(
            "max_translational_velocity").get_parameter_value().double_value
        self.param_max_rotational_velocity = self.get_parameter(
            "max_rotational_velocity").get_parameter_value().double_value

    def destroy_node(self):
        """Send a stop twist message and calls the super destroy method."""
        self.publisher_cmd_vel.publish(Twist())
        super().destroy_node()

    def vector_calc(self):
        """
        Calculate an avoidance vector and if it is needed to avoid.

        Returns
        -------
        [avoid_needed{boolean}, direction{Twist}]

        """
        if self.current_ranges is None:
            return [False, None]

        avoid_distance = self.param_max_range
        direction, obstacle_free = ScanCalculationFunctions.potential_field(self.param_front_attraction,
                                                                            avoid_distance,
                                                                            self.param_max_rotational_velocity,
                                                                            self.param_max_translational_velocity,
                                                                            self.param_min_range,
                                                                            self.param_threshold,
                                                                            self.current_ranges,
                                                                            self.angles)
        avoid_needed = not obstacle_free

        return [avoid_needed, direction]

    def command_callback(self, msg):
        """
        Publish the received message or adjusts it.

        If it is needed to avoid an obstacle the avoid Twist message is published,
        the given command otherwise
        """
        self.get_logger().debug('heard: "%s"' % msg)

        [adjust, direction] = self.vector_calc()

        if adjust:
            msg = direction
            self.get_logger().debug('Adjusting to"%s"' % direction)

        self.publisher_cmd_vel.publish(msg)

    def range_data_callback(self, msg):
        """
        Check for every received scan if it is needed to avoid an obstacle.

        If needed publish avoid message, do nothing otherwise.
        """
        self.current_ranges = msg.ranges
        self.angles = msg.angles

        self.get_logger().debug('heard: "%s"' % msg)

        [adjust, direction] = self.vector_calc()

        if adjust:
            self.publisher_cmd_vel.publish(direction)
            self.get_logger().debug('Adjusting to"%s"' % direction)


def main(args=None):
    setup_node.init_and_spin(args, HardwareProtectionLayer)


if __name__ == '__main__':
    main()
