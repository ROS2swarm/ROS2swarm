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

from geometry_msgs.msg import Twist
from ros2swarm.utils import setup_node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from ros2swarm.movement_pattern.movement_pattern import MovementPattern

from ros2swarm.utils.scan_calculation_functions import ScanCalculationFunctions
from communication_interfaces.msg import DoubleMessage


class DispersionPattern(MovementPattern):
    """
    Pattern to reach an distribution of the participating robots in the available area.

    The dispersion pattern node creates drive commands based on the laser scan messages it receive

    pattern_node >> publishing to the self.get_namespace()/drive_command topic
    """

    def __init__(self):
        """Initialize the dispersion pattern node."""
        super().__init__('dispersion_pattern')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('dispersion_max_range', None),
                ('dispersion_min_range', None),
                ('dispersion_front_attraction', None),
                ('dispersion_threshold', None),
                ('dispersion_stop_if_alone', None),
                ('dispersion_allow_dynamic_max_range_setting', False),
                ('max_translational_velocity', None),
                ('max_rotational_velocity', None),
                ('lidar_config', None)
            ])

        self.scan_subscription = self.create_subscription(
            LaserScan,
            self.get_namespace() + '/scan',
            self.swarm_command_controlled(self.scan_callback),
            qos_profile=qos_profile_sensor_data
        )

        self.max_range_subscription = self.create_subscription(
            DoubleMessage,
            self.get_namespace() + '/max_distance',
            self.max_range_callback,
            10
        )

        self.param_max_range = float(
            self.get_parameter("dispersion_max_range").get_parameter_value().double_value)
        self.param_min_range = self.get_parameter(
            "dispersion_min_range").get_parameter_value().double_value
        self.param_front_attraction = self.get_parameter(
            "dispersion_front_attraction").get_parameter_value().double_value
        self.param_threshold = self.get_parameter(
            "dispersion_threshold").get_parameter_value().integer_value
        self.param_stop_if_alone = self.get_parameter(
            "dispersion_stop_if_alone").get_parameter_value().bool_value
        self.param_allow_dynamic_max_range_setting = self.get_parameter(
            "dispersion_allow_dynamic_max_range_setting").get_parameter_value().bool_value
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

    def max_range_callback(self, message: DoubleMessage):
        if self.param_allow_dynamic_max_range_setting:
            # TODO check range is fitting
            self.param_max_range = float(message.data)

    def vector_calc(self, current_scan):
        """Calculate the direction vector for the current scan."""
        if current_scan is None:
            return Twist()

        direction, stop = ScanCalculationFunctions.potential_field(self.param_front_attraction, self.param_max_range,
                                                                   self.param_max_rotational_velocity,
                                                                   self.param_max_translational_velocity,
                                                                   self.param_min_range, current_scan,
                                                                   self.param_threshold, self.lidar_config)
        if self.param_stop_if_alone:
            direction = Twist() if stop else direction
        return direction


def main(args=None):
    """Create a node for the dispersion pattern and handle the setup."""
    setup_node.init_and_spin(args, DispersionPattern)


if __name__ == '__main__':
    main()
