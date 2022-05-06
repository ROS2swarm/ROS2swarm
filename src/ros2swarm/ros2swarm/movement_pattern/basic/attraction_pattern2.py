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


class AttractionPattern2(MovementPattern):
    """
    Pattern to reach an group of the participating robots in the available area.

    The dispersion pattern node creates drive commands based on the laser scan messages it receive

    2 = using attraction_field instead of repulsion_field

    pattern_node >> publishing to the self.get_namespace()/drive_command topic
    """

    def __init__(self):
        """Initialize the attraction pattern node."""
        super().__init__('attraction_pattern2')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('attraction2_max_range', None),
                ('attraction2_min_range', None),
                ('attraction2_front_attraction', None),
                ('attraction2_threshold', None),
                ('attraction2_stop_if_alone', None),
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

        self.param_max_range = float(
            self.get_parameter("attraction2_max_range").get_parameter_value().double_value)
        self.param_min_range = self.get_parameter(
            "attraction2_min_range").get_parameter_value().double_value
        self.param_front_attraction = self.get_parameter(
            "attraction2_front_attraction").get_parameter_value().double_value
        self.param_threshold = self.get_parameter(
            "attraction2_threshold").get_parameter_value().integer_value
        self.param_stop_if_alone = self.get_parameter(
            "attraction2_stop_if_alone").get_parameter_value().bool_value
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

    def vector_calc(self, current_scan):
        """Calculate the direction vector for the current scan."""
        if current_scan is None:
            return Twist()

        direction, stop = ScanCalculationFunctions.attraction_field(
            self.param_front_attraction,
            self.param_max_range,
            self.param_max_rotational_velocity,
            self.param_max_translational_velocity,
            self.param_min_range,
            current_scan,
            self.param_threshold,
            self.lidar_config)

        if self.param_stop_if_alone:
            direction = Twist() if stop else direction
        return direction


def main(args=None):
    """Create a node for the attraction pattern, spins it and handles the destruction."""
    setup_node.init_and_spin(args, AttractionPattern2)


if __name__ == '__main__':
    main()
