#!/usr/bin/env python3
#    Copyright 2022 Marian Begemann
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
from ros2swarm.utils import setup_node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data


class MessagePump(Node):
    """
    Node used to transport topics from the ROS2Bridge to the ROS2Swarm domain for the Jackal robot
    """

    def __init__(self):
        """Initialize the node."""
        super().__init__('hardware_protection_layer')

        self.declare_parameters(
            namespace='',
            parameters=[
            ])

        self.name_space = self.create_subscription(
            Twist,
            self.get_namespace() + '/cmd_vel',
            self.command_callback,
            10)

        self.publisher_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data
        )

        self.publisher_scan = self.create_publisher(LaserScan, self.get_namespace() + '/scan', 10)


    def destroy_node(self):
        """Send a stop twist message and calls the super destroy method."""
        self.publisher_cmd_vel.publish(Twist())
        super().destroy_node()

    def command_callback(self, msg):
        self.publisher_cmd_vel.publish(msg)

    def scan_callback(self, scan_msg):
        self.publisher_scan.publish(scan_msg)


def main(args=None):
    setup_node.init_and_spin(args, MessagePump)


if __name__ == '__main__':
    main()
