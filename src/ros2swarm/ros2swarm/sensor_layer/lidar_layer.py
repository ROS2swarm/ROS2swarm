#!/usr/bin/env python3
#    Copyright 2020 Josephine Brauer, Tanja Kaiser
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
from ros2swarm.utils import setup_node
from sensor_msgs.msg import LaserScan
from communication_interfaces.msg import RangeData
from rclpy.qos import qos_profile_sensor_data
from ros2swarm.utils.scan_calculation_functions import ScanCalculationFunctions, ReductionOption

class LidarLayer(Node):
    """
    Subscribe to a LaserScan topic.
    
    Determine and publish a RangeData message on topic /range_data
    based on the received LaserScan message.
    
    Provide a service that identifies the rays defining a robot and
    for each robot the center ray for the current scan.

    LidarLayer >> HardwareProtectionLayer >> publish twist message
    or
    LidarLayer >> pattern_node >> HardwareProtectionLayer
    >> publish twist message
    """

    def __init__(self):
        """Initialize the node."""
        super().__init__('lidar_layer')
        
        # declares parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('max_range', 0.0),
                ('min_range', 0.0),
                ('object_reduction', ''),
                ('object_threshold', 0.0),
                ('object_min_width', 0),
                ('object_max_width', 0),
            ])
            
        # gets parameter values from config file
        self.param_max_range = float(
            self.get_parameter("max_range").get_parameter_value().double_value)
        self.param_min_range = self.get_parameter(
            "min_range").get_parameter_value().double_value
        self.param_reduction = self.get_parameter(
            "object_reduction").get_parameter_value().string_value
        self.param_object_threshold = self.get_parameter(
            "object_threshold").get_parameter_value().double_value
        self.param_object_min_width = self.get_parameter(
            "object_min_width").get_parameter_value().integer_value
        self.param_object_max_width = self.get_parameter(
            "object_max_width").get_parameter_value().integer_value
        self.scan = None
        
        # sensor message publishers and subsribers
        self.scan_subscription = self.create_subscription(
            LaserScan,
            self.get_namespace() + '/scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data)
            
        # publisher for range data
        self.range_data_publisher = self.create_publisher(
            RangeData,
            self.get_namespace() + '/range_data',
            10)

    def scan_callback(self, scan_msg):
        """
        Save the received scan.

        Publish a RangeData message on basis of the current scan.
        """
        
        self.scan = scan_msg
        
        # array of angles between the rays of the scan and the x-axis
        # of the robot
        angle_min = scan_msg.angle_min
        angle_increment = scan_msg.angle_increment
        angles = [(angle_min + i*angle_increment) for i in range(len(scan_msg.ranges))]
        
        # publish range data message
        msg = RangeData()
        msg.header = scan_msg.header
        msg.ranges = scan_msg.ranges
        msg.angles = angles
        self.range_data_publisher.publish(msg)

def main(args=None):
    """
    Create a node for the LiDAR Layer, spin it and handle the
    destruction.
    """
    setup_node.init_and_spin(args, LidarLayer)


if __name__ == '__main__':
    main()
