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
import rclpy
import numpy as np

from rclpy.node import Node
from ros2swarm.utils import setup_node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Pose
from communication_interfaces.msg import RangeData
from std_msgs.msg import Header
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import euler_from_quaternion
from functools import partial


class IRTFLayer(Node):
    """
    Subscribe to a variable number of Range topics.
    The topics can be specified in
    ros2swarm/config/robot/specifications.yaml.
    
    Determine and publish a RangeData message on topic /range_data
    based on the received Range messages.
    
    Provide a service that identifies the rays defining a robot and
    for each robot the center ray for the current ranges.

    IRTFLayer >> HardwareProtectionLayer >> publish twist message
    or
    IRTFLayer >> pattern_node >> HardwareProtectionLayer >>
    publish twist message
    """

    def __init__(self):
        """Initialize the node."""
        super().__init__('ir_tf_layer') 
        # declare paramters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('range_topics', ''),
                ('max_range', 0.0),
                ('min_range', 0.0),
            ])
            
        # get the value of the parameters from config file
        self.range_topics = self.get_parameter(
            "range_topics").get_parameter_value().string_array_value
        self.param_max_range = float(
            self.get_parameter("max_range").get_parameter_value().double_value)
        self.param_min_range = self.get_parameter(
            "min_range").get_parameter_value().double_value

        # class variables
        self.current_ranges = [None] * len(self.range_topics)
        self.ir_angles = [None] * len(self.range_topics)
        self.angles = [None] * len(self.range_topics)
        
        # publishers and subscribers of sensor topics
        self.range_subs = [self.create_subscription(
                              Range,
                              self.get_namespace() + "" + self.range_topics[i],
                              partial(self.range_callback,index=i),
                              qos_profile=qos_profile_sensor_data)
                              for i in range(len(self.range_topics))]
                              
        # publisher for range data
        self.range_data_publisher = self.create_publisher(
            RangeData,
            self.get_namespace() + '/range_data',
            10)
            
        # variables for getting information from the tf tree
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def range_callback(self, range_msg, index):
        """
        Save the range in the array of current ranges and get the angle
        between the corresponding ray and the x-axis of the robot from 
        the tf tree.

        Publish a RangeData message on basis of the current ranges, if 
        at least one range measurement has already been recieved from
        each sensor.
        """
        # saves the received range
        self.current_ranges[index] = range_msg.range
        try:
            # gets transform between the robot's base and the frame of
            # the sensor that published the range message
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                'base_link',
                range_msg.header.frame_id,
                now)
            # gets the yaw angle of the transform    
            euler_angles = euler_from_quaternion([trans.transform.rotation.w, 
                                                  trans.transform.rotation.x, 
                                                  trans.transform.rotation.y, 
                                                  trans.transform.rotation.z])
            theta = euler_angles[2]
            # transformation matrix describing the transform between
            # the robot's base and the frame of the sensor that
            # published the range message
            t = np.matrix([[np.cos(theta), -np.sin(theta), trans.transform.translation.x],
                           [np.sin(theta), np.cos(theta), trans.transform.translation.y],
                           [0, 0, 1]])
            # transforms a point in the sensor's frame to a point in
            # the robot's base frame
            point = np.matrix([[range_msg.range], 
                               [0], 
                               [1]])
            transformed_point = np.matmul(t,
                                          point)
            # gets the angle of the polar coordinates of the
            # transformed point
            angle = np.arctan2(transformed_point.item(1), 
                               transformed_point.item(0))
            self.angles[index] = angle
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
                
        # publishes a range data and detects the robots in the ranges
        if not (None in self.current_ranges or None in self.angles):
            msg = RangeData()
            msg.header = range_msg.header
            msg.header.frame_id = "base_link"
            msg.ranges = self.current_ranges
            msg.angles = self.angles
            self.range_data_publisher.publish(msg)
            self.current_ranges = [None] * len(self.range_topics)


def main(args=None):
    """
    Create a node for the IRTF layer, spin it 
    and handle the destruction.
    """
    setup_node.init_and_spin(args, IRTFLayer)


if __name__ == '__main__':
    main()
