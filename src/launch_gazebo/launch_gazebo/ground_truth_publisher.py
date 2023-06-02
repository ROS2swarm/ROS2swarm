#!/usr/bin/env python3
#    Copyright 2023 Tanja Kaiser
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
import functools
import rclpy

from rclpy.node import Node
from ros2swarm.utils import setup_node
from communication_interfaces.msg import ModelStatesStamped
from rclpy.qos import qos_profile_sensor_data
from gazebo_msgs.msg import ModelStates 

class GroundTruthPublisher(Node):


    def __init__(self):
        """Initialize the node."""
        super().__init__('ground_truth_publisher') 
    
        
        # publishers and subscribers of sensor topics
        self.pose_subscription = [self.create_subscription(
                                       ModelStates,
                                       "/gazebo/model_states",
                                       self.callback,
                                       qos_profile=qos_profile_sensor_data)]
        
        # publisher for range data
        self.data_publisher = self.create_publisher(
            ModelStatesStamped,
            '/ground_truth',
            100)

    def callback(self, received):
        """
        Save the range in the array of current ranges and get the angle
        between the corresponding ray and the x-axis of the robot from
        the specifications file.

        Publish a RangeData message on basis of the current ranges, if
        at least one range measurement has already been received from
        each sensor.
        """
        

        msg = ModelStatesStamped()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.name = received.name
        msg.pose = received.pose
        msg.twist = received.twist
        self.data_publisher.publish(msg)


def main(args=None):
    """
    Create a node for the IR Layer, spin it and handle the
    destruction.
    """
    setup_node.init_and_spin(args, GroundTruthPublisher)


if __name__ == '__main__':
    main()
