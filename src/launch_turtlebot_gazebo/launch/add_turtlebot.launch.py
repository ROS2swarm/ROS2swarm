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
# based on:
# https://discourse.ros.org/t/spawning-a-robot-entity-using-a-node-with-gazebo-and-ros-2/9985,
# at date 2020-06-08

import argparse
import launch_ros.actions
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    parser = argparse.ArgumentParser(description='Environment settings')
    parser.add_argument('-n', '--number_robots', type=int, default=1,
                        help='The number of robots to spawn in the world')
    parser.add_argument('-p', '--pattern', type=str, default='',
                        help='The pattern executed by the robots')
    parser.add_argument('-i', '--namespace_index', type=int, default=1,
                        help='The first used namespace index')
    parser.add_argument('-r', '--robot', type=str, default='',
                        help='The type of robot')
    args, unknown = parser.parse_known_args()
    number_robots = args.number_robots
    start_index = args.namespace_index
    pattern = args.pattern
    robot = args.robot
    config_dir = os.path.join(get_package_share_directory('ros2swarm'), 'config', robot)
    print("number of robots :", number_robots)
    print("pattern :", pattern)
    print("namespace_index :", start_index)

    ld = LaunchDescription()

    for i in (range(args.number_robots)):
        num = i + start_index
        # add gazebo node
        gazebo_node = launch_ros.actions.Node(
            package='launch_turtlebot_gazebo',
            node_executable='add_bot_node',
            node_namespace=['namespace_', str(num)],
            node_name=['gazeboTurtleBotNode_', str(num)],
            output='screen',
            arguments=[
                '--robot_name', ['robot_name_', str(num)],
                '--robot_namespace', ['robot_namespace_', str(num)],
                '-x', '0.0',
                '-y', [str(i), '.0'],
                '-z', '0.1'
                '-r', robot
            ]
        )
        ld.add_action(gazebo_node)

        # Add Hardware protection layer node
        ros2_hardware_protection_layer_node = launch_ros.actions.Node(
            package='ros2swarm',
            node_executable='hardware_protection_layer',
            node_namespace=['robot_namespace_', str(num)],
            output='screen',
            parameters=[os.path.join(config_dir, 'hardware_protection_layer' + '.yaml')]
        )
        ld.add_action(ros2_hardware_protection_layer_node)

        # add pattern
        ros2_pattern_node = launch_ros.actions.Node(
            package='ros2swarm',
            node_executable=args.pattern,
            node_namespace=['robot_namespace_', str(num)],
            output='screen',
        )
        ld.add_action(ros2_pattern_node)
    return ld
