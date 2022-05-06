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

import os
import argparse
import launch_ros.actions

from launch import LaunchDescription
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Creates the environment with gazebo, add add robots and starts their behaviour"""

    launch_file_dir = os.path.join(get_package_share_directory('launch_turtlebot_gazebo'))
    launch_pattern_dir = os.path.join(get_package_share_directory('ros2swarm'),
                                      'launch', 'pattern')
    launch_bringup_dir = os.path.join(get_package_share_directory('ros2swarm'))

    parser = argparse.ArgumentParser(description='Environment settings')
    parser.add_argument('-w', '--gazebo_world', type=str, default='',
                        help='Name of the gazebo world')
    parser.add_argument('-n', '--number_robots', type=int, default='',
                        help='The number of robots to spawn in the world')
    parser.add_argument('-p', '--pattern', type=str, default='',
                        help='The pattern executed by the robots')
    parser.add_argument('-l', '--log_level', type=str, default='',
                        help='The log level used in this execution')
    parser.add_argument('-v', '--version', type=str, default='',
                        help="ROS version used")
    parser.add_argument('-r', '--robot', type=str, default='',
                        help='The type of robot')

    args, unknown = parser.parse_known_args()
    world_file_name = args.gazebo_world
    number_robots = args.number_robots
    pattern = args.pattern
    log_level = args.log_level
    ros_version = args.version
    robot = args.robot

    print("world_file_name:", world_file_name)
    print("number of robots :", number_robots)
    print("pattern :", pattern)
    print("log level :", log_level)
    print("ros version :", ros_version)
    print("robot :", robot)

    ld = LaunchDescription()

    # Add the log level argument to the launch description
    log = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            "log_level",
            default_value=[log_level],
            description="Logging level",
        )
    ])
    ld.add_action(log)

    if ros_version == "2":
        # Add gazebo start script
        gazebo_start = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/start_gazebo.launch.py']),
            launch_arguments={'world_name': world_file_name}.items(),
        )
        ld.add_action(gazebo_start)

        for i in range(args.number_robots):
            # add gazebo node
            gazebo_node = launch_ros.actions.Node(
                package='launch_turtlebot_gazebo',
                node_executable='add_bot_node',
                node_namespace=['namespace_', str(i)],
                node_name=['gazeboTurtleBotNode_', str(i)],
                output='screen',
                arguments=[
                    '--robot_name', ['robot_name_', str(i)],
                    '--robot_namespace', ['robot_namespace_', str(i)],
                    '-x', '0.0',
                    '-y', [str(i), '.0'],
                    '-z', '0.1',
                    '-r', robot
                ]
            )
            ld.add_action(gazebo_node)

    # find out exact path of the patter launch file
    for i in range(args.number_robots):
        pattern_launch_file_name = pattern + '.launch.py'
        for root, dirs, files in os.walk(launch_pattern_dir):
            for name in files:
                if name == pattern_launch_file_name:
                    pattern_path = os.path.abspath(os.path.join(root, name))


        # add patterns
        launch_patterns = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [launch_bringup_dir, '/' + 'bringup_patterns.launch.py']),
            launch_arguments={'robot': robot,
                              'robot_namespace': ['robot_namespace_', str(i)],
                              'pattern': pattern_path,
                              'ros_version': ros_version}.items(),

        )
        ld.add_action(launch_patterns)

    return ld
