#!/usr/bin/env python3
#
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

import os
import sys
import launch_ros.actions
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """Add turtlebot(s) to an already running gazebo simulation."""

    for arg in sys.argv:
        if arg.startswith("start_index:="):  # The start index for the robots number
            start_index = int(arg.split(":=")[1])
        if arg.startswith("number_robots:="):  # The number of robots to spawn in the world
            number_robots = int(arg.split(":=")[1])
        elif arg.startswith("pattern:="):  # The pattern executed by the robots
            pattern = arg.split(":=")[1]
        elif arg.startswith("log_level:="):  # The log level used in this execution
            log_level = arg.split(":=")[1]
        elif arg.startswith("robot:="):  # The type of robot
            robot = arg.split(":=")[1]
        elif arg.startswith("version:="):  # ROS version used
            version = int(arg.split(":=")[1])
        else:
            if arg not in ['/opt/ros/foxy/bin/ros2',
                           'launch',
                           'launch_turtlebot_gazebo',
                           'add_turtlebot.launch.py']:
                print("Argument not known: '", arg, "'")

    print("number of robots :", number_robots)
    print("pattern :", pattern)
    print("log level :", log_level)
    print("robot :", robot)
    print("namespace_index :", start_index)
    print("version :", version)

    ros_version = version

    ld = LaunchDescription()

    for i in (range(number_robots)):
        num = i + start_index
        # add gazebo node
        gazebo_node = launch_ros.actions.Node(
            package='launch_turtlebot_gazebo',
            executable='add_bot_node',
            namespace=['namespace_', str(num)],
            name=['gazeboTurtleBotNode_', str(num)],
            output='screen',
            arguments=[
                '--robot_name', ['robot_name_', str(num)],
                '--robot_namespace', ['robot_namespace_', str(num)],
                '-x', '1.0',
                '-y', [str(i), '.0'],
                '-z', '0.1',
                '--type_of_robot', robot
            ]
        )
        ld.add_action(gazebo_node)

    # allows to use the same configuration files for each robot type but different mesh models
    robot_type = robot
    if robot_type.startswith('burger'):
        robot_type = "burger"
    elif robot_type.startswith('waffle_pi'):
        robot_type = "waffle_pi"

    print("robot configuration:", robot_type)

    config_dir = os.path.join(get_package_share_directory('ros2swarm'), 'config', robot_type)

    urdf_file_name = 'turtlebot3_' + robot + '.urdf'
    urdf_file = os.path.join(get_package_share_directory('turtlebot3_description'), 'urdf', urdf_file_name)
    launch_pattern_dir = os.path.join(get_package_share_directory('ros2swarm'), 'launch', 'pattern')
    launch_bringup_dir = os.path.join(get_package_share_directory('ros2swarm'))

    # find out exact path of the patter launch file
    for i in (range(number_robots)):
        num = i + start_index
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
                              'robot_type': robot_type,
                              'robot_namespace': ['robot_namespace_', str(num)],
                              'pattern': pattern_path,
                              'config_dir': config_dir,
                              'urdf_file': urdf_file,
                              'ros_version': str(ros_version)}.items(),

        )
        ld.add_action(launch_patterns)

    return ld
