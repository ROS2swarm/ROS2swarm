#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Darby Lim
# Modified by Marian Begemann based on:
# https://discourse.ros.org/t/giving-a-turtlebot3-a-namespace-for-multi-robot-experiments/10756,
# at date 09.07.2020

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    robot_type = LaunchConfiguration("robot", default='robot_type_default')
    usb_port = LaunchConfiguration('usb_port', default='/dev/ttyACM0')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    turtle_namespace = LaunchConfiguration('turtle_namespace', default='robot_namespace_NOT_SET')

    tb3_param_dir = LaunchConfiguration(
        'tb3_param_dir',
        default=PathJoinSubstitution([get_package_share_directory('ros2swarm'), 'param', PythonExpression(['"', robot_type, '"',' + ".yaml"'])]))
    # os.path.join(            get_package_share_directory('ros2swarm'), 'param', robot_type + '.yaml'))

    # lidar_pkg_dir = LaunchConfiguration(
    #     'lidar_pkg_dir',
    #     default=os.path.join(get_package_share_directory('hls_lfcd_lds_driver'), 'launch'))

    return LaunchDescription([
        DeclareLaunchArgument(
            'turtle_namespace',
            default_value=turtle_namespace,
            description='Use this as namespace for the turtlebot'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'usb_port',
            default_value=usb_port,
            description='Connected USB port with OpenCR'),

        DeclareLaunchArgument(
            'tb3_param_dir',
            default_value=tb3_param_dir,
            description='Full path to turtlebot3 parameter file to load'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/turtlebot3_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time,
                              'turtle_namespace': turtle_namespace}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/hlds_laser.launch.py']),
            # PythonLaunchDescriptionSource([lidar_pkg_dir, '/hlds_laser.launch.py']),
            launch_arguments={'port': '/dev/ttyUSB0',
                              'frame_id': 'base_scan',
                              'turtle_namespace': turtle_namespace
                              }.items(),
        ),

        Node(
            package='turtlebot3_node',
            executable='turtlebot3_ros',
            namespace=[turtle_namespace],
            parameters=[tb3_param_dir],
            arguments=['-i', usb_port],
            output='screen'),
    ])
