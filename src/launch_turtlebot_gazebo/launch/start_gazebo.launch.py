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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    """Starts a gazebo simulation in the mode, where elements could be added via command line"""

    world_name = LaunchConfiguration('world_name', default='arena_large.world')
    world_directory = os.path.join(get_package_share_directory('launch_turtlebot_gazebo'), 'worlds')
    #use_sim_time = LaunchConfiguration('use_sim_time', default='True')

    return LaunchDescription([
        DeclareLaunchArgument('world_name', description='The name of the world to load, default: turtle.world'),
         
        ExecuteProcess(
            # To start in paused mode add: '--pause'
            cmd=['gazebo', [world_directory, '/', world_name], '-s', 'libgazebo_ros_factory.so'],
            #cmd=['gazebo', '--verbose', [world_directory, '/', world_name], '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # died eventually at startup
        #ExecuteProcess(
        #   cmd=['ros2', 'param', 'set', '/gazebo', 'use_sim_time', use_sim_time],
        #   output='screen'
        # ),
    ])
