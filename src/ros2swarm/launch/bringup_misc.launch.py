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
import launch.substitutions
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    """Bring up miscellaneous nodes that are not patterns or robots (like items_master)."""

    log_level = LaunchConfiguration("log_level", default='debug')
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')

    ld = LaunchDescription()

    # add items_master_node
    items_master_node = launch_ros.actions.Node(
        package='ros2swarm',
        executable='items_master_node',
        namespace='items_master_namespace',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level]
    )
    ld.add_action(items_master_node)

    return ld
