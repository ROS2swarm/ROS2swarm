#!/usr/bin/env python3
#    Copyright 2020 Vincent Jansen and Daniel Tidde
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
import launch_ros.actions
import argparse
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Start the nodes required for the majority rule pattern."""
    parser = argparse.ArgumentParser(description='Robot')
    parser.add_argument('-r', '--robot', type=str, default='burger',
                        help='The type of robot')
    args, unknown = parser.parse_known_args()
    robot = args.robot
    robot_namespace = LaunchConfiguration('robot_namespace', default='robot_namespace_default')
    log_level = LaunchConfiguration("log_level", default='debug')
    config_dir = os.path.join(get_package_share_directory('ros2swarm'), 'config', robot)

    ld = LaunchDescription()
    ros2_pattern_node = launch_ros.actions.Node(
        package='ros2swarm',
        node_executable='majority_rule_pattern',
        node_namespace=robot_namespace,
        output='screen',
        parameters=[
            os.path.join(config_dir, 'voting_pattern', 'basic', 'majority_rule_pattern.yaml')],
        arguments=[['__log_level:=', log_level]]
    )
    ld.add_action(ros2_pattern_node)

    return ld
