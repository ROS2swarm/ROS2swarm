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
import launch_ros.actions
import argparse
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Start the nodes required for the attraction pattern."""
    parser = argparse.ArgumentParser(description='Robot')
    parser.add_argument('-r', '--robot', type=str, default='',
                        help='The type of robot')
    args, unknown = parser.parse_known_args()
    robot = args.robot
    robot_namespace = LaunchConfiguration('robot_namespace', default='robot_namespace_default')
    # allows to use the same configuration files for each robot type but different mesh models
    robot_config = robot
    if robot_config.startswith('burger'):
        robot_config = "burger"
    elif robot_config.startswith('waffle_pi'):
        robot_config = "waffle_pi"
    config_dir = os.path.join(get_package_share_directory('ros2swarm'), 'config', robot_config)
    log_level = LaunchConfiguration("log_level", default='debug')

    ld = LaunchDescription()
    ros2_pattern_node = launch_ros.actions.Node(
        package='ros2swarm',
        node_executable='attraction_pattern2',
        node_namespace=robot_namespace,
        output='screen',
        parameters=[
            os.path.join(config_dir, 'movement_pattern', 'basic', 'attraction_pattern2.yaml')],
        arguments=[['__log_level:=', log_level]]
    )
    ld.add_action(ros2_pattern_node)

    return ld
