#!/usr/bin/env python3
#    Copyright 2021 Marian Begemann
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
import launch_ros.actions
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    """Start the nodes required for the flocking pattern."""
    robot_namespace = LaunchConfiguration('robot_namespace', default='robot_namespace_default')
    config_dir = LaunchConfiguration('config_dir', default='config_dir_default')
    log_level = LaunchConfiguration("log_level", default='debug')

    ld = LaunchDescription()
    ros2_pattern_node = launch_ros.actions.Node(
        package='ros2swarm',
        executable='discussed_dispersion_pattern',
        namespace=robot_namespace,
        output='screen',
        parameters=[
            PathJoinSubstitution([config_dir, 'movement_pattern', 'combined', 'discussed_dispersion_pattern.yaml'])],
        arguments=['--ros-args', '--log-level', log_level]
    )
    ld.add_action(ros2_pattern_node)

    ros2_pattern_subnode_dispersion = launch_ros.actions.Node(
        package='ros2swarm',
        executable='dispersion_pattern',
        namespace=robot_namespace,
        output='screen',
        parameters=[
            PathJoinSubstitution([config_dir, 'movement_pattern', 'combined', 'discussed_dispersion_pattern.yaml'])],
        remappings=[
            (['/', robot_namespace, '/max_distance'],
             ['/', robot_namespace, '/dispersion_distance']),
            (['/', robot_namespace, '/drive_command'],
             ['/', robot_namespace, '/drive_command_dispersion_pattern']),
        ],
        arguments=['--ros-args', '--log-level', log_level]
    )
    ld.add_action(ros2_pattern_subnode_dispersion)

    ros2_pattern_subnode_majority_rule = launch_ros.actions.Node(
        package='ros2swarm',
        executable='majority_rule_pattern',
        namespace=robot_namespace,
        output='screen',
        parameters=[
            PathJoinSubstitution([config_dir, 'movement_pattern', 'combined', 'discussed_dispersion_pattern.yaml'])],
        arguments=['--ros-args', '--log-level', log_level]
    )
    ld.add_action(ros2_pattern_subnode_majority_rule)

    return ld
