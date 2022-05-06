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
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Bring up all needed hardware protection patterns and the behaviour pattern."""
    parser = argparse.ArgumentParser(description='Environment settings')
    parser.add_argument('-r', '--robot', type=str, default='waffle_pi',
                        help='The type of robot')
    args, unknown = parser.parse_known_args()
    robot = args.robot

    # allows to use the same configuration files for each robot type but different mesh models
    robot_config = robot
    if robot_config.startswith('burger'):
        robot_config = "burger"
    elif robot_config.startswith('waffle_pi'):
        robot_config = "waffle_pi"

    print("robot configuration:", robot_config)

    config_dir = os.path.join(get_package_share_directory('ros2swarm'), 'config', robot_config)
    robot_namespace = LaunchConfiguration('robot_namespace', default='robot_namespace_default')
    pattern = LaunchConfiguration('pattern', default='pattern_default')
    log_level = LaunchConfiguration("log_level", default='debug')




    ld = LaunchDescription()
    # Add Hardware protection layer
    ros2_hardware_protection_layer_node = launch_ros.actions.Node(
        package='ros2swarm',
        node_executable='hardware_protection_layer',
        node_namespace=robot_namespace,
        output='screen',
        parameters=[os.path.join(config_dir, 'hardware_protection_layer' + '.yaml')],
        arguments=[['__log_level:=', log_level]]
    )
    ld.add_action(ros2_hardware_protection_layer_node)

    # add pattern
    launch_pattern = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([pattern]),
        launch_arguments={'robot': robot_config,
                          'robot_namespace': [robot_namespace]}.items(),

    )
    ld.add_action(launch_pattern)

    if not robot_config.startswith('jackal'):

        use_sim_time = LaunchConfiguration('use_sim_time', default='True')
        declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
            'use_robot_state_pub',
            default_value='True',
            description='Whether to start the robot state publisher')
        ld.add_action(declare_use_robot_state_pub_cmd)

        #TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
        #urdf_file_name = 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'
        urdf_file_name = 'turtlebot3_' + robot + '.urdf'
        print('urdf_file_name : {}'.format(urdf_file_name))
        urdf = os.path.join(
            get_package_share_directory('turtlebot3_description'),
            'urdf',
            urdf_file_name)

        # TODO test if this model also works
        # urdf = os.path.join(
        #    get_package_share_directory("turtlebot3_gazebo"), "models",
        #    "turtlebot3_" + TURTLEBOT3_MODEL,
        #    "model.sdf")

        # add state publisher
        robot_state_publisher = launch_ros.actions.Node(
            package='robot_state_publisher',
            node_executable='robot_state_publisher',
            node_namespace=robot_namespace,
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[urdf]
            # TODO robot_state_publisher supports log level from ros2 foxy onwards
            # arguments=[['__log_level:=', log_level], urdf]
        )

        # start_robot_state_publisher_cmd = Node(
        #    condition=IfCondition(use_robot_state_pub),
        #    package='robot_state_publisher',
        #    executable='robot_state_publisher',
        #    name='robot_state_publisher',
        #    namespace=robot_namespace,
        #    output='screen',
        #    parameters=[{'use_sim_time': use_sim_time}],
        #    #remappings=remappings,
        #    arguments=[urdf])

        # start_robot_state_publisher_cmd = Node(
        #    condition=IfCondition(use_robot_state_pub),
        #    package='robot_state_publisher',
        #    executable='robot_state_publisher',
        #    name='robot_state_publisher',
        #    namespace=robot_namespace,
        #    output='screen',
        #    parameters=[{'use_sim_time': use_sim_time}],
        #    # remappings=remappings,
        #    arguments=[urdf])

        ld.add_action(robot_state_publisher)

    else:
        message_pump = launch_ros.actions.Node(
            package='ros2swarm',
            node_executable='message_pump',
            node_namespace=robot_namespace,
            output='screen',
            #parameters=[os.path.join(config_dir, 'hardware_protection_layer' + '.yaml')],
            arguments=[['__log_level:=', log_level]]
        )
        ld.add_action(message_pump)


    return ld
