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
import sys
import launch_ros.actions

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Creates the environment with gazebo, add robots and starts their behaviour"""

    launch_file_dir = os.path.join(get_package_share_directory('launch_turtlebot_gazebo'))
    launch_pattern_dir = os.path.join(get_package_share_directory('ros2swarm'), 'launch', 'pattern')
    launch_bringup_dir = os.path.join(get_package_share_directory('ros2swarm'))

    for arg in sys.argv:
        if arg.startswith("gazebo_world:="):  # Name of the gazebo world
            gazebo_world = arg.split(":=")[1]
        elif arg.startswith("number_robots:="):  # The number of robots to spawn in the world
            number_robots = int(arg.split(":=")[1])
        elif arg.startswith("pattern:="):  # The pattern executed by the robots
            pattern = arg.split(":=")[1]
        elif arg.startswith("log_level:="):  # The log level used in this execution
            log_level = arg.split(":=")[1]
        elif arg.startswith("robot:="):  # The type of robot
            robot = arg.split(":=")[1]
        else:
            if arg not in ['/opt/ros/foxy/bin/ros2',
                           'launch',
                           'launch_turtlebot_gazebo',
                           'create_enviroment.launch.py']:
                print("Argument not known: '", arg, "'")

    world_file_name = gazebo_world

    print("---------------------------------------")
    print("world file name  |", world_file_name)
    print("---------------------------------------")
    print("number of robots |", number_robots)
    print("---------------------------------------")
    print("pattern          |", pattern)
    print("---------------------------------------")
    print("log level        |", log_level)
    print("---------------------------------------")
    print("robot            |", robot)
    print("---------------------------------------")

    # allows to use the same configuration files for each robot type but different mesh models
    robot_type = robot
    gazebo_flag = True
    if robot_type.startswith('burger'):
        robot_type = "burger"
    elif robot_type.startswith('waffle_pi'):
        robot_type = "waffle_pi"
    elif robot_type.startswith('jackal'):
        robot_type = "jackal"
        gazebo_flag = False

    print("robot type       |", robot_type)
    print("---------------------------------------")

    ld = LaunchDescription()

    # Add the log level argument to the launch description
    log = LaunchDescription([
        DeclareLaunchArgument(
            "log_level",
            default_value=log_level,
            description="Logging level",
        )
    ])
    ld.add_action(log)

    if gazebo_flag:
        # Add gazebo start script
        gazebo_start = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/start_gazebo.launch.py']),
            launch_arguments={'world_name': world_file_name}.items(),
        )
        ld.add_action(gazebo_start)

        for i in range(number_robots):
            # add gazebo node
            gazebo_node = launch_ros.actions.Node(
                package='launch_turtlebot_gazebo',
                executable='add_bot_node',
                namespace=['namespace_', str(i)],
                name=['gazeboTurtleBotNode_', str(i)],
                output='screen',
                arguments=[
                    '--robot_name', ['robot_name_', str(i)],
                    '--robot_namespace', ['robot_namespace_', str(i)],
                    '-x', '0.0',
                    '-y', [str(i), '.0'],
                    '-z', '0.1',
                    '--type_of_robot', robot
                ]
            )
            ld.add_action(gazebo_node)

    config_dir = os.path.join(get_package_share_directory('ros2swarm'), 'config', robot_type)

    urdf_file_name = 'turtlebot3_' + robot + '.urdf'
    urdf_file = os.path.join(get_package_share_directory('turtlebot3_description'), 'urdf', urdf_file_name)

    # find out exact path of the patter launch file
    for i in range(number_robots):
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
                              'robot_namespace': ['robot_namespace_', str(i)],
                              'pattern': pattern_path,
                              'config_dir': config_dir,
                              'urdf_file': urdf_file}.items(),

        )
        ld.add_action(launch_patterns)

    return ld
