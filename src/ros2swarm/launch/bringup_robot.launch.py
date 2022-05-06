# !/usr/bin/env python3
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
import launch

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """
    Run this launch script on a Turtlebot3 Waffle Pi or Burger.

    It starts up the turtle and all nodes required to execute a pattern.
     Arguments:
        robot_number -  The number of the robot to create a unique namespace
        pattern - The name of the pattern to execute

    Returns
    -------
        The launch description

    """

    launch_file_dir = os.path.join(get_package_share_directory('ros2swarm'))
    launch_pattern_dir = os.path.join(get_package_share_directory('ros2swarm'), 'launch', 'pattern')

    for arg in sys.argv:
        if arg.startswith("robot_number:="):  # The number of the robot
            robot_number = int(arg.split(":=")[1])
        elif arg.startswith("pattern:="):  # The pattern executed by the robots
            pattern = arg.split(":=")[1]
        elif arg.startswith("log_level:="):  # The log level used in this execution
            log_level = arg.split(":=")[1]
        elif arg.startswith("robot:="):  # The type of robot
            robot = arg.split(":=")[1]
        else:
            if arg not in ['/opt/ros/foxy/bin/ros2',
                           'launch',
                           'ros2swarm',
                           'launch_turtlebot_gazebo',
                           'bringup_robot.launch.py']:
                print("Argument not known: '", arg, "'")

    print("---------------------------------------")
    print("robot number     |", robot_number)
    print("---------------------------------------")
    print("pattern          |", pattern)
    print("---------------------------------------")
    print("log level        |", log_level)
    print("---------------------------------------")
    print("robot            |", robot)
    print("---------------------------------------")

    robot_type = robot
    robot_node = True
    if robot_type.startswith('burger'):
        robot_type = "burger"
    elif robot_type.startswith('waffle_pi'):
        robot_type = "waffle_pi"
    elif robot_type.startswith('jackal'):
        robot_type = "jackal"
        robot_node = False

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

    if robot_node:
        # add turtle node
        turtle_namespace = ['robot_namespace_', str(robot_number)]
        turtle_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/turtlebot3_bringup.launch.py']),
            launch_arguments={'turtle_namespace': turtle_namespace,
                              'pattern': pattern,
                              'robot': robot}.items(),
        )
        ld.add_action(turtle_node)

    urdf_file_name = 'turtlebot3_' + robot + '.urdf'
    urdf_file = os.path.join(get_package_share_directory('turtlebot3_description'), 'urdf', urdf_file_name)

    config_dir = os.path.join(get_package_share_directory('ros2swarm'), 'config', robot_type)

    # find out exact path of the patter launch file
    pattern_launch_file_name = pattern + '.launch.py'
    for root, dirs, files in os.walk(launch_pattern_dir):
        for name in files:
            if name == pattern_launch_file_name:
                pattern_path = os.path.abspath(os.path.join(root, name))

    # add patterns
    launch_patterns = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_dir, '/' + 'bringup_patterns.launch.py']),
        launch_arguments={'robot': robot,
                          'robot_type': robot_type,
                          'robot_namespace': ['robot_namespace_', str(robot_number)],
                          'pattern': pattern_path,
                          'config_dir': config_dir,
                          'urdf_file': urdf_file
                          }.items(),
    )
    ld.add_action(launch_patterns)

    return ld
