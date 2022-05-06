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
import argparse
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch

def generate_launch_description():
    """
    Run this launch script on a Turtlebot3 Waffle Pi.

    It starts up the turtle and all nodes required to execute a pattern.
     Arguments:
        n --robot_number/-n: The number of the robot to create a unique namespace
        p --pattern/-p: The name of the pattern to execute

    Returns
    -------
        The launch description

    """
    launch_file_dir = os.path.join(get_package_share_directory('ros2swarm'))
    launch_pattern_dir = os.path.join(get_package_share_directory('ros2swarm'),
                                      'launch', 'pattern')

    parser = argparse.ArgumentParser(description='Environment settings')
    parser.add_argument('-n', '--robot_number', type=int, default='',
                        help='The number of robot to spawn in the world')
    parser.add_argument('-p', '--pattern', type=str, default='',
                        help='The pattern executed by the robots')
    parser.add_argument('-l', '--log_level', type=str, default='',
                        help='The log level used in this execution')
    parser.add_argument('-r', '--robot', type=str, default='',
                        help='The TurtleBot robot used')
    args, unknown = parser.parse_known_args()
    robot_number = args.robot_number
    pattern = args.pattern
    log_level = args.log_level
    robot = args.robot
    print("number of the robot: ", robot_number)
    print("pattern :", pattern)
    print("log level :", log_level)
    print("robot :", robot)

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

    # add turtle node
    turtle_namespace = ['robot_namespace_', str(robot_number)]
    turtle_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_dir, '/turtlebot3_bringup.launch.py']),
        launch_arguments={'turtle_namespace': turtle_namespace, 'pattern': pattern, 'robot': robot}.items(),
    )
    ld.add_action(turtle_node)

    # find out exact path of the patter launch file
    pattern_launch_file_name = pattern + '.launch.py'
    for root, dirs, files in os.walk(launch_pattern_dir):
        for name in files:
            if name == pattern_launch_file_name:
                pattern_path = os.path.abspath(os.path.join(root, name))

    # add patterns
    launch_patterns = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_dir, '/' + 'bringup_patterns.launch.py']),
        launch_arguments={'robot_namespace': ['robot_namespace_', str(robot_number)],
                          'pattern': pattern_path,
                          'robot': robot}.items(),
    )
    ld.add_action(launch_patterns)

    return ld
