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
import rclpy
import argparse
from launch import LaunchDescription
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, Command 
from launch_ros.actions import Node
from gazebo_msgs.srv import SpawnEntity
import xml.etree.ElementTree as ET

def generate_launch_description():
    """Creates the environment with gazebo, add robots and starts their behaviour"""

    launch_file_dir = os.path.join(get_package_share_directory('launch_gazebo'))
    launch_pattern_dir = os.path.join(get_package_share_directory('ros2swarm'), 'launch', 'pattern')
    launch_bringup_dir = os.path.join(get_package_share_directory('ros2swarm'))
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    path = os.path.join(get_package_share_directory('turtlebot3_description'), 'urdf', 'turtlebot3_waffle_pi.urdf')
    urdf_file = LaunchConfiguration('urdf_file', default=path) 

        
    ld = LaunchDescription()
    
    parser = argparse.ArgumentParser(description='Spawn Robot into Gazebo')    
    for arg in sys.argv:
        if arg.startswith("gazebo_world:="):  # Name of the gazebo world
            gazebo_world = arg.split(":=")[1]        
        elif arg.startswith("robot:="):  # The type of robot
            robot = arg.split(":=")[1]
        elif arg.startswith("sensor_type:="):  # The type of sensor
            sensor_type = arg.split(":=")[1]
        elif arg.startswith("x_start:="):  # position first robot on x-axis 
            x_start = float(arg.split(":=")[1])
        elif arg.startswith("x_dist:="):  # position first robot on x-axis 
            x_dist = float(arg.split(":=")[1])
        elif arg.startswith("y_start:="):  # position of first robot on y-axis 
            y_start = float(arg.split(":=")[1])
        elif arg.startswith("y_dist:="):  # increment of positions on y-axis 
            y_dist = float(arg.split(":=")[1])
        else:
            if arg not in ['/opt/ros/foxy/bin/ros2',
                           'launch',
                           'launch_gazebo',
                           'map_environment.launch.py']:
                print("Argument not known: '", arg, "'")

    args, unknown = parser.parse_known_args()
    world_file_name = gazebo_world
    
    # allows to use the same configuration files for each robot type but different mesh models
    robot_type = robot
    gazebo_flag = True
    
    
    if gazebo_flag:
        # Add gazebo start script
        gazebo_start = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/start_gazebo.launch.py']),
            launch_arguments={'world_name': world_file_name}.items(),
        )
        ld.add_action(gazebo_start)
              
        state_publisher = launch_ros.actions.Node(
        	package='robot_state_publisher',
        	executable='robot_state_publisher',
        	output='screen',
               parameters=[{'use_sim_time': use_sim_time,                     	           
                            'robot_description': Command(['xacro ', urdf_file])}],     
              )
        ld.add_action(state_publisher)
            
    
    return ld
