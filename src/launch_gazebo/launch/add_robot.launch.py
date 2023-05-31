#!/usr/bin/env python3
#
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
# based on:
# https://discourse.ros.org/t/spawning-a-robot-entity-using-a-node-with-gazebo-and-ros-2/9985,
# at date 2020-06-08

import os
import sys
import launch_ros.actions
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """Add turtlebot(s) to an already running gazebo simulation."""

    launch_file_dir = os.path.join(get_package_share_directory('launch_gazebo'))
    launch_pattern_dir = os.path.join(get_package_share_directory('ros2swarm'), 'launch', 'pattern')
    launch_bringup_dir = os.path.join(get_package_share_directory('ros2swarm'))
    # driving swarm 
    robot_file = os.path.join(get_package_share_directory('ros2swarm'), 'param', 'ROS2swarm_sim.yaml')
    map_file = os.path.join(launch_file_dir, 'maps', 'default.yaml') 
    tf_exchange_dir = get_package_share_directory('tf_exchange')
    
    for arg in sys.argv:
        if arg.startswith("start_index:="):  # The start index for the robots number
            start_index = int(arg.split(":=")[1])
        elif arg.startswith("gazebo_world:="):  # Name of the gazebo world
            gazebo_world = arg.split(":=")[1]   
        elif arg.startswith("number_robots:="):  # The number of robots to spawn in the world
            number_robots = int(arg.split(":=")[1])
        elif arg.startswith("pattern:="):  # The pattern executed by the robots
            pattern = arg.split(":=")[1]
        elif arg.startswith("log_level:="):  # The log level used in this execution
            log_level = arg.split(":=")[1]
        elif arg.startswith("robot:="):  # The type of robot
            robot = arg.split(":=")[1]
        elif arg.startswith("sensor_type:="):  # The type of sensor
            sensor_type = arg.split(":=")[1]
        elif arg.startswith("version:="):  # ROS version used
            ros_version = int(arg.split(":=")[1])
        elif arg.startswith("x_start:="):  # position first robot on x-axis 
            x_start = float(arg.split(":=")[1])
        elif arg.startswith("x_dist:="):  # position first robot on x-axis 
            x_dist = float(arg.split(":=")[1])
        elif arg.startswith("y_start:="):  # position of first robot on y-axis 
            y_start = float(arg.split(":=")[1])
        elif arg.startswith("y_dist:="):  # increment of positions on y-axis 
            y_dist = float(arg.split(":=")[1])
        elif arg.startswith("driving_swarm:="):  # use driving swarm framework 
            driving_swarm = arg.split(":=")[1]
            
            if driving_swarm == 'True':
                # map file for driving swarm 
                yaml_file = gazebo_world[:-6] + '.yaml' 
                map_file = os.path.join(launch_file_dir, 'maps', yaml_file)
        else:
            if arg not in ['/opt/ros/foxy/bin/ros2',
                           'launch',
                           'launch_gazebo',
                           'add_robot.launch.py']:
                print("Argument not known: '", arg, "'")

    
    print("number of robots |", number_robots)
    print("---------------------------------------")
    print("pattern          |", pattern)
    print("---------------------------------------")
    print("log level        |", log_level)
    print("---------------------------------------")
    print("robot            |", robot)
    print("---------------------------------------")
    print("sensor_type      |", sensor_type)
    print("---------------------------------------")
    print("namespace_index  |", start_index)
    print("---------------------------------------")
    print("version	     |", ros_version)
    print("---------------------------------------")
    
    # allows to use the same configuration files for each robot type but different mesh models
    robot_type = robot
    gazebo_flag = True
    if robot_type.startswith('burger'):
        robot_type = "burger"
        baseframe = 'base_link'  
    elif robot_type.startswith('waffle_pi'):
        robot_type = "waffle_pi"
        baseframe = 'base_link'  
    elif robot_type.startswith('thymio'):
        robot_type = "thymio"
        baseframe = 'base_link'  
    elif robot_type.startswith('jackal'):
        robot_type = "jackal"
        baseframe = 'base_link'  
        #gazebo_flag = False
    elif robot_type.startswith('limo'):
        robot_type = "limo"
        baseframe = 'base_link'  
        #gazebo_flag = False

    print("robot type       |", robot_type)
    print("---------------------------------------")

    ld = LaunchDescription()

    config_dir = os.path.join(get_package_share_directory('ros2swarm'), 'config', robot_type)


    if robot_type.startswith('burger') or robot_type.startswith('waffle_pi'):
        urdf_file_name = 'turtlebot3_' + robot + '.urdf'
        urdf_file = os.path.join(get_package_share_directory('turtlebot3_description'), 'urdf', urdf_file_name)
    elif robot_type.startswith('thymio'):
        urdf_file_name = 'thymio.urdf'
        urdf_file = os.path.join(get_package_share_directory('thymio_description'), 'urdf', urdf_file_name)
    elif robot_type.startwith('jackal'):
        
        config_jackal_velocity_controller = PathJoinSubstitution(
            [FindPackageShare('jackal_control'), 'config', 'control.yaml']
            )
        
        urdf_file_name = 'jackal.urdf.xacro'
        urdf_file = os.path.join(get_package_share_directory('jackal_description'), 'urdf', urdf_file_name)
    elif robot_type.startwith('limo'):
        urdf_file_name = 'limo_four_diff.xacro'
        urdf_file = os.path.join(get_package_share_directory('limo_description'), 'urdf', urdf_file_name)

    launch_pattern_dir = os.path.join(get_package_share_directory('ros2swarm'), 'launch', 'pattern')
    launch_bringup_dir = os.path.join(get_package_share_directory('ros2swarm'))


    for i in range(number_robots):
        
        if driving_swarm == 'True': 
            num = i + start_index
                
            rviz_config_file = LaunchConfiguration('rviz_config_file', default=os.path.join(get_package_share_directory('driving_swarm_bringup'), 'rviz', 'custom.rviz'))
                
            rviz = IncludeLaunchDescription(
	               PythonLaunchDescriptionSource(
	               os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'rviz_launch.py')),
		        #condition=IfCondition(LaunchConfiguration('use_rviz')),
		        launch_arguments={
		            'namespace': ['robot_', str(num)],
		            'use_namespace': 'true',
		            'use_sim_time': 'true',
		            'rviz_config': rviz_config_file
		        }.items()
		    )
            ld.add_action(rviz)
                
            # DRIVING SWARM 
            tf_exchange = IncludeLaunchDescription(
	                      PythonLaunchDescriptionSource(
		                   os.path.join(tf_exchange_dir, 'launch', 'tf_exchange.launch.py')),
		                   launch_arguments={
		                   'namespace': ['robot_', str(num)],
		                   'robot_name': ['robot_', str(num)],
		                   'base_frame': baseframe,
		                  }.items()
		          )
            ld.add_action(tf_exchange) 
        

        # add gazebo node
        gazebo_node = launch_ros.actions.Node(
            package='launch_gazebo',
            executable='add_bot_node',
            namespace=['robot_namespace_', str(num)],
            name=['gazeboRobotNode_', str(num)],
            output='screen',
            arguments=[
                '--robot_name', ['robot_', str(num)],
                '--robot_namespace', ['robot_', str(num)],
                '-x', str(x_start + i * x_dist),
                '-y', str(y_start + i * y_dist),
                '-z', '0.1',
                '--type_of_robot', robot,
                '-ds', driving_swarm, 
            ]
        )
        ld.add_action(gazebo_node)


    # find out exact path of the pattern launch file
    for i in (range(number_robots)):
        num = i + start_index
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
			       'sensor_type': sensor_type,             
                              'robot_namespace': ['robot_', str(num)],
                              'pattern': pattern_path,
                              'config_dir': config_dir,
                              'urdf_file': urdf_file,
                              'ros_version': str(ros_version),
                              'map': map_file,
                              'driving_swarm': driving_swarm,
                              'params_file': os.path.join(
                                             get_package_share_directory('ros2swarm'), 'param', 'nav2_params_' + robot_type + '_namespaced.yaml')}.items(),
        )
        ld.add_action(launch_patterns)

    return ld
