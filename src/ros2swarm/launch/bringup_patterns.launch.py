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
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, PythonExpression
from launch.conditions import IfCondition, LaunchConfigurationEquals, LaunchConfigurationNotEquals

def generate_launch_description():
    """Bring up all needed hardware protection patterns and the behaviour pattern.""" 
    
    config_dir = LaunchConfiguration('config_dir', default='config_dir_default')
    urdf_file = LaunchConfiguration('urdf_file', default='urdf_file_default')
    robot_namespace = LaunchConfiguration('robot_namespace', default='robot_namespace_default')
    pattern = LaunchConfiguration('pattern', default='pattern_default')
    log_level = LaunchConfiguration('log_level', default='debug')
    robot = LaunchConfiguration('robot', default='robot_default')
    robot_type = LaunchConfiguration('robot_type', default='robot_type_default')
    sensor_type = LaunchConfiguration('sensor_type', default='sensor_type_default')
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    driving_swarm = LaunchConfiguration('driving_swarm', default='False')
    local_map = LaunchConfiguration('map')
    
    # driving swarm 
    nav2_dir = get_package_share_directory('nav2_bringup')
    slam = LaunchConfiguration('slam', default='False')
    autostart = 'True'

    params_file = LaunchConfiguration('params_file', default=os.path.join(get_package_share_directory('ros2swarm'), 'params', 'nav2_params_waffle_pi_namespaced.yaml'))

    ld = LaunchDescription()

    # Add sensor layer
    ros2_sensor_layer_node = launch_ros.actions.Node(
        package='ros2swarm',
        executable=[sensor_type,'_layer'],
        namespace=robot_namespace,
        output='screen',
        parameters=[PathJoinSubstitution([config_dir, 'sensor_specification' + '.yaml'])],
        arguments=['--ros-args', '--log-level', log_level]
    )
    ld.add_action(ros2_sensor_layer_node)

    # Add Hardware protection layer
    ros2_hardware_protection_layer_node = launch_ros.actions.Node(
        package='ros2swarm',
        executable='hardware_protection_layer',
        namespace=robot_namespace,
        output='screen',
        parameters=[PathJoinSubstitution([config_dir, 'hardware_protection_layer' + '.yaml'])],
        arguments=['--ros-args', '--log-level', log_level]
    )
    ld.add_action(ros2_hardware_protection_layer_node)

    # add pattern
    launch_pattern = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([pattern]),
        launch_arguments={'robot': robot_type,
                          'robot_namespace': [robot_namespace],
                          'config_dir': config_dir,
                          }.items(),

    )
    ld.add_action(launch_pattern)

    robot_state_publisher = GroupAction([
        launch_ros.actions.PushRosNamespace(
            namespace=robot_namespace),
                   
        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
       	       # namespace=robot_namespace,
        	        output='screen',
        	        condition=LaunchConfigurationNotEquals('robot_type', 'thymio'),
        	        parameters=[{'use_sim_time': use_sim_time,
                    	           'robot_description': Command(['xacro ', urdf_file])}],     	     
                       remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
  	            ),
  	
  	# add state publisher for thymio 
  	launch_ros.actions.Node(
	    package='robot_state_publisher',
	    executable='robot_state_publisher',
	    namespace=robot_namespace,
	    output='screen',
	    condition=LaunchConfigurationEquals('robot_type', 'thymio'),
	    parameters=[{'use_sim_time': use_sim_time,
	 	             }],
	    arguments=[urdf_file, '--ros-args', '--log-level', 'warn'],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
        ),
  	            

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_dir, 'launch',
                                                  'localization_launch.py')),
            condition=LaunchConfigurationEquals('driving_swarm', 'True'),
            launch_arguments={'namespace': robot_namespace,
                              'map': local_map,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file,
                              'use_lifecycle_mgr': 'false'}.items())
         ])
            
    
    ld.add_action(robot_state_publisher)

    return ld
