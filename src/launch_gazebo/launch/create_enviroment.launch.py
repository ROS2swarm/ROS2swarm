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
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from datetime import datetime 

def generate_launch_description():
    """Creates the environment with gazebo, add robots and starts their behaviour"""

    launch_file_dir = os.path.join(get_package_share_directory('launch_gazebo'))
    launch_pattern_dir = os.path.join(get_package_share_directory('ros2swarm'), 'launch', 'pattern')
    launch_bringup_dir = os.path.join(get_package_share_directory('ros2swarm'))
    map_file = 'default.yaml' 
    robot_file = os.path.join(get_package_share_directory('ros2swarm'), 'param', 'ROS2swarm_sim.yaml')
    run_timeout = 0.0 
    init_timeout = 0.0
    logging_output_dir = None 
    
    ld = LaunchDescription()
    
    for arg in sys.argv:
        if arg.startswith("gazebo_world:="):  # Name of the gazebo world
            gazebo_world = arg.split(":=")[1]        
        elif arg.startswith("number_robots:="):  # The number of robots to spawn in the world
            number_robots = int(arg.split(":=")[1])
        elif arg.startswith("total_robots:="):  # The number of robots to spawn in the world
            total_robots = int(arg.split(":=")[1])
        elif arg.startswith("pattern:="):  # The pattern executed by the robots
            pattern = arg.split(":=")[1]
        elif arg.startswith("log_level:="):  # The log level used in this execution
            log_level = arg.split(":=")[1]
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
        elif arg.startswith("logging_output_dir:="):  # increment of positions on y-axis 
            logging_output_dir = arg.split(":=")[1]
        elif arg.startswith("gui:="):  # GUI or no GUI 
            if arg.split(":=")[1]: 
                simulator = 'gazebo'
            else: 
                simulator = 'gzserver'
        elif arg.startswith("run_timeout:="):  # duration of experiment
            run_timeout = float(arg.split(":=")[1])
        elif arg.startswith("init_timeout:="):  # timeout before experiment start
            init_timeout = float(arg.split(":=")[1])
        elif arg.startswith("driving_swarm:="):  # use driving swarm framework 
            driving_swarm = arg.split(":=")[1]
            
            if driving_swarm == 'True':
                yaml_file = gazebo_world[:-6] + '.yaml' 
                map_file = os.path.join(launch_file_dir, 'maps', yaml_file)
        elif arg.startswith("logging:="):  # log data in rosbag 
            logging = arg.split(":=")[1]
        else:
            if arg not in ['/opt/ros/foxy/bin/ros2',
                           'launch',
                           'launch_gazebo',
                           'create_enviroment.launch.py']:
                print("Argument not known: '", arg, "'")

    world_file_name = gazebo_world
    
    if logging_output_dir == None:             
        logging_output_dir = str(gazebo_world[:-6]) + '_' + str(total_robots) + '_' + str(pattern[:-8]) + '_' + str(x_start) + '_' + str(y_start) + '_' + str(x_dist) + '_' + str(y_dist) + '_'  + datetime.utcnow().strftime('%Y-%m-%d_%H:%M:%S') 


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
    print("sensor_type      |", sensor_type)
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
        baseframe = 'base_link' # ToDo 
    elif robot_type.startswith('jackal'):
        robot_type = "jackal"
        #gazebo_flag = True
        baseframe = 'base_link' # ToDo 
    elif robot_type.startswith('limo'):
        robot_type = "limo"
        baseframe = 'base_link'  

    print("robot type       |", robot_type)
    print("---------------------------------------")



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
            launch_arguments={'world_name': world_file_name,
                              'simulator': simulator}.items(),
        )
        ld.add_action(gazebo_start)
        
        state_node = Node(package="launch_gazebo",
                          executable="ground_truth_publisher",
                          output="screen",
                          parameters=[{
                          'use_sim_time': True,
                          }])
        ld.add_action(state_node)
        
        
        # driving swarm 
        if driving_swarm == 'True' or logging=='True': 
        
            command_node = Node(package="experiment_supervisor",
                                executable="command_node",
                                output="screen",
                                parameters=[{
                                    'use_sim_time': True,
                                    'run_timeout': run_timeout,
                                    'init_timeout': init_timeout,
                                    'robots': ['robot_' + str(i) for i in range(number_robots)],
                               }])

            exit_event_handler = RegisterEventHandler(event_handler=OnProcessExit(
                                                      target_action=command_node,
                                                      on_exit=EmitEvent(event=Shutdown(reason="command node exited"))
                                                      )
                                                     )
            ld.add_action(command_node)
            ld.add_action(exit_event_handler)

        for i in range(number_robots):
        
            if driving_swarm == 'True': 
                rviz_config_file = LaunchConfiguration('rviz_config_file', default=os.path.join(get_package_share_directory('driving_swarm_bringup'), 'rviz', 'custom.rviz'))
                
                rviz = IncludeLaunchDescription(
	                   PythonLaunchDescriptionSource(
		            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'rviz_launch.py')),
		        #condition=IfCondition(LaunchConfiguration('use_rviz')),
		        launch_arguments={
		            'namespace': ['robot_', str(i)],
		            'use_namespace': 'true',
		            'use_sim_time': 'true',
		            'rviz_config': rviz_config_file
		        }.items()
		    )
                ld.add_action(rviz)
                
                # DRIVING SWARM 
                tf_exchange = IncludeLaunchDescription(
		        PythonLaunchDescriptionSource(
		            os.path.join(get_package_share_directory('tf_exchange'), 'launch', 'tf_exchange.launch.py')),
		            launch_arguments={
		            'namespace': ['robot_', str(i)],
		            'robot_name': ['robot_', str(i)],
		            'base_frame': baseframe,
		        }.items()
		    )
                ld.add_action(tf_exchange) 
            
            
            # add gazebo node
            gazebo_node = Node(
                package='launch_gazebo',
                executable='add_bot_node',
                namespace=['namespace_', str(i)],
                name=['gazeboRobotNode_', str(i)],
                output='screen',
                arguments=[
                    '--robot_name', ['robot_', str(i)],
                    '--robot_namespace', ['robot_', str(i)],
                    '-x', str(x_start + i * x_dist),
                    '-y', str(y_start + i * y_dist),
                    '-z', '0.1',
                    '--type_of_robot', robot,
                    '-ds', driving_swarm, 
                ]
            )
            ld.add_action(gazebo_node)

    config_dir = os.path.join(get_package_share_directory('ros2swarm'), 'config', robot_type)

    if robot_type.startswith('burger') or robot_type.startswith('waffle_pi'):
        urdf_file_name = 'turtlebot3_' + robot + '.urdf'
        urdf_file = os.path.join(get_package_share_directory('turtlebot3_description'), 'urdf', urdf_file_name)
    elif robot_type.startswith('thymio'):
        urdf_file_name = 'thymio.urdf'
        urdf_file = os.path.join(get_package_share_directory('thymio_description'), 'urdf', urdf_file_name)
    elif robot_type.startswith('jackal'):
        urdf_file_name = 'jackal.urdf.xacro'
        urdf_file = os.path.join(get_package_share_directory('jackal_description'), 'urdf', urdf_file_name)
    elif robot_type.startswith('limo'):
        urdf_file_name = 'limo_four_diff.xacro'
        urdf_file = os.path.join(get_package_share_directory('limo_description'), 'urdf', urdf_file_name)
        
    # find out exact path of the pattern launch file
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
			                        'sensor_type': sensor_type,
                              'robot_namespace': ['robot_', str(i)],
                              'sensor_type': sensor_type, 
                              'pattern': pattern_path,
                              'config_dir': config_dir,
                              'urdf_file': urdf_file, 
                              'map': map_file,
                              'driving_swarm': driving_swarm,
                              'params_file': os.path.join(
            get_package_share_directory('ros2swarm'), 'param', 'nav2_params_' + robot_type + '_namespaced.yaml')
        }.items()
        )
        ld.add_action(launch_patterns)

    
    if logging=='True':

    
        exp_measurement_dir = get_package_share_directory('experiment_measurement')
        
        rosbag_recording = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(exp_measurement_dir,
                             'launch', 'rosbag_recording.launch.py')),
                 launch_arguments={'n_robots': str(total_robots), 
                              'robots_file': robot_file,
                              'use_rosbag': 'True',
                              'rosbag_topics_file': os.path.join(get_package_share_directory('ros2swarm'), 'param', 'rosbag_topics.yaml'), 
                              'qos_override_file': os.path.join(get_package_share_directory('experiment_measurement'), 'params', 'qos_override.yaml'),
                              'rosbag_output_dir': logging_output_dir, 
                             }.items()
        )
    
        ld.add_action(rosbag_recording)
    

    return ld
