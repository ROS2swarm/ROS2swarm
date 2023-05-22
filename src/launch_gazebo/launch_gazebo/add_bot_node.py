# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Mofified by Marian Begemann on the base of
# https://github.com/ros-planning/navigation2/blob/master/nav2_bringup/nav2_gazebo_spawner
#   /nav2_gazebo_spawner/nav2_gazebo_spawner.py - 2020-09-25


"""
Script used to spawn a robot in a generic position
"""
import os
import rclpy
import argparse
import xacro
import time 
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from launch_ros.actions import Node
from lifecycle_msgs.srv import GetState
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import xml.etree.ElementTree as ET

def callback(msg):
     print('I heard: "%s"' % msg.data)

def main():
    """ Main for spawning robot node """
    # Get input arguments from user
    # argv = sys.argv[1:]

    # Get input arguments from user
    parser = argparse.ArgumentParser(description='Spawn Robot into Gazebo')
    parser.add_argument('-n', '--robot_name', type=str, default='robot',
                        help='Name of the robot to spawn')
    parser.add_argument('-ns', '--robot_namespace', type=str, default='robot',
                        help='ROS namespace to apply to the tf and plugins')
    parser.add_argument('-x', type=float, default=0,
                        help='the x component of the initial position [meters]')
    parser.add_argument('-y', type=float, default=0,
                        help='the y component of the initial position [meters]')
    parser.add_argument('-z', type=float, default=0,
                        help='the z component of the initial position [meters]')
    parser.add_argument('-t', '--type_of_robot', type=str, default='waffle_pi',
                        help='the type of robot')
    parser.add_argument('-ds', '--driving_swarm', type=str, default='False',
                        help='activate data recording with driving swarm framework')                 

    args, unknown = parser.parse_known_args()

    # Start node
    rclpy.init()
    node = rclpy.create_node("entity_spawner_"+args.robot_name)

    node.get_logger().debug(
        'Creating Service client to connect to `/spawn_entity`')
    client = node.create_client(SpawnEntity, "/spawn_entity")
    
    if args.driving_swarm == 'True':
        topic = f'/{args.robot_namespace}/initialpose'
        pub = node.create_publisher(PoseWithCovarianceStamped, topic, 10)


    node.get_logger().debug("Connecting to `/spawn_entity` service...")
    if not client.service_is_ready():
        client.wait_for_service()
        node.get_logger().debug("...connected!")

    # Get path to the turtlebot3
    if args.type_of_robot == "burger" or args.type_of_robot == "waffle_pi":
        sdf_file_path = os.path.join(
            get_package_share_directory("turtlebot3_gazebo"), "models",
            "turtlebot3_"+args.type_of_robot,
            "model.sdf")
        robot_description = open(sdf_file_path, 'r').read()
    elif args.type_of_robot == "thymio":
        sdf_file_path = os.path.join(
            get_package_share_directory("thymio_description"), "urdf",
            "thymio.sdf")
        robot_description = open(sdf_file_path, 'r').read()
    elif args.type_of_robot == "jackal":
        sdf_file_path = os.path.join(
            get_package_share_directory("jackal_description"), "urdf",
            "jackal.urdf.xacro")
        xml = xacro.process_file(sdf_file_path)
        robot_description = xml.toprettyxml(indent='  ')
    elif args.type_of_robot == "limo":
    	 sdf_file_path = os.path.join(
    	    get_package_share_directory('limo_description'), 'urdf', 
    	   'limo_four_diff.xacro')  
    	 xml = xacro.process_file(sdf_file_path)
    	 robot_description = xml.toprettyxml(indent='  ')

    # remapping tf topic 
    root = ET.fromstring(robot_description)
    for plugin in root.iter('plugin'):
        if 'libgazebo_ros_diff_drive.so' in plugin.attrib.values():
            break 

    ros_params = plugin.find('ros')
    ros_tf_remap = ET.SubElement(ros_params, 'remapping')
    ros_tf_remap.text = '/tf:=/' + args.robot_namespace + '/tf'

    print("sdf_file_path: ", sdf_file_path)

    node.get_logger().debug('spawning `{}` on namespace `{}` at {}, {}, {}'.format(
        args.robot_name, args.robot_namespace, args.x, args.y, args.z))
   
    request = SpawnEntity.Request()
    request.name = args.robot_name
    request.xml = ET.tostring(root, encoding='unicode')    
    request.robot_namespace = args.robot_namespace
    request.initial_pose.position.x = float(args.x)
    request.initial_pose.position.y = float(args.y)
    request.initial_pose.position.z = float(args.z)

    node.get_logger().debug("Sending service request to `/spawn_entity`")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        print('response: %r' % future.result())
    else:
        raise RuntimeError(
            'exception while calling service: %r' % future.exception())
    
    if args.driving_swarm == "True":
        # driving swarm launch elements 
        request_acml = GetState.Request()
        topic = f'/{args.robot_namespace}/amcl/get_state'
        client = node.create_client(GetState, topic)
        
        if not client.service_is_ready():
            node.get_logger().info(f'waiting for service {topic}')
            client.wait_for_service()
            node.get_logger().info(f'connected to state service')

        while True:
            future = client.call_async(request_acml)
            rclpy.spin_until_future_complete(node, future)
            if future.result() is not None:
                print('response: %r' % future.result())
                node.get_logger().info(f'{future.result()}')
                if future.result().current_state.id == 3:
                    break
            else:
                raise RuntimeError(
                    'exception while calling service: %r' % future.exception())

        time.sleep(5.0)
 
        # Send initial pose
        # geometry_msgs/msg/PoseWithCovarianceStamped
        node.get_logger().info('Sending initial pose')
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = node.get_clock().now().to_msg()
        pose.pose.pose = request.initial_pose
        pub.publish(pose)

    node.get_logger().debug("Done! Shutting down add_bot_node.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
