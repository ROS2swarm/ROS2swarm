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
spawn_turtlebot.py

Script used to spawn a turtlebot in a generic position
"""
import os
import rclpy
import argparse
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity


def main():
    """ Main for spawning turtlebot node """
    # Get input arguments from user
    # argv = sys.argv[1:]

    # Get input arguments from user
    parser = argparse.ArgumentParser(description='Spawn Robot into Gazebo with navigation2')
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

    args, unknown = parser.parse_known_args()

    #TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

    # Start node
    rclpy.init()
    node = rclpy.create_node("entity_spawner")

    node.get_logger().debug(
        'Creating Service client to connect to `/spawn_entity`')
    client = node.create_client(SpawnEntity, "/spawn_entity")

    node.get_logger().debug("Connecting to `/spawn_entity` service...")
    if not client.service_is_ready():
        client.wait_for_service()
        node.get_logger().debug("...connected!")

    # Get path to the turtlebot3
    sdf_file_path = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"), "models",
        "turtlebot3_"+args.type_of_robot,
        #"turtlebot3_"+TURTLEBOT3_MODEL,
        "model.sdf")
    # TODO add here exportvariable!

    print("sdf_file_path: ", sdf_file_path)

    node.get_logger().debug('spawning `{}` on namespace `{}` at {}, {}, {}'.format(
        args.robot_name, args.robot_namespace, args.x, args.y, args.z))

    request = SpawnEntity.Request()
    request.name = args.robot_name
    request.xml = open(sdf_file_path, 'r').read()
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

    node.get_logger().debug("Done! Shutting down add_bot_node.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
