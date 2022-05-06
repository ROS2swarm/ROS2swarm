#    Copyright 2021 Marian Begemann
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
import unittest

import ament_index_python

import time

import launch
import launch.actions

import launch_ros
import launch_ros.actions
import rclpy
from geometry_msgs.msg import Twist

from ament_index_python.packages import get_package_share_directory

TEST_PROC_PATH = os.path.join(
    ament_index_python.get_package_prefix('ros2swarm'),
    'lib/ros2swarm',
    'drive_pattern'
)

proc_env = os.environ.copy()
proc_env['PYTHONUNBUFFERED'] = '1'

test_process = launch.actions.ExecuteProcess(
    cmd=[sys.executable, TEST_PROC_PATH],
    env=proc_env,
)

log_level = 'info'
robot_namespace = 'robot_namespace_1'


# @pytest.mark.rostest
# @pytest.mark.launch_test
def generate_test_description(ready_fn):
    config_dir = os.path.join(get_package_share_directory('ros2swarm'), 'config')

    ld = launch.LaunchDescription()
    ros2_pattern_node = launch_ros.actions.Node(
        package='ros2swarm',
        node_executable='drive_pattern',
        node_namespace=robot_namespace,
        output='screen',
        parameters=[os.path.join(
            config_dir, 'waffle_pi', 'movement_pattern', 'basic', 'drive_pattern.yaml')],
        arguments=[['__log_level:=', log_level]]
    )
    ld.add_action(ros2_pattern_node)

    ready = launch.actions.OpaqueFunction(function=lambda context: ready_fn())
    ld.add_action(ready)

    return ld


class TestDrivePattern(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.flag = False
        self.counter = 0

        self.testing_node = rclpy.create_node('testing_node')
        # self.testing_node = rclpy.create_node('testing_node', namespace='robot_namespace_1')

        self.sub = self.testing_node.create_subscription(
            Twist,
            'robot_namespace_1/drive_command',
            self.callback1,
            10)

        # TODO wait

    def tearDown(self):
        self.testing_node.destroy_node()

    def callback1(self, msg):
        # self.flag = True
        self.counter += 1

    def test_1(self):
        # rospy.wait_for_service('add_two_ints')
        end_time = time.time() + 10
        # TODO startup time ensure different... pattern node needs time to start
        start_time = end_time - 9
        while time.time() < start_time:
            while time.time() < end_time:
                rclpy.spin_once(self.testing_node, timeout_sec=0.1)
                # if self.flag:
                if self.counter > 10:
                    self.flag = True
                    break
        self.assertTrue(self.flag)

    def test_2(self):
        self.assertTrue(True)
    #    self.proc_output.assertWaitFor('Loop 1', timeout=10)

# @launch_testing.post_shutdown_test()
# class TestProcessOutput(unittest.TestCase):
#    def test_exit_code(self):
#        launch_testing.asserts.assertExitCodes(self.proc_info)
#
#    def test_full_output(self):
#        # Using the SequentialStdout context manager asserts that the following stdout
#        # happened in the same order that it's checked
#        with assertSequentialStdout(self.proc_output, dut_process) as cm:
#            cm.assertInStdout('Starting Up')
#            for n in range(4):
#                cm.assertInStdout('Loop {}'.format(n))
#            if os.name != 'nt':
#                # On Windows, process termination is always forced
#                # and thus the last print in good_proc never makes it.
#                cm.assertInStdout('Shutting Down')
#
#    def test_out_of_order(self):
#        # This demonstrates that we notice out-of-order IO
#        with self.assertRaisesRegexp(AssertionError, "'Loop 2' not found"):
#            with assertSequentialStdout(self.proc_output, dut_process) as cm:
#                cm.assertInStdout('Loop 1')
#                cm.assertInStdout('Loop 3')
#                cm.assertInStdout('Loop 2')  # This should raise
#
