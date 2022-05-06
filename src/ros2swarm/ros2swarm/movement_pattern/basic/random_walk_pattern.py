#!/usr/bin/env python3
#    Copyright 2021 Tavia Plattenteich
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

from geometry_msgs.msg import Twist
import random
from ros2swarm.movement_pattern.movement_pattern import MovementPattern
from ros2swarm.utils import setup_node


class RandomWalkPattern(MovementPattern):

    def __init__(self):
        """Initialize the random walk pattern node."""
        super().__init__('random_walk')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('random_walk_linear', None),
                ('random_walk_angular', None),
                ('random_walk_timer_period', None),
                ('random_walk_rot_interval', None),
                ('random_walk_lin_interval_min', None),
                ('random_walk_lin_interval_max', None)
            ])

        self.walk = self.create_timer(5, self.swarm_command_controlled_timer(self.random))
        self.timer = self.create_timer(
            self.get_parameter("random_walk_timer_period").get_parameter_value().double_value,
            self.swarm_command_controlled_timer(self.timer_callback))
        self.param_x = float(self.get_parameter("random_walk_linear").get_parameter_value().double_value)
        self.param_z = float(
            self.get_parameter("random_walk_angular").get_parameter_value().double_value)
        self.rot_interval = float(self.get_parameter("random_walk_rot_interval").get_parameter_value().double_value)
        self.lin_interval_min = float(self.get_parameter("random_walk_lin_interval_min")
                                      .get_parameter_value().double_value)
        self.lin_interval_max = float(self.get_parameter("random_walk_lin_interval_max")
                                      .get_parameter_value().double_value)
        self.i = 0
        self.turn = False
        self.current_msg = Twist()

    def timer_callback(self):
        """Publish the configured twist message when called."""
        self.command_publisher.publish(self.current_msg)
        self.get_logger().debug('Publishing {}:"{}"'.format(self.i, self.current_msg))
        self.i += 1

    def random(self):
        """Randomly change direction."""
        msg = Twist()
        if self.turn:
            sign = 1 if random.random() < 0.5 else -1
            msg.angular.z = random.uniform(self.param_z / 5, 4 * self.param_z / 5) * sign  # Richtung?
            msg.linear.x = 0.0
            self.walk.cancel()
            self.walk = self.create_timer(random.uniform(0, self.rot_interval), self.random)
        else:
            msg.angular.z = 0.0
            msg.linear.x = self.param_x
            self.walk.cancel()
            bu = random.uniform(self.lin_interval_min, self.lin_interval_max)
            self.walk = self.create_timer(bu, self.random)
            self.get_logger().info('Publishing {}:"{}"'.format(self.i, bu))
        self.turn = not self.turn
        self.current_msg = msg


def main(args=None):
    setup_node.init_and_spin(args, RandomWalkPattern)


if __name__ == '__main__':
    main()
