#!/usr/bin/env python3
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

from geometry_msgs.msg import Twist
from ros2swarm.utils import setup_node
from ros2swarm.movement_pattern.movement_pattern import MovementPattern
from communication_interfaces.msg import OpinionMessage, DoubleMessage


class DiscussedDispersion(MovementPattern):
    """
    Pattern to perform a dispersion of the swarm based a common first to discuss distance value.
    Keep distance at 1,2, or 3 meters, depending on common opinion 1 to 3.
    """

    def __init__(self):
        """Initialize the discussed dispersion pattern node."""
        super().__init__('discussed_dispersion_pattern')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('max_translational_velocity', None),
                ('max_rotational_velocity', None),
                ('discussed_dispersion_timer_period', None),
                ('discussed_dispersion_discussion_base_distance', None),
                ('discussed_dispersion_discussion_time', None),
                ('discussed_dispersion_discussion_opinion_multiply', None),
            ])
        self.counter = 0
        self.dispersion_latest = Twist()
        self.opinion_latest = OpinionMessage()
        self.message = DoubleMessage()

        # messages from subpattern subscriptions
        self.dispersion_subpattern = self.create_subscription(
            Twist,
            self.get_namespace() + '/drive_command_dispersion_pattern',
            self.command_callback_dispersion,
            10)
        self.majority_rule_subpattern = self.create_subscription(
            OpinionMessage,
            self.get_namespace() + '/opinion',
            self.command_callback_majority_rule,
            10)

        self.dispersion_distance_publisher = self.create_publisher(DoubleMessage,
                                                       self.get_namespace() + '/dispersion_distance', 10)

        self.param_max_translational_velocity = self.get_parameter(
            "max_translational_velocity").get_parameter_value().double_value
        self.param_max_rotational_velocity = self.get_parameter(
            "max_rotational_velocity").get_parameter_value().double_value
        timer_period = self.get_parameter(
            "discussed_dispersion_timer_period").get_parameter_value().double_value
        discussion_time = self.get_parameter(
            "discussed_dispersion_discussion_time").get_parameter_value().double_value
        self.base_distance = self.get_parameter(
            "discussed_dispersion_discussion_base_distance").get_parameter_value().double_value
        self.opinion_multiply = self.get_parameter(
            "discussed_dispersion_discussion_opinion_multiply").get_parameter_value().double_value

        self.timer = self.create_timer(timer_period,
                                       self.swarm_command_controlled_timer(self.discussed_dispersion_callback))
        self.switch = discussion_time / timer_period

    def command_callback_dispersion(self, incoming_msg: Twist):
        """Assign the message to variable"""
        self.dispersion_latest = incoming_msg

    def command_callback_majority_rule(self, incoming_msg: OpinionMessage):
        """Assign the message to variable"""
        self.opinion_latest = incoming_msg

    def discussed_dispersion_callback(self):
        """ Controls the discussed dispersion pattern. """

        if self.counter <= self.switch:
            self.get_logger().debug('Opinion is: "{}"'.format(self.opinion_latest.opinion))
            self.counter += 1
        else:
            distance = self.base_distance + (self.opinion_latest.opinion * self.opinion_multiply)
            self.message.data = distance if distance <= 3.5 else 3.5 # TODO or zero?
            self.dispersion_distance_publisher.publish(self.message)
            self.command_publisher.publish(self.dispersion_latest)


def main(args=None):
    """Create a node for the discussed dispersion pattern, spins it and handles the destruction."""
    setup_node.init_and_spin(args, DiscussedDispersion)


if __name__ == '__main__':
    main()
