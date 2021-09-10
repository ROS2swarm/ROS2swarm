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

import numpy
import random

from communication_interfaces.msg import OpinionMessage
from ros2swarm.utils import setup_node
from ros2swarm.utils.vote_list import VoteList
from ros2swarm.voting_pattern.voting_pattern import VotingPattern


class VoterModelPattern(VotingPattern):
    """
    Implementation of the Voter Model.

    Pattern to reach conclusion by setting the own option to the opinion of an random nearby
    other robot. The opinion could be any integer.

    pattern_node >> communicate under the topic: vote_channel
    """

    def __init__(self):
        """Initialize the voter model pattern node."""
        super().__init__('voter_model_pattern')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('voter_model_initial_value', None),
                ('voter_model_choose_start_value_at_random', None),
                ('voter_model_min_opinion', None),
                ('voter_model_max_opinion', None),
                ('voter_model_timer_period', None),
            ])

        self.param_initial_value = self.get_parameter(
            "voter_model_initial_value").get_parameter_value().integer_value
        self.param_choose_start_value_at_random = self.get_parameter(
            "voter_model_choose_start_value_at_random").get_parameter_value().bool_value
        self.param_min_opinion = self.get_parameter(
            "voter_model_min_opinion").get_parameter_value().integer_value
        self.param_max_opinion = self.get_parameter(
            "voter_model_max_opinion").get_parameter_value().integer_value
        param_timer_period = self.get_parameter(
            "voter_model_timer_period").get_parameter_value().double_value

        # get robot id
        self.id = super().get_robot_id()

        # set initial opinion
        if self.param_choose_start_value_at_random:
            self.opinion = numpy.random.randint(self.param_min_opinion,
                                                high=self.param_max_opinion)
        else:
            self.opinion = self.param_initial_value

        # create reused OpinionMessage
        self.opinion_message = OpinionMessage()
        self.opinion_message.id = self.id
        self.opinion_message.opinion = self.opinion

        # list to store opinions
        self.opinion_list = []

        # define time period to listen to other opinions
        self.timer = self.create_timer(param_timer_period, self.swarm_command_controlled_timer(self.timer_callback))

        # OpinionMessage: {id[integer],opinion[integer]}
        self.broadcast_publisher = self.create_publisher(OpinionMessage,
                                                         '/voting_broadcast',
                                                         10)

        self.broadcast_subscription = self.create_subscription(
            OpinionMessage,
            '/voting_broadcast',
            self.swarm_command_controlled(self.voting_broadcast_callback),
            10)

        self.first_broadcast_flag = False

    def timer_callback(self):
        """Select a new opinion of another entity and emit the own opinion."""
        self.get_logger().debug('Robot "{}" has opinion "{}" and a list of size "{}"'
                                .format(self.get_namespace(), self.opinion,
                                        len(self.opinion_list)))
        # update opinion if at least one opinion were received and initial opinion send once
        if len(self.opinion_list) > 0 and self.first_broadcast_flag:
            chosen_opinion = random.choice(self.opinion_list)
            self.opinion = chosen_opinion.opinion
            self.opinion_list = []

        # emit opinion
        self.opinion_message.opinion = self.opinion
        self.broadcast_publisher.publish(self.opinion_message)
        self.first_broadcast_flag = True

    def voting_broadcast_callback(self, opinion_msg):
        """Store heard opinion message in a list to use it later."""
        self.opinion_list = VoteList.update_opinion(self.opinion_list, opinion_msg, self.id)


def main(args=None):
    """Create a node for the voter model pattern, spins it and handles the destruction."""
    setup_node.init_and_spin(args, VoterModelPattern)


if __name__ == '__main__':
    main()
