#!/usr/bin/env python3
#    Copyright 2020 Vincent Jansen, Daniel Tidde, Marian Begemann and Tanja Kaiser
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
from collections import Counter

import numpy
import random

from communication_interfaces.msg import OpinionMessage
from ros2swarm.utils import setup_node
from ros2swarm.utils.vote_list import VoteList
from ros2swarm.voting_pattern.voting_pattern import VotingPattern
import datetime


class MajorityRulePattern(VotingPattern):
    """
    Implementation of the Majority Rule.

    Pattern to reach conclusion by setting the own option to the opinion
    of the majority of neighbors.
    The opinion could be any integer.

    pattern_node >> communicate under the topic: vote_channel
    """

    def __init__(self):
        """Initialize the majority rule pattern node."""
        super().__init__('majority_rule_pattern')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('majority_rule_initial_value', None),
                ('majority_rule_choose_start_value_at_random', None),
                ('majority_rule_min_opinion', None),
                ('majority_rule_max_opinion', None),
                ('majority_rule_timer_period', None),
            ])

        self.param_initial_value = self.get_parameter(
            "majority_rule_initial_value").get_parameter_value().integer_value
        self.param_choose_start_value_at_random = self.get_parameter(
            "majority_rule_choose_start_value_at_random").get_parameter_value().bool_value
        self.param_min_opinion = self.get_parameter(
            "majority_rule_min_opinion").get_parameter_value().integer_value
        self.param_max_opinion = self.get_parameter(
            "majority_rule_max_opinion").get_parameter_value().integer_value
        param_timer_period = self.get_parameter(
            "majority_rule_timer_period").get_parameter_value().double_value

        print(str(self.get_namespace()))
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
                                                         '/majority_broadcast',
                                                         10)

        self.opinion_publisher = self.create_publisher(OpinionMessage,
                                                       self.get_namespace() + '/opinion',
                                                       10)

        self.broadcast_subscription = self.create_subscription(
            OpinionMessage,
            '/majority_broadcast',
            self.swarm_command_controlled(self.majority_broadcast_callback),
            10)

        self.first_broadcast_flag = False

    def timer_callback(self):
        """Select a new opinion of another entity and emit the own opinion."""
        self.get_logger().debug('Robot "{}" has opinion "{}" and a list of size "{}"'
                                .format(self.get_namespace(), self.opinion,
                                        len(self.opinion_list)))

        # update opinion if at least one opinion were received and initial opinion send once
        if len(self.opinion_list) > 0 and self.first_broadcast_flag:
            self.get_logger().debug('Turtle "{}" reduce opinions "{}" at time "{}"'
                                    .format(self.id, self.opinion_list, datetime.datetime.now()))

            opinions = [e.opinion for e in self.opinion_list]
            # find max opinion
            distribution = Counter(opinions).most_common()
            # check the maximum is reached by more than one opinion
            maximum = distribution[0][1]
            maxima = []
            for e in distribution:
                if e[1] == maximum:
                    maxima.append(e[0])
                else:
                    # the input is ordered so no need for further search
                    break
            # choose randomly one of the maxima
            self.opinion = random.choice(maxima)

            self.get_logger().debug('Robot "{}" reduce opinions "{}"->"{}"'
                                    .format(self.id, opinions, self.opinion))

            self.opinion_list = []

        # emit opinion
        self.opinion_message.opinion = self.opinion
        self.broadcast_publisher.publish(self.opinion_message)
        self.opinion_publisher.publish(self.opinion_message)

        self.get_logger().debug('Robot "{}" send opinion "{}" at time "{}"'
                                .format(self.id, self.opinion, datetime.datetime.now()))
        self.first_broadcast_flag = True

    def majority_broadcast_callback(self, opinion_msg):
        """Store heard opinion message in a list to use it later."""

        self.get_logger().debug('Robot "{}" got opinion "{}" with list "{}" at time "{}"'
                                .format(self.id, opinion_msg, self.opinion_list,
                                        datetime.datetime.now()))

        self.opinion_list = VoteList.update_opinion(self.opinion_list, opinion_msg, self.id)


def main(args=None):
    """Create a node for the majority rule pattern, spins it and handles the destruction."""
    setup_node.init_and_spin(args, MajorityRulePattern)


if __name__ == '__main__':
    main()
