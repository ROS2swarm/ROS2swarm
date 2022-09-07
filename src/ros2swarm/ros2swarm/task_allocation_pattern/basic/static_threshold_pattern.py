#    Copyright 2022 Antoine Sion
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
import rclpy
from ros2swarm.abstract_pattern import AbstractPattern
from communication_interfaces.msg import IntListMessage
from communication_interfaces.srv import ItemService
from ros2swarm.utils import setup_node
from ros2swarm.utils.state import State
import random
import numpy as np
from time import sleep


class StaticThresholdPattern(AbstractPattern):
    """
    Implementation of the Static Threshold approach.

    """

    def __init__(self):
        """Initialize the static threshold pattern node."""
        super().__init__('static_threshold_pattern')
        self.declare_parameters(
            namespace='',
            parameters=[
                # ('parameter_name', None),
            ])

        # self.parameter_name = self.get_parameter(
        #     "parameter_name").get_parameter_value().integer_value

        self.future = None

        self.robot_state = State.TASK_ALLOCATION
        self.items_list = []

        self.has_item = False
        self.item_type_hold = None
        self.item_type_to_take = 0

        self.subscription = self.create_subscription(IntListMessage, '/items_list', self.item_list_callback, 10)
        self.cli_item_service = self.create_client(ItemService, '/item_service')

        while not self.cli_item_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req_item_service = ItemService.Request()

        # taking items from items_master (example with timers, to implement in main)
        # self.timer_send_request = self.create_timer(10, self.send_request) #first we need to send a request to items_master

        # self.timer_check_future = self.create_timer(1, self.check_future) #then we check periodically if the request has been processed
        sleep(random.uniform(1, 4))
        self.timer = self.create_timer(1, self.swarm_command_controlled_timer(self.static_threshold_pattern_callback))

        self.threshold = random.uniform(0.5, 0.8)
        self.n = 2

    def send_request(self):
        self.req_item_service.item_index = self.item_type_to_take
        self.future = self.cli_item_service.call_async(self.req_item_service)

    def check_future(self):
        if (self.future != None and self.future.done()):
            if (self.future.result().success):
                self.has_item = True
                self.item_type_hold = self.item_type_to_take
                self.get_logger().info('Got item %s' % self.item_type_to_take)
            else:
                self.has_item = False
                self.item_type_hold = None
                self.get_logger().info('Did not get item %s' % self.item_type_to_take)
            self.future = None
            return True
        else:
            return False

    def item_list_callback(self, msg):
        # TODO implement condition for updating the list of items ONLY if the robot is in proximity of items_master
        self.items_list = msg.data

    def responseT(self, stimIntensity):
        output = stimIntensity ** self.n / (stimIntensity ** self.n + self.threshold)
        self.get_logger().info('Publishing {}:"{}"'.format(output, self.get_namespace()))
        # self.get_logger().info('The threshold is  %s and robot is %s' % output,self.get_namespace())
        return output

    def static_threshold_pattern_callback(self):

        if (self.robot_state == State.TASK_ALLOCATION):
            # fancy algorithm based on static threshold allocation to implement
            is_all_zero = not np.any(self.items_list)
            self.get_logger().info('The items %s' % self.items_list)
            if (is_all_zero):
                self.get_logger().info('No item s')
            else:
                # calculate the probabilities for the stimulation (the normalized demand of each task)
                # this goes in to the function T
                taskdemand1 = self.items_list[0] / (self.items_list[0] + self.items_list[1] + self.items_list[2])
                taskdemand2 = self.items_list[1] / (self.items_list[0] + self.items_list[1] + self.items_list[2])
                taskdemand3 = self.items_list[2] / (self.items_list[0] + self.items_list[1] + self.items_list[2])
                prob0 = self.responseT(taskdemand1)
                prob1 = self.responseT(taskdemand2)
                prob2 = self.responseT(taskdemand3)
                probabilities = [prob0, prob1, prob2]
                choose = [0, 1, 2]
                item_take = 0

                # prob1 = (taskdemand1 / (taskdemand1 + 0.5))
                # prob2 = (taskdemand2 / (taskdemand2 + 0.5))
                # prob3 = (taskdemand3 / (taskdemand3 + 0.5))
                self.get_logger().info('The Prob1 %s' % prob0)
                self.get_logger().info('The Prob2 %s' % prob1)
                self.get_logger().info('The Prob3 %s' % prob2)

                chosen1 = random.choice(choose)
                choose.remove(chosen1)
                self.get_logger().info('Chosen prob is  {}: and now list is "{}"'.format(chosen1, choose))

                chosen2 = random.choice(choose)
                choose.remove(chosen2)
                self.get_logger().info('Chosen prob2 is  {}: and now list is "{}"'.format(chosen2, choose))

                chosen3 = random.choice(choose)
                choose.remove(chosen3)
                self.get_logger().info('Chosen prob3 is  {}: and now list is "{}"'.format(chosen3, choose))

                rand = random.random()
                self.get_logger().info('The Rand 1 %s' % rand)
                if (rand < probabilities[chosen1]):
                    item_taken = 1
                    self.item_type_to_take = self.items_list.index(self.items_list[chosen1])
                rand = random.random()
                self.get_logger().info('The Rand 2 %s' % rand)
                if (rand < probabilities[chosen2]):
                    item_taken = 1
                    self.item_type_to_take = self.items_list.index(self.items_list[chosen2])
                rand = random.random()
                self.get_logger().info('The Rand 3 %s' % rand)
                if (rand < probabilities[chosen3]):
                    item_taken = 1
                    self.item_type_to_take = self.items_list.index(self.items_list[chosen3])
                if (item_take == 0):
                    self.item_type_to_take = self.items_list.index(max(self.items_list))  # just the max for now

            self.get_logger().info('The items to move %s' % self.items_list)
            self.robot_state = State.DO_TASK

        elif (self.robot_state == State.DO_TASK):
            # do the task here ! taking the item, moving, dropping the item, coming back
            # taking item
            """
            msg = Twist()
            inc_x = self.goalx-self.x
            inc_y = self.goaly-self.y
            angle_to_goal = atan2(inc_y, inc_x)
            if abs(angle_to_goal - self.theta) > 0.1:
            	msg.linear.x = 0.0
            	msg.angular.z = 0.3
            else:
            	msg.linear.x = 0.5
            	msg.angular.z = 0.0
            self.command_publisher.publish(msg)
            self.get_logger().info('Publishing {}'.format( msg))
            """
            if (self.future == None):
                self.send_request()
            if (self.check_future()):
                self.robot_state = State.TASK_ALLOCATION  # TODO replace this with moving etc

    def destroy_node(self):
        """Call the super destroy method."""
        super().destroy_node()


def main(args=None):
    """Create a node for the static threshold pattern, spins it and handles the destruction."""
    setup_node.init_and_spin(args, StaticThresholdPattern)


if __name__ == '__main__':
    main()