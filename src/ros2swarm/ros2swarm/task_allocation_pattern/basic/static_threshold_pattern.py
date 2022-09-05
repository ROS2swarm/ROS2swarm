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

        self.timer = self.create_timer(1, self.swarm_command_controlled_timer(self.static_threshold_pattern_callback))

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

    def static_threshold_pattern_callback(self):
        if (self.robot_state == State.TASK_ALLOCATION):
            # fancy algorithm based on static threshold allocation to implement
            is_all_zero = not np.any(self.items_list)
            self.get_logger().info('The items %s' % self.items_list)
            if (is_all_zero):
                self.get_logger().info('No item s')
            else:
                taskdemand1 = self.items_list[0] / (self.items_list[0] + self.items_list[1] + self.items_list[2])
                taskdemand2 = self.items_list[1] / (self.items_list[0] + self.items_list[1] + self.items_list[2])
                taskdemand3 = self.items_list[2] / (self.items_list[0] + self.items_list[1] + self.items_list[2])
                prob1 = (taskdemand1 / (taskdemand1 + 0.5))
                prob2 = (taskdemand2 / (taskdemand2 + 0.5))
                prob3 = (taskdemand3 / (taskdemand3 + 0.5))
                self.get_logger().info('The Prob1 %s' % prob1)
                self.get_logger().info('The Prob2 %s' % prob2)
                self.get_logger().info('The Prob3 %s' % prob3)
                rand = random.random()
                self.get_logger().info('The Rand 1 %s' % rand)
                if (random.random() < prob1):
                    self.item_type_to_take = self.items_list.index(self.items_list[0])
                rand = random.random()
                self.get_logger().info('The Rand 2 %s' % rand)
                if (random.random() < prob2):
                    self.item_type_to_take = self.items_list.index(self.items_list[1])
                rand = random.random()
                self.get_logger().info('The Rand 3 %s' % rand)
                if (random.random() < prob3):
                    self.item_type_to_take = self.items_list.index(self.items_list[2])
                else:
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
