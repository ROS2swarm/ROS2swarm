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
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
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
        self.model_states = ModelStates()

        self.has_item = False
        self.item_type_hold = None
        self.item_type_to_take = 0

        self.subscription_items_list = self.create_subscription(IntListMessage, '/items_list', self.item_list_callback,
                                                                10)
        self.subscription_models_states = self.create_subscription(ModelStates, '/model_states/model_states',
                                                                   self.model_states_callback, 10)
        self.cli_item_service = self.create_client(ItemService, '/item_service')
        self.command_publisher = self.create_publisher(Twist, self.get_namespace() + '/drive_command', 10)

        while not self.cli_item_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req_item_service = ItemService.Request()

        # taking items from items_master (example with timers, to implement in main)
        # self.timer_send_request = self.create_timer(10, self.send_request) #first we need to send a request to items_master

        # self.timer_check_future = self.create_timer(1, self.check_future) #then we check periodically if the request has been processed
        # sleep(random.uniform(1, 4))
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

    def model_states_callback(self, msg):
        # the robot has access to the global position of all objects with this
        self.model_states = msg
        self.pose_indexA = Pose()
        self.pose_indexB = Pose()
        self.pose_indexC = Pose()
        self.get_logger().info('The msg %s' % self.model_states.name)
        pose_indexA = self.model_states.pose[self.model_states.name.index('A_zone')]
        pose_indexB = self.model_states.pose[self.model_states.name.index('B_zone')]
        pose_indexC = self.model_states.pose[self.model_states.name.index('C_zone')]
        # pose_objA = self.model_states.pose[rob_index]
        self.get_logger().info('The Pose orientation checking %s' % pose_indexA.orientation) #pose_indexA.position.x
        indices = [i for i, x in enumerate(self.model_states.name) if "robot_name_" in x]
        for i in indices:
            robotname = self.model_states.name[i]
            robotpose = Pose()
            robotpose = self.model_states.pose[i]
            self.get_logger().info('The robots %s' % robotpose.position)  # pose_indexA.position.x

    def responseT(self, stimIntensity):
        output = stimIntensity ** self.n / (stimIntensity ** self.n + self.threshold)
        self.get_logger().info('Publishing {}:"{}"'.format(output, self.get_namespace()))
        # self.get_logger().info('The threshold is  %s and robot is %s' % output,self.get_namespace())
        return output

    def static_threshold_pattern_callback(self):

        if (self.robot_state == State.TASK_ALLOCATION):
            # fancy algorithm based on static threshold allocation to implement
            is_all_zero = not np.any(self.items_list)
            # self.get_logger().info('The items %s' % self.items_list)
            if (is_all_zero):
                self.get_logger().info('No item s')
            else:
                # calculate the probabilities for the stimulation (the normalized demand of each task)
                # this goes in to the function T
                task_demand = []
                prob = []
                choose = []
                for i in range(len(self.items_list)):
                    task_demand.append(self.items_list[i] / sum(self.items_list))
                    prob.append(self.responseT(task_demand[i]))
                    choose.append(i)

                item_taken = 0

                while (len(choose) > 0 and item_taken == 0):
                    chosen = random.choice(choose)
                    choose.remove(chosen)
                    rand = random.random()
                    self.get_logger().info('The Rand %s' % rand)
                    if (rand < prob[chosen]):
                        item_taken = 1
                        self.item_type_to_take = self.items_list.index(self.items_list[chosen])

                if (item_taken == 0):
                    self.robot_state = State.TASK_ALLOCATION
                    self.get_logger().info('I have decided to not take an item !')
                else:
                    self.robot_state = State.DO_TASK
                    self.get_logger().info('I have decided to take an item !')


        elif (self.robot_state == State.DO_TASK):
            # do the task here ! taking the item, moving, dropping the item, coming back
            # taking item
            if (self.future == None):
                self.send_request()
            if (self.check_future()):
                self.robot_state = State.CARRYING_ITEM

        elif (self.robot_state == State.CARRYING_ITEM):
            msg = Twist()
            speed_value = 0.5
            vector.x = zones[self.item_type_to_take].x - robot_position.x
            vector.y = zones[self.item_type_to_take].y - robot_position.y
            vector.magnitude = sqrt(vector.x * vector.x + vector.y * vector.y)
            vector.x = vector.x / vector.magnitude
            vector.y = vector.y / vector.magnitude
            msg.linear.x = vector.x * speed_value
            msg.linear.y = vector.y * speed_value
            theta = np.arctan((zones[self.item_type_to_take].y - robot_position.y) / (
                        zones[self.item_type_to_take].x - robot_position.x))
            msg.angular.z = theta - robot_orientation
            self.command_publisher.publish(msg)
            distance_to_zone_x = dist(zones[self.item_type_to_take].x - robot_position.x)
            distance_to_zone_y = dist(zones[self.item_type_to_take].y - robot_position.y)
            epsilon = 1  # in meters (size of the circle)
            if (distance_to_zone_x < epsilon or distance_to_zone_y < epsilon):
                self.robot_state = State.DROPPING_ITEM

        elif (self.robot_state == State.DROPPING_ITEM):
            self.get_logger().info('Dropped item %s' % self.item_type_to_take)
            self.item_type_hold = None
            self.item_type_to_take = 0

            self.robot_state = State.BACK_TO_NEST

        elif (self.robot_state == State.BACK_TO_NEST):
            msg = Twist()  # example with linear speed
            msg.linear.x = 1.0
            msg.angular.z = 0.0
            self.command_publisher.publish(msg)
            distance_to_nest_x = dist(nest.x - robot_position.x)
            distance_to_nest_y = dist(nest.y - robot_position.y)
            epsilon = 1
            if (distance_to_nest_x < epsilon or distance_to_nest_y < epsilon):
                self.robot_state = State.DROPPING_ITEM

            self.robot_state = State.TASK_ALLOCATION

    def destroy_node(self):
        """Call the super destroy method."""
        super().destroy_node()


def main(args=None):
    """Create a node for the static threshold pattern, spins it and handles the destruction."""
    setup_node.init_and_spin(args, StaticThresholdPattern)


if __name__ == '__main__':
    main()
