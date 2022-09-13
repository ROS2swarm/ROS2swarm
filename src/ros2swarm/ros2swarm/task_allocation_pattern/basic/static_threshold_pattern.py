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
from geometry_msgs.msg import Twist,Pose
from ros2swarm.utils import setup_node
from ros2swarm.utils import quaternion_transform
from ros2swarm.utils.state import State

import random
import numpy as np
from time import sleep
import time
import csv
from rclpy.node import Node

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
        self.robot_state = State.BACK_TO_NEST
        self.items_list = []
        self.model_states = ModelStates()
        self.zones = []
        self.pose_items_master_zone = Pose()
        self.robot_pose = Pose()

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

        self.timer = self.create_timer(1, self.swarm_command_controlled_timer(self.static_threshold_pattern_callback))

        self.threshold = []
        for i in range(3):
            self.threshold.append(random.uniform(0.0, 1.0))
        self.n = 2
        self.filepath_log = "log_items_removed.csv"
        items_string_list = ['robot_name','item_picked', 'start_time','end_time','time taken']
        with open(self.filepath_log, 'w', encoding='UTF8') as f:
            writer = csv.writer(f)
            # write the header
            writer.writerow(items_string_list)
        self.moved = [0,0,0,0,0]
        self.t0 = 0
        self.t1 = 1
        self.time=0

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
        self.items_list = msg.data

    def model_states_callback(self, msg):
        # the robot has access to the global position of all objects with this
        self.model_states = msg
        # self.get_logger().info('The msg %s' % self.model_states.name)
        zone_list = ['A_zone', 'B_zone', 'C_zone']
        temp_zones = []
        for zone_id in range(len(zone_list)):
            temp_zones.append(self.model_states.pose[self.model_states.name.index(zone_list[zone_id])])
        self.zones = temp_zones
        self.pose_items_master_zone = self.model_states.pose[self.model_states.name.index('items_master_zone')]
        # pose_objA = self.model_states.pose[rob_index]
        # self.get_logger().info('The Pose orientation checking %s' % self.pose_zoneA.orientation) #pose_indexA.position.x
        indices = [i for i, x in enumerate(self.model_states.name) if "robot_name_" in x]
        for i in indices:
            if(self.get_namespace()[-1] == self.model_states.name[i][-1]):
                self.robot_pose = self.model_states.pose[i]

    def responseT(self, stimIntensity, item_type):
        output = stimIntensity ** self.n / (stimIntensity ** self.n + self.threshold[item_type])
        self.get_logger().info('Publishing {}:"{}"'.format(output, self.get_namespace()))
        # self.get_logger().info('The threshold is  %s and robot is %s' % output,self.get_namespace())
        return output

    def static_threshold_pattern_callback(self):
        if (self.robot_state == State.TASK_ALLOCATION):
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
                    prob.append(self.responseT(task_demand[i], i))
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
                        self.moved[0]=self.get_namespace()
                        self.moved[1]= self.item_type_to_take

                if (item_taken == 0):
                    self.robot_state = State.TASK_ALLOCATION
                    self.command_publisher.publish(Twist())
                    self.get_logger().info('I have decided to not take an item !')
                else:
                    self.robot_state = State.DO_TASK
                    self.get_logger().info('I have decided to take an item !')


        elif (self.robot_state == State.DO_TASK):
            # do the task here ! taking the item, moving, dropping the item, coming back
            time_string = str(self.get_clock().now().to_msg()).split('sec=')[1]
            time_string = time_string.split(',')[0]
            self.t0 = float(time_string)
            self.get_logger().info('The timer is  %s' % time_string)
            if (self.future == None):
                self.send_request()
            if (self.check_future()):
                self.robot_state = State.CARRYING_ITEM

        elif (self.robot_state == State.CARRYING_ITEM):
            self.set_speeds(self.zones[self.item_type_to_take])
            distance_to_zone_x = abs(self.zones[self.item_type_to_take].position.x - self.robot_pose.position.x)
            distance_to_zone_y = abs(self.zones[self.item_type_to_take].position.y - self.robot_pose.position.y)
            epsilon = 2.5  # in meters (size of the circle)
            if(distance_to_zone_x < epsilon and distance_to_zone_y < epsilon):
                self.command_publisher.publish(Twist())
                self.robot_state = State.DROPPING_ITEM

        elif (self.robot_state == State.DROPPING_ITEM):
            self.get_logger().info('Dropped item %s' % self.item_type_to_take)
            self.item_type_hold = None
            self.item_type_to_take = 0
            self.robot_state = State.BACK_TO_NEST
            time_string = str(self.get_clock().now().to_msg()).split('sec=')[1]
            time_string = time_string.split(',')[0]
            self.t1 = float(time_string)
            total = self.t1-self.t0
            with open(self.filepath_log, 'a', encoding='UTF8') as ff:
                writer = csv.writer(ff)
                self.moved[2]=self.t0
                self.moved[3]=self.t1
                self.moved[4]=total
                # write the data
                self.get_logger().info('The item ot take {}'.format(self.moved))
                writer.writerow(self.moved)

        elif (self.robot_state == State.BACK_TO_NEST):
            self.set_speeds(self.pose_items_master_zone)
            distance_to_zone_x = abs(self.pose_items_master_zone.position.x - self.robot_pose.position.x)
            distance_to_zone_y = abs(self.pose_items_master_zone.position.y - self.robot_pose.position.y)
            epsilon = 2.5  # in meters (size of the circle)
            if(distance_to_zone_x < epsilon and distance_to_zone_y < epsilon):
                self.get_logger().info('At the start zone !')
                self.command_publisher.publish(Twist())
                self.robot_state = State.TASK_ALLOCATION

    def set_speeds(self, zone):
        msg = Twist()
        vector_x = zone.position.x - self.robot_pose.position.x
        vector_y = zone.position.y - self.robot_pose.position.y
        vector_magnitude = np.sqrt(vector_x * vector_x + vector_y * vector_y)
        vector_x = vector_x / vector_magnitude
        vector_y = vector_y / vector_magnitude
        # self.get_logger().info('X speed %s' % str(vector_x))
        # self.get_logger().info('Y speed %s' % str(vector_y))
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        if(vector_x >= 0):
            theta = np.arctan((zone.position.y - self.robot_pose.position.y) / (zone.position.x - self.robot_pose.position.x))
        if(vector_x < 0 and vector_y >0):
            theta = 3.1415 + np.arctan((zone.position.y - self.robot_pose.position.y) / (zone.position.x - self.robot_pose.position.x))
        if(vector_x < 0 and vector_y <0):
            theta = -3.1415 + np.arctan((zone.position.y - self.robot_pose.position.y) / (zone.position.x - self.robot_pose.position.x))
        t_x,t_y,t_z = quaternion_transform.euler_from_quaternion(self.robot_pose.orientation)

        delta = theta - t_z
        if(delta > 3.1415):
            delta = delta - 2*3.1415
        if(delta < -3.1415):
            delta = delta + 2*3.1415
        if(delta < 0):
            msg.angular.z = -0.4
        if(delta > 0):
            msg.angular.z = 0.4
        if(delta < 3.1415/4 and delta > -3.1415/4):
            msg.linear.x = 1.0
            msg.angular.z = msg.angular.z/2
        # self.get_logger().info('t_z %s' % str(t_z))
        # self.get_logger().info('theta %s' % str(theta))
        # self.get_logger().info('delta %s' % str(delta))
        # self.get_logger().info('Z angular speed %s' % str((theta - t_z)))
        self.command_publisher.publish(msg)

    def destroy_node(self):
        """Call the super destroy method."""
        super().destroy_node()


def main(args=None):
    """Create a node for the static threshold pattern, spins it and handles the destruction."""
    setup_node.init_and_spin(args, StaticThresholdPattern)


if __name__ == '__main__':
    main()