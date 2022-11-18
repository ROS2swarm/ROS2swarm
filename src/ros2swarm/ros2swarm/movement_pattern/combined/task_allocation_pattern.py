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

import rclpy
from ros2swarm.movement_pattern.movement_pattern import MovementPattern
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

class TaskAllocationPattern(MovementPattern):
    """
    Implementation of the Static Threshold approach.
    Every robot is initialised with different thresholds for each type of item.
    These determine the probability to take an item and do the associated task.
    """

    def __init__(self):
        """Initialize the task allocation pattern node."""
        super().__init__('task_allocation_pattern')
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

        #to see the number of each item at items_master
        self.subscription_items_list = self.create_subscription(IntListMessage, '/items_list', self.item_list_callback,
                                                                10)
        #to ease the implementation of movement, absolute positioning is used
        self.subscription_models_states = self.create_subscription(ModelStates, '/model_states/model_states',
                                                                   self.model_states_callback, 10)
        #to request items
        self.cli_item_service = self.create_client(ItemService, '/item_service')

        #to input the target pose to the move_to_target_pattern
        self.target_pose_publisher = self.create_publisher(Pose, self.get_namespace() + '/target_pose', 10)

        #to input the robot pose to the move_to_target_pattern
        self.robot_pose_publisher = self.create_publisher(Pose, self.get_namespace() + '/robot_pose', 10)

        while not self.cli_item_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req_item_service = ItemService.Request()

        #main loop
        self.timer = self.create_timer(1, self.swarm_command_controlled_timer(self.task_allocation_pattern_callback))

        self.threshold = []
        for i in range(3): #set threshold for each of three object
            self.threshold.append(random.uniform(0.0, 1.0))
        self.n = 2  #set the responsiveness to stimulus
        self.filepath_log = "log_items_removed.csv"  #file to save object type removed and time taken to drop and come back
        items_string_list = ['robot_name','item_picked','start_time','drop_time','end_time','time_taken'] #header in csv log file
        with open(self.filepath_log, 'w', encoding='UTF8') as f:
            writer = csv.writer(f)
            # write the header
            writer.writerow(items_string_list)
        self.moved = ['','','','','',''] #list to save data to write to csv file as a row
        self.t0 = 0
        self.t1 = 0
        self.t2 = 0
        self.moved[0]= str(self.get_namespace())

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

    def responseT(self, stimIntensity, item_type): #the response function takes stimulus(task demand) and caluclates response value
        output = stimIntensity ** self.n / (stimIntensity ** self.n + self.threshold[item_type])
        self.get_logger().info('Publishing {}:"{}"'.format(output, self.get_namespace()))
        # self.get_logger().info('The threshold is  %s and robot is %s' % output,self.get_namespace())
        return output

    def task_allocation_pattern_callback(self):
        if (self.robot_state == State.TASK_ALLOCATION):
            is_all_zero = not np.any(self.items_list)
            # self.get_logger().info('The items %s' % self.items_list)
            if (is_all_zero): # if items to pick = 0
                self.get_logger().info('No item s')
            else:
                # calculate the probabilities for the stimulation (the normalized demand of each task)
                # this goes in to the function T
                task_demand = []
                prob = []
                choose = []
                for i in range(len(self.items_list)):
                    task_demand.append(self.items_list[i] / sum(self.items_list)) # the stimulus
                    prob.append(self.responseT(task_demand[i], i)) #get values from response function
                    choose.append(i) #append to list to randomnly choose which one to probabilistically check first

                item_taken = 0

                while (len(choose) > 0 and item_taken == 0): #choose one probability randomnly and remove items probabilistically
                    chosen = random.choice(choose)
                    choose.remove(chosen)
                    rand = random.random()
                    self.get_logger().info('The Rand %s' % rand)
                    if (rand < prob[chosen]): # compare the chosen item probability with a random number 0-1 to remove an item of that type
                        item_taken = 1
                        self.item_type_to_take = self.items_list.index(self.items_list[chosen])
                        self.moved[1]= str(self.item_type_to_take)
                        self.get_logger().info('The item in moved  take {}'.format(self.moved))

                if (item_taken == 0):
                    self.robot_state = State.TASK_ALLOCATION
                    self.target_pose_publisher.publish(self.robot_pose)
                    self.robot_pose_publisher.publish(self.robot_pose)
                    self.get_logger().info('I have decided to not take an item !')
                else:
                    self.robot_state = State.DO_TASK
                    self.get_logger().info('I have decided to take an item !')


        elif (self.robot_state == State.DO_TASK):
            # do the task here ! taking the item, moving, dropping the item, coming back
            time_string = str(self.get_clock().now().to_msg()).split('sec=')[1]  #get current time
            time_string = time_string.split(',')[0]
            self.t0 = int(time_string) #save time of when task to drop an item starts
            self.moved[2]=str(self.t0)
            self.get_logger().info('The start timer is  %s' % self.t0)
            if (self.future == None):
                self.send_request()
            if (self.check_future()):
                self.robot_state = State.CARRYING_ITEM

        elif (self.robot_state == State.CARRYING_ITEM):
            self.target_pose_publisher.publish(self.zones[self.item_type_to_take])
            self.robot_pose_publisher.publish(self.robot_pose)
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
            self.t1 = int(time_string)
            self.moved[3]=str(self.t1)#log time to drop the item in zones to save in csv file
            self.get_logger().info('The drop timer is  %s' % self.t1)

        elif (self.robot_state == State.BACK_TO_NEST):
            self.target_pose_publisher.publish(self.pose_items_master_zone)
            self.robot_pose_publisher.publish(self.robot_pose)
            distance_to_zone_x = abs(self.pose_items_master_zone.position.x - self.robot_pose.position.x)
            distance_to_zone_y = abs(self.pose_items_master_zone.position.y - self.robot_pose.position.y)
            epsilon = 2.5  # in meters (size of the circle)
            if(distance_to_zone_x < epsilon and distance_to_zone_y < epsilon):
                self.get_logger().info('At the start zone !')
                self.target_pose_publisher.publish(self.robot_pose)
                self.robot_pose_publisher.publish(self.robot_pose)
                self.robot_state = State.TASK_ALLOCATION
                time_string = str(self.get_clock().now().to_msg()).split('sec=')[1]
                time_string = time_string.split(',')[0]
                self.t2 = int(time_string)
                total = self.t2-self.t0  #get time took to complete the task (pick object from nest, drop to zone and come back to nest)
                self.get_logger().info('The total timer is  %s' % str(total))
                with open(self.filepath_log, 'a', encoding='UTF8') as ff:  #write in the csv file
                    writer = csv.writer(ff)
                    self.moved[4]=str(self.t2)
                    self.moved[5]=str(total)
                    # write the data
                    self.get_logger().info('The item ot take {}'.format(self.moved))
                    writer.writerow(self.moved)

    def destroy_node(self):
        """Call the super destroy method."""
        super().destroy_node()


def main(args=None):
    """Create a node for the task allocation pattern, spins it and handles the destruction."""
    setup_node.init_and_spin(args, TaskAllocationPattern)


if __name__ == '__main__':
    main()
