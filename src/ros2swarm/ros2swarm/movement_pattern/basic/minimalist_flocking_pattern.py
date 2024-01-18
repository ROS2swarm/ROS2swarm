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
import math
import random
import torch 
from geometry_msgs.msg import Twist
from ros2swarm.utils import setup_node
from communication_interfaces.msg import RangeData
from rclpy.qos import qos_profile_sensor_data
from ros2swarm.movement_pattern.movement_pattern import MovementPattern
from ros2swarm.utils.state import State
from ros2swarm.utils.scan_calculation_functions import ScanCalculationFunctions
from enum import IntEnum, unique
from ros2swarm.utils.kin_detection_functions import KinDetectionFunctions

def check_robot(max_range, min_range, current_ranges, threshold):
    """Check if there is a robot in the area"""
    count = sum([1 if min_range < y < max_range else 0 for y in current_ranges])
    return count if count < threshold else 0


@unique
class Directions(IntEnum):
    """Defines directions"""
    FRONT = 0
    LEFT = 1
    BEHIND = 2
    RIGHT = 3


class MinimalistFlockingPattern(MovementPattern):
    """
    Pattern to perform a flocking of the participating robots in the available area.

    The flocking pattern node creates drive commands based on the laser scan messages it receive.

    pattern_node >> publishing to the self.get_namespace()/drive_command topic
    """

    def __init__(self):
        """Initialize the minimalist flocking pattern node."""
        super().__init__('minimalist_flocking_pattern')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('minimalist_flocking_translational_velocity', 0.0),
                ('minimalist_flocking_rotational_left_velocity', 0.0),
                ('minimalist_flocking_rotational_right_velocity', 0.0),
                ('minimalist_flocking_drive_timer_period', 0),
                ('minimalist_flocking_zone1_threshold', 0.0),
                ('minimalist_flocking_zone2_threshold', 0.0),
                ('minimalist_flocking_zone3_threshold', 0.0),
                ('minimalist_flocking_zone4_threshold', 0.0),
                ('minimalist_flocking_zone2_robot_threshold', 0),
                ('minimalist_flocking_robot_threshold', 0),
                ('minimalist_flocking_zone_borders', [0.0,0.0]),
                ('max_range', 0.0),
                ('min_range', 0.0), 
                ('masking', True)
            ])

        self.state = State.INIT
        self.counter = 0
        self.direction = Twist()
        self.ranges = []

        # sensor subscription
        self.range_data_subscription = self.create_subscription(
            RangeData,
            self.get_namespace() + '/range_data',
            self.swarm_command_controlled(self.range_data_callback),
            qos_profile=qos_profile_sensor_data
        )

        # global parameters
        self.param_translational_velocity = self.get_parameter(
            "minimalist_flocking_translational_velocity").get_parameter_value().double_value
        self.param_rotational_left_velocity = self.get_parameter(
            "minimalist_flocking_rotational_left_velocity").get_parameter_value().double_value
        self.param_rotational_right_velocity = self.get_parameter(
            "minimalist_flocking_rotational_right_velocity").get_parameter_value().double_value
        self.param_drive_timer_period = self.get_parameter(
            "minimalist_flocking_drive_timer_period").get_parameter_value().integer_value
        self.param_zone1_threshold = self.get_parameter(
            "minimalist_flocking_zone1_threshold").get_parameter_value().double_value
        self.param_zone2_threshold = self.get_parameter(
            "minimalist_flocking_zone2_threshold").get_parameter_value().double_value
        self.param_zone3_threshold = self.get_parameter(
            "minimalist_flocking_zone3_threshold").get_parameter_value().double_value
        self.param_zone4_threshold = self.get_parameter(
            "minimalist_flocking_zone4_threshold").get_parameter_value().double_value
        self.param_zone2_robot_threshold = self.get_parameter(
            "minimalist_flocking_zone2_robot_threshold").get_parameter_value().integer_value
        self.param_robot_threshold = self.get_parameter(
            "minimalist_flocking_robot_threshold").get_parameter_value().integer_value
        self.param_zone_borders = self.get_parameter(
            "minimalist_flocking_zone_borders").get_parameter_value().double_array_value
        self.param_max_range = self.get_parameter(
            "max_range").get_parameter_value().double_value
        self.param_min_range = self.get_parameter(
            "min_range").get_parameter_value().double_value
        self.masking = self.get_parameter("masking").get_parameter_value().bool_value
        self.mask = []

        if self.masking:
            # load pytorch model (pretrained)
            self.model = KinDetectionFunctions.load_model() 

        self.turn_right = Twist()
        self.turn_right.angular.z = self.param_rotational_right_velocity
        self.turn_left = Twist()
        self.turn_left.angular.z = self.param_rotational_left_velocity
        self.move = Twist()
        self.move.linear.x = self.param_translational_velocity

    def range_data_callback(self, incoming_msg):
        """Publish the message to the sub pattern topic and resets the _latest variables."""
        self.ranges = [[], [], [], []]
        
        adj_ranges = ScanCalculationFunctions.adjust_ranges(incoming_msg.ranges,
                                                            self.param_min_range,
                                                            self.param_max_range)
                                                            
        # ToDo masking 
        if self.masking:
            self.mask = KinDetectionFunctions.propagate_input(incoming_msg.ranges, 3.6, self.model)
            adj_ranges = ScanCalculationFunctions.mask_ranges(adj_ranges, self.mask)
        

        # Divide the ranges of the scan in 4 parts:
        # [0] = front, [1] = left, [2] = behind, [3] = right
        for i in range(len(adj_ranges)):
            #right
            if  self.param_zone_borders[2] < incoming_msg.angles[i] <= self.param_zone_borders[3]:
                self.ranges[Directions.RIGHT].append(adj_ranges[i])
            #front
            elif self.param_zone_borders[3] < incoming_msg.angles[i] or incoming_msg.angles[i] <= self.param_zone_borders[0]:
                self.ranges[Directions.FRONT].append(adj_ranges[i])
            #left
            elif self.param_zone_borders[0] < incoming_msg.angles[i] <= self.param_zone_borders[1]:
                self.ranges[Directions.LEFT].append(adj_ranges[i])
            #rear
            elif self.param_zone_borders[1] < incoming_msg.angles[i] <= self.param_zone_borders[2]:
                self.ranges[Directions.BEHIND].append(adj_ranges[i])
              
        self.direction = self.vector_calc()
        self.command_publisher.publish(self.direction)

    def vector_calc(self):
        """State machine for the minimalist flocking behavior.
           Calculates the direction vector for the current scan."""
        self.counter += 1

        # determine nearby robots per zone  
        obstacle = self.robot_detect()

        # zone 1: avoid robots in front
        if self.state is State.AVOID or (self.state is State.INIT and
                                         (not ScanCalculationFunctions.is_obstacle_free(
                                             float(self.param_zone1_threshold),
                                             self.ranges[Directions.FRONT],
                                             self.param_robot_threshold))):
            behave = self.collision_avoid()
            self.state = State.AVOID

        # zone 2: separation
        elif self.state is State.DISPERSION or (
                self.state is State.INIT and (obstacle[0] or obstacle[1] or obstacle[2])):
            behave = self.separation()
            self.state = State.DISPERSION

        # zone 3: attraction/cohesion
        elif self.state is State.ATTRACTION or (self.state is State.INIT and
                                                (check_robot(self.param_max_range,
                                                             self.param_zone3_threshold,
                                                             self.ranges[Directions.LEFT],
                                                             self.param_robot_threshold) or
                                                 check_robot(self.param_max_range,
                                                             self.param_zone3_threshold,
                                                             self.ranges[Directions.RIGHT],
                                                             self.param_robot_threshold))):
            behave = self.cohesion()
            self.state = State.ATTRACTION

        # zone 4: cohesion - turn towards robot in the back
        elif self.state is State.ATTRACTION_BACK or (self.state is State.INIT and
                                                     check_robot(self.param_max_range,
                                                                 self.param_zone4_threshold,
                                                                 self.ranges[Directions.BEHIND],
                                                                 self.param_robot_threshold)):
            behave = self.cohesion_behind()
            self.state = State.ATTRACTION_BACK

        # else: drive straight
        else:
            behave = self.move
            self.counter = 0

        # Counter to hold the behavior for the number of periods defined py timer period parameter
        if self.counter >= self.param_drive_timer_period:
            self.state = State.INIT
            self.counter = 0
        return behave

    def collision_avoid(self):
        """zone 1: avoid the obstacle in front"""
        min_range = min(self.ranges[Directions.FRONT])
        direction = self.turn_left
        if self.direction.angular.z == 0.0:
            for i in range(0, int(len(self.ranges[Directions.FRONT])/2)):
                if self.ranges[Directions.FRONT][i] == min_range:
                    direction = self.turn_right
                    break
        else:
            direction = self.direction
        return direction

    def robot_detect(self):
        """zone2: Check for a robot which is to close to us"""
        # considering up to self.param_zone2_robot_threshold occupied ranges as a robot,
        # otherwise it might be a wall
        obstacle = [False, False, False]
        # left
        obstacle[0] = (1 < ScanCalculationFunctions.sum_ranges(
            float(self.param_zone2_threshold),
            self.ranges[Directions.LEFT]) < self.param_zone2_robot_threshold)
        # front
        obstacle[1] = (1 < ScanCalculationFunctions.sum_ranges(
            float(self.param_zone2_threshold),
            self.ranges[Directions.FRONT]) < self.param_zone2_robot_threshold)
        # right
        obstacle[2] = (1 < ScanCalculationFunctions.sum_ranges(
            float(self.param_zone2_threshold),
            self.ranges[Directions.RIGHT]) < self.param_zone2_robot_threshold)
        return obstacle

    def separation(self):
        """zone 2: avoid too close obstacles"""
        direction = self.direction
        obstacle = self.robot_detect()

        if obstacle[0]:
            direction = self.turn_right
        elif obstacle[1]:
            if self.direction.angular.z == 0:
                if random.randint(0, 1) == 0:
                    direction = self.turn_left
                else:
                    direction = self.turn_right
            else:
                direction = self.direction
        elif obstacle[2]:
            direction = self.turn_left
        return direction

    def cohesion(self):
        """zone 3: turn to the Robot on the left/right"""
        direction = Twist()
        robot = [check_robot(self.param_max_range, self.param_zone3_threshold,
                             self.ranges[Directions.LEFT],
                             self.param_robot_threshold),
                 check_robot(self.param_max_range, self.param_zone3_threshold,
                             self.ranges[Directions.RIGHT],
                             self.param_robot_threshold)]

        if robot[0] == robot[1]:
            # equal number of robots on both sides
            return self.move
        elif robot[0] > robot[1]:
            direction.angular.z = self.turn_left.angular.z
            return direction
        elif robot[1] > robot[0]:
            direction.angular.z = self.turn_right.angular.z
            return direction
        return self.move

    def cohesion_behind(self):
        """zone 4: Stop and turn if there is a robot behind us"""
        min_range = min(self.ranges[Directions.BEHIND])
        direction = self.turn_right
        if self.direction.angular.z == 0.0:
            for i in range(0, int(len(self.ranges[Directions.BEHIND])/2)):
                if self.ranges[Directions.BEHIND][i] == min_range:
                    direction = self.turn_left
            return direction
        else:
            return self.direction


def main(args=None):
    """Create a node for the flocking pattern, spins it and handles the destruction."""

    setup_node.init_and_spin(args, MinimalistFlockingPattern)


if __name__ == '__main__':
    main()
