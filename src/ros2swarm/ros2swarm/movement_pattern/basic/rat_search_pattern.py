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
import random
import time

from geometry_msgs.msg import Twist
from ros2swarm.utils import setup_node
from ros2swarm.utils.maze_graph import MazeGraph
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from ros2swarm.movement_pattern.movement_pattern import MovementPattern

from ros2swarm.utils.scan_calculation_functions import ScanCalculationFunctions

from communication_interfaces.msg import StringMessage

import numpy as np
import math
import numpy

import cv2
from ros2swarm.utils.state import State

from cv_bridge import CvBridge

bridge = CvBridge()


def vector_length(vec):
    """
    Returns the length of the given vector
    """
    return vec[0] ** 2 + vec[1] ** 2


def pol2cart(r, phi):
    """
    Converts polar coordinates into cartesian coordinates
    :param r: length
    :param phi: angle in degrees
    :Returns The corresponding cartesian vector
    """
    x = r * math.cos(math.radians(phi))
    y = r * math.sin(math.radians(phi))
    return np.array([x, y])
    #TODO move to a vector util place


class RatSearchPattern(MovementPattern):
    """
    Pattern to search a maze based on rat behavior.
    """

    def __init__(self):
        """Initialize the rat search pattern node."""
        super().__init__('rat_search_pattern')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('rat_search_max_range', None),
                ('rat_search_min_range', None),
                ('rat_search_timer_period', None),
                ('rat_search_color_red', None),
                ('rat_search_color_green', None),
                ('rat_search_color_blue', None),
                ('rat_search_right_distance', None),
                ('rat_search_left_distance', None),
                ('rat_search_distance_buffer', None),

                ('rat_search_wall_right_degree', None),
                ('rat_search_wall_degree', None),
                ('rat_search_wall_batch_size', None),

                ('max_translational_velocity', None),
                ('max_rotational_velocity', None),
            ])

        self.scan_subscription = self.create_subscription(
            LaserScan,
            self.get_namespace() + '/scan',
            self.swarm_command_controlled(self.scan_callback),
            qos_profile=qos_profile_sensor_data
        )

        self.camera_subscription = self.create_subscription(
            Image,
            self.get_namespace() + '/camera/image_raw',
            self.swarm_command_controlled(self.camera_callback),
            qos_profile=qos_profile_sensor_data
        )

        self.information_publisher = self.create_publisher(
            StringMessage,
            self.get_namespace() + '/information',
            10
        )

        self.param_max_range = float(self.get_parameter(
            "rat_search_max_range").get_parameter_value().double_value)
        self.param_min_range = float(self.get_parameter(
            "rat_search_min_range").get_parameter_value().double_value)
        self.param_color_red = self.get_parameter(
            "rat_search_color_red").get_parameter_value().double_value
        self.param_color_green = self.get_parameter(
            "rat_search_color_green").get_parameter_value().double_value
        self.param_color_blue = self.get_parameter(
            "rat_search_color_blue").get_parameter_value().double_value

        self.param_right_distance = self.get_parameter(
            "rat_search_right_distance").get_parameter_value().double_value
        self.param_left_distance = self.get_parameter(
            "rat_search_left_distance").get_parameter_value().double_value
        self.param_distance_buffer = self.get_parameter(
            "rat_search_distance_buffer").get_parameter_value().double_value

        self.param_wall_right_degree = self.get_parameter(
            "rat_search_wall_right_degree").get_parameter_value().integer_value
        self.param_wall_degree = self.get_parameter(
            "rat_search_wall_degree").get_parameter_value().integer_value
        self.param_wall_batch_size = self.get_parameter(
            "rat_search_wall_batch_size").get_parameter_value().integer_value

        self.param_max_translational_velocity = self.get_parameter(
            "max_translational_velocity").get_parameter_value().double_value
        self.param_max_rotational_velocity = self.get_parameter(
            "max_rotational_velocity").get_parameter_value().double_value
        timer_period = float(self.get_parameter(
            "rat_search_timer_period").get_parameter_value().double_value)

        self.timer = self.create_timer(timer_period, self.swarm_command_controlled_timer(self.timer_callback))

        self.state = State.INIT
        self.current_scan = None
        self.current_image = None
        self.current_image_dominant_color = None
        self.direction = Twist()
        self.invocation_counter = 0
        self.location_graph = MazeGraph()
        self.last_decision = State.TUNNEL
        self.start_time = int(time.time())
        self.goal_found = False
        self.goal_founding_published = False

        info = StringMessage()
        info.data = 'Starting RUN'
        self.information_publisher.publish(info)

    def get_direction_batch_average(self, target_direction):

        under = target_direction - self.param_wall_batch_size
        upper = target_direction + self.param_wall_batch_size
        num = 2 * self.param_wall_batch_size + 1

        sum_up = 0.0
        for e in range(under, upper):
            if self.current_scan.ranges[e] <= 3.5:  # TODO make configurable
                sum_up += self.current_scan.ranges[e]

        return sum_up > 0.0, sum_up / num

    def wall_follow(self, right_degree_front, right_degree_back):

        # get vector a
        a_valid, a = self.get_direction_batch_average(right_degree_front)
        vec_a = pol2cart(a, right_degree_front)

        # get vector b
        b_valid, b = self.get_direction_batch_average(right_degree_back)
        vec_b = pol2cart(b, right_degree_back)

        if not (a_valid and b_valid):
            return False, Twist()

        vec_c = -vec_b + vec_a

        # turn by 90 degree
        vec_c_t = np.array([-vec_c[1], vec_c[0]])

        # norm vector
        vec_c_t_normed = vec_c_t / numpy.linalg.norm(vec_c_t)

        vec_wt = vec_c_t_normed * self.param_right_distance

        vec_t = vec_a + vec_wt

        vector = np.array([0.0, 0.0])
        vector[0] = numpy.sqrt(vec_t[0] ** 2 + vec_t[1] ** 2)  # length
        vector[1] = np.arctan2(vec_t[1], vec_t[0])  # phi

        return True, ScanCalculationFunctions.create_normed_twist_message(
            vector,
            self.param_max_translational_velocity,
            self.param_max_rotational_velocity)

    def get_time_since_start(self):
        """Time in seconds since pattern start up."""
        return int(time.time()) - self.start_time

    def timer_callback(self):

        if self.state != State.INIT:

            self.invocation_counter = self.invocation_counter + 1

            if self.location_graph.is_next_location_leaf():
                # take a look on the image
                goal_reached = self.check_if_goal_in_image()
                if goal_reached:
                    self.location_graph.mark_as_goal()
                    if not self.goal_founding_published:
                        self.goal_founding_published = True
                        info = StringMessage()
                        info.data = 'GOAL:{}, Location ID:{}'.format(self.get_time_since_start(),
                                                                     self.location_graph.lastLocation.node_id)
                        self.information_publisher.publish(info)
                    self.goal_found = True
                    self.get_logger().debug('goal found: {}, TIME:{}'.format(goal_reached, self.get_time_since_start()))

            num, _, _ = self.identify_segments(3.5)

            self.get_logger().debug(
                'next leaf?{},num open segments: {}'.format(self.location_graph.is_next_location_leaf(), num))
            if self.location_graph.is_next_location_leaf() and num == 1:
                # update to location_graph when turning in dead end
                self.get_logger().info('turn counter {}'.format(self.invocation_counter))
                if self.invocation_counter >= 150:
                    # TODO do only once --> next_node == leaf !! done already
                    self.get_logger().info('ENDING-UPDATE-TIME:{}'.format(self.get_time_since_start()))
                    self.location_graph.update(State.ENDING, self.get_time_since_start())
                    self.get_logger().info(self.location_graph)

                    info = StringMessage()
                    info.data = 'Ending:{}, Location ID:{}'.format(self.get_time_since_start(),
                                                                   self.location_graph.lastLocation.node_id)
                    self.information_publisher.publish(info)

                    self.invocation_counter = 0
                    self.get_logger().debug('>>>ENDING<<<')

        self.direction = Twist()
        self.get_logger().debug('State is: {}'.format(self.state))

        if self.state == State.INIT:
            self.state, self.direction = self.init_movement()
        elif self.state == State.TUNNEL:
            self.state, self.direction = self.tunnel_movement()
        elif self.state == State.CROSSING:
            self.state = self.crossing()
        elif self.state == State.TUNNEL_CORNER:
            self.state, self.direction = self.tunnel_corner_movement()
        elif self.state == State.CROSSING_LEFT:
            self.state, self.direction = self.crossing_left_movement()
        elif self.state == State.CROSSING_RIGHT:
            self.state, self.direction = self.crossing_right_movement()
        elif self.state == State.SEARCH_WALL:
            self.state, self.direction = self.search_wall_movement()
        elif self.state == State.START_CHAMBER:
            pass
        elif self.state == State.ENDING:
            pass

        self.get_logger().debug('direction:{}'.format(self.direction))
        self.command_publisher.publish(self.direction)

    def search_wall_movement(self):
        # TODO optimize this code

        # TODO move calculation to init section
        right_degree = self.param_wall_right_degree
        right_degree_front = right_degree + int(self.param_wall_degree / 2)
        right_degree_back = right_degree - int(self.param_wall_degree / 2)
        r_valid, r = self.get_direction_batch_average(right_degree)
        f_valid, f = self.get_direction_batch_average(right_degree_front)
        b_valid, b = self.get_direction_batch_average(right_degree_back)
        if r_valid and f_valid and b_valid:
            direction = Twist()
            state = State.TUNNEL
        else:
            direction = Twist()
            direction.linear.x = self.param_max_translational_velocity
            state = State.SEARCH_WALL
        return state, direction

    def crossing_left_movement(self):
        state = State.CROSSING_LEFT

        dis = 2.2
        openings, _, _ = self.identify_segments(dis)
        if openings != 3:
            state = State.TUNNEL
            direction = Twist()
            direction.linear.x = self.param_max_translational_velocity
        else:
            direction = self.turn_left()
        return state, direction

    def turn_left(self):
        dis = 3.5
        # find left end of opening
        last_is_opening = self.current_scan.ranges[0] >= dis
        left_end = 3
        for e in range(0, len(self.current_scan.ranges) - 1, +1):
            current_is_opening = self.current_scan.ranges[e] >= dis
            if last_is_opening and not current_is_opening:
                left_end = e
                break
            last_is_opening = current_is_opening
        last_is_opening = False
        right_end = 0
        end_reached = True
        for e in range(left_end, 0, -1):
            current_is_opening = self.current_scan.ranges[e] >= dis
            if last_is_opening and not current_is_opening:
                right_end = e
                end_reached = False
                break
            last_is_opening = current_is_opening
        left_right = False
        if end_reached:
            left_right = True
            for e in range(len(self.current_scan.ranges) - 1, 0, -1):
                current_is_opening = self.current_scan.ranges[e] >= dis
                if last_is_opening and not current_is_opening:
                    right_end = e
                    end_reached = False
                    break
        if end_reached:
            direction = Twist()
            direction.linear.x = self.param_max_translational_velocity
            direction.angular.z = self.param_max_rotational_velocity / 5
            pass
        else:

            ranges = []
            for i in range(0, len(self.current_scan.ranges) - 1):
                if self.current_scan.ranges[i] > 3.5:
                    ranges.append(3.5)
                else:
                    ranges.append(self.current_scan.ranges[i])

            if left_right:
                sub_ranges = []
                for i in range(right_end, 359):
                    sub_ranges.append(pol2cart(ranges[i], i))
                for i in range(0, left_end):
                    sub_ranges.append(pol2cart(ranges[i], i))
            else:
                sub_ranges = []
                for i in range(right_end, left_end):
                    sub_ranges.append(pol2cart(ranges[i], i))

            sub_ranges2 = []
            for i in range(0, int(len(sub_ranges) / 2)):
                sub_ranges2.append(sub_ranges[i])

            vector = ScanCalculationFunctions.combine_vectors(sub_ranges2)
            # vector = ScanCalculationFunctions.add_attraction(vector, -2.0)
            direction = ScanCalculationFunctions.create_normed_twist_message(vector,
                                                                             self.param_max_translational_velocity,
                                                                             self.param_max_rotational_velocity)

            front = Twist()
            front.linear.x = self.param_max_translational_velocity / 26.0
            direction = ScanCalculationFunctions.combine_twist_messages(direction, front,
                                                                        self.param_max_translational_velocity,
                                                                        self.param_max_translational_velocity)
        return direction

    def crossing_right_movement(self):
        state = State.CROSSING_RIGHT
        dis = 2.2
        openings, _, _ = self.identify_segments(dis)
        if openings != 3:
            state = State.TUNNEL
            direction = Twist()
            direction.linear.x = self.param_max_translational_velocity
        else:
            direction = self.turn_right()
        return state, direction

    def turn_right(self):
        dis = 3.5
        last_is_opening = self.current_scan.ranges[len(self.current_scan.ranges) - 1] >= dis
        right_end = 356
        for e in range(len(self.current_scan.ranges) - 1, 0, -1):
            current_is_opening = self.current_scan.ranges[e] >= dis
            if last_is_opening and not current_is_opening:
                right_end = e
                break
            last_is_opening = current_is_opening
        # find left end of opening
        last_is_opening = False
        left_end = 0
        end_reached = True
        for e in range(right_end, len(self.current_scan.ranges) - 1, +1):
            current_is_opening = self.current_scan.ranges[e] >= dis
            if last_is_opening and not current_is_opening:
                left_end = e
                end_reached = False
                break
            last_is_opening = current_is_opening
        left_right = False
        if end_reached:
            left_right = True
            for e in range(0, len(self.current_scan.ranges) - 1, +1):
                current_is_opening = self.current_scan.ranges[e] >= dis
                if last_is_opening and not current_is_opening:
                    left_end = e
                    end_reached = False
                    break
        if end_reached:
            direction = Twist()
            direction.linear.x = self.param_max_translational_velocity
            direction.angular.z = -self.param_max_rotational_velocity / 5
            pass
        else:

            ranges = []
            for i in range(0, len(self.current_scan.ranges) - 1):
                if self.current_scan.ranges[i] > 3.5:
                    ranges.append(3.5)
                else:
                    ranges.append(self.current_scan.ranges[i])

            if left_right:
                sub_ranges = []
                for i in range(right_end, 359):
                    sub_ranges.append(pol2cart(ranges[i], i))
                for i in range(0, left_end):
                    sub_ranges.append(pol2cart(ranges[i], i))
            else:
                sub_ranges = []
                for i in range(right_end, left_end):
                    sub_ranges.append(pol2cart(ranges[i], i))

            sub_ranges2 = []
            for i in range(0, int(len(sub_ranges) / 2)):
                sub_ranges2.append(sub_ranges[i])

            vector = ScanCalculationFunctions.combine_vectors(sub_ranges2)
            # vector = ScanCalculationFunctions.add_attraction(vector, -2.0)
            direction = ScanCalculationFunctions.create_normed_twist_message(vector,
                                                                             self.param_max_translational_velocity,
                                                                             self.param_max_rotational_velocity)

            front = Twist()
            front.linear.x = self.param_max_translational_velocity / 26.0
            direction = ScanCalculationFunctions.combine_twist_messages(direction, front,
                                                                        self.param_max_translational_velocity,
                                                                        self.param_max_translational_velocity)

        return direction

    def tunnel_corner_movement(self):
        state = State.TUNNEL_CORNER
        right_degree = self.param_wall_right_degree
        right_degree_front = right_degree + int(self.param_wall_degree / 2)
        right_degree_back = right_degree - int(self.param_wall_degree / 2)
        valid, direction = self.wall_follow(right_degree_front, right_degree_back)
        if valid:
            state = State.TUNNEL
        else:
            left_search = 0
            right_search = 359

            dis = 3.5
            # dis = 3.0
            for e in range(0, 179, +1):
                current_is_opening = self.current_scan.ranges[e] >= dis
                if current_is_opening:
                    left_search = e
                    break

            for e in range(len(self.current_scan.ranges) - 1, 180, -1):
                current_is_opening = self.current_scan.ranges[e] >= dis
                if current_is_opening:
                    right_search = e
                    break

            # identify which opening is nearer to front direction
            ranges = []
            for i in range(0, len(self.current_scan.ranges)):
                if self.current_scan.ranges[i] > 3.5:
                    ranges.append(3.5)
                else:
                    ranges.append(self.current_scan.ranges[i])

            rr = 359 - right_search
            ll = left_search
            if rr <= ll:
                direction = self.turn_right()
            # direction = self.turn_right()
            else:
                direction = self.turn_left()
                # direction = Twist()
                # direction.linear.x = self.param_max_translational_velocity
        return state, direction

    def init_movement(self):
        new_state = State.INIT
        direction = Twist()
        fully_initialized = not (self.current_image is None
                                 or self.current_scan is None)
        if fully_initialized:
            direction.linear.x = self.param_max_translational_velocity
            new_state = State.SEARCH_WALL
        return new_state, direction

    def tunnel_movement(self):
        next_state = State.TUNNEL
        direction = Twist()
        # TODO DETECT IF CROSSING IS REACHED
        dis = 1.75
        openings, _, _ = self.identify_segments(dis)
        dis = 3.5
        openings2, _, _ = self.identify_segments(dis)
        if openings >= 3 and openings2 >= 3:
            # really crossing?
            self.get_logger().info('>>CROSSING-DETECTED')
            direction = Twist()
            direction.linear.x = self.param_max_translational_velocity / 26.0
            next_state = State.CROSSING
        else:
            right_degree = self.param_wall_right_degree
            right_degree_front = right_degree + int(self.param_wall_degree / 2)
            right_degree_back = right_degree - int(self.param_wall_degree / 2)
            valid, direction = self.wall_follow(right_degree_front, right_degree_back)
            if not valid:
                # TODO 1) try following front opening
                # TODO 2) add front attraction to direction
                ## TODO --> TODO alternative if basis don't work, follow front opening + detect crossing independently

                # retry with smaller values
                right_degree = self.param_wall_right_degree
                right_degree_front = right_degree + int(self.param_wall_degree / 3)
                right_degree_back = right_degree - int(self.param_wall_degree / 3)
                valid, direction = self.wall_follow(right_degree_front, right_degree_back)
                self.get_logger().debug('>> smaller')

                if not valid:
                    right_degree = self.param_wall_right_degree
                    right_degree_front = right_degree + int(self.param_wall_degree / 4)
                    right_degree_back = right_degree - int(self.param_wall_degree / 2)
                    valid, direction = self.wall_follow(right_degree_front, right_degree_back)
                    self.get_logger().debug('>> smaller2')

                    if not valid:
                        self.get_logger().debug('>> TUNNEL DOES NOT READ')
                        next_state = State.CROSSING
                        direction = Twist()
                        direction.linear.x = self.param_max_translational_velocity

            # TODO try:
            front = Twist()
            front.linear.x = self.param_max_translational_velocity / 26.0
            direction = ScanCalculationFunctions.combine_twist_messages(direction, front,
                                                                        self.param_max_translational_velocity,
                                                                        self.param_max_translational_velocity)

        return next_state, direction

    def crossing(self):

        # state = State.CROSSING

        dis = 2.2
        # dis = 2.0
        # dis = 2.1
        open_segments, _, _ = self.identify_segments(dis)
        self.get_logger().debug('open segments:{}'.format(open_segments))
        if open_segments <= 2:
            state = State.TUNNEL_CORNER

        if open_segments >= 3:
            open_segments2, _, _ = self.identify_segments(3.5)
            if open_segments2 >= 3:
                state = self.decide_direction()
            else:
                state = State.TUNNEL_CORNER

        return state

    def decide_direction(self):
        if self.location_graph.nextLocation.isLeaf:
            # this should not be called at this time -> ignoring wish and assume we are in a tunnel
            self.get_logger().warn('LEAF!! - Location Graph corrupted')

            info = StringMessage()
            info.data = '<<Graph corrupted>> Node:{}, Location ID:{}'.format(self.get_time_since_start(),
                                                                             self.location_graph.lastLocation.node_id)
            self.information_publisher.publish(info)

            # TODO kill if current make parameter
            exit()  # TODO remove for swarm usage

            return State.CROSSING_RIGHT if random.randint(0, 100) <= 50 else State.CROSSING_LEFT

        last_visited = self.location_graph.calc_tube_last_visited()
        unexplored = self.location_graph.calc_tube_less_explored()
        self.get_logger().debug('last_visited:{}, unexplored:{}'.format(last_visited, unexplored))

        dominant = random.randint(0, 100)
        r = random.randint(0, 100)

        if self.goal_found:
            if dominant <= 45:
                # decision based on unexplored
                if r <= 78:
                    # long ago
                    state = last_visited[1]
                else:
                    # recent
                    state = last_visited[0]
            else:
                # decision based on last_visited
                if r <= 60:
                    # less explored
                    state = unexplored[0]
                else:
                    # more explored
                    state = unexplored[1]
        else:
            if dominant <= 64:
                # decision based on unexplored
                if r <= 78:
                    # long ago
                    state = last_visited[1]
                else:
                    # recent
                    state = last_visited[0]
            else:
                # decision based on last_visited
                if r <= 60:
                    # less explored
                    state = unexplored[0]
                else:
                    # more explored
                    state = unexplored[1]

        # if v > 50:
        #    state = State.CROSSING_RIGHT
        # else:
        #    state = State.CROSSING_LEFT

        if self.invocation_counter >= 200:
            # expect this high number of invocation as that a new crossing could be reached
            self.get_logger().info('CROSSING-UPDATE-TIME:{}'.format(self.get_time_since_start()))
            self.location_graph.update(state, self.get_time_since_start())
            self.get_logger().info(self.location_graph)

            info = StringMessage()
            info.data = 'Node:{}, Location ID:{}, Turning:{}'.format(self.get_time_since_start(),
                                                                     self.location_graph.lastLocation.node_id, state)
            self.information_publisher.publish(info)

            self.invocation_counter = 0
        else:
            info = StringMessage()
            info.data = 'Crossing but to early'
            self.information_publisher.publish(info)
            # counter not valid choose last option again
            state = self.last_decision

        self.last_decision = state
        return state

    def identify_segments(self, dis):

        list_segment_start = []
        list_segment_end = []
        open_segments = 0

        last_ray_wall = self.current_scan.ranges[len(self.current_scan.ranges) - 1] <= dis
        for i in range(0, len(self.current_scan.ranges)):
            current_is_wall = self.current_scan.ranges[i] <= dis
            if last_ray_wall and current_is_wall:
                # ongoing wall
                pass
            elif last_ray_wall and not current_is_wall:
                # end wall -> start of opening
                open_segments += 1
                list_segment_start.append(i)
            elif not last_ray_wall and current_is_wall:
                # end opening -> start of wall
                list_segment_end.append(i)
            else:
                # ongoing opening
                pass
            last_ray_wall = current_is_wall
        return open_segments, list_segment_start, list_segment_end

    def scan_callback(self, incoming_msg: LaserScan):
        """Call back if a new scan msg is available."""
        self.current_scan = incoming_msg

    def camera_callback(self, raw_image_msg: Image):
        """Call back if a new scan msg is available."""
        self.current_image = raw_image_msg

    def check_if_goal_in_image(self):

        height = self.current_image.height
        width = self.current_image.width
        # data = raw_image_msg.data
        cv_image = bridge.imgmsg_to_cv2(self.current_image, desired_encoding='passthrough')
        # cv_image = bridge.imgmsg_to_cv2(raw_image_msg, desired_encoding=raw_image_msg.encoding)

        # calc region of interest (the middle third)
        h = int(height / 4)
        sub = cv_image[h:2 * h, 0:width]

        # calc dominant color
        # https://stackoverflow.com/questions/43111029/how-to-find-the-average-colour-of-an-image-in-python-with-opencv/43111221
        pixels = np.float32(sub.reshape(-1, 3))

        n_colors = 5
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 200, .1)
        flags = cv2.KMEANS_RANDOM_CENTERS

        _, labels, palette = cv2.kmeans(pixels, n_colors, None, criteria, 10, flags)
        _, counts = np.unique(labels, return_counts=True)
        dominant = palette[np.argmax(counts)]
        self.current_image_dominant_color = dominant

        goal_reached = \
            dominant[0] > self.param_color_red or \
            dominant[1] > self.param_color_green or \
            dominant[2] > self.param_color_blue
        self.get_logger().debug(
            'dominant color:{},r:{},g:{},b:{},goal:{}'.format(dominant, self.param_color_red, self.param_color_green,
                                                              self.param_color_blue, goal_reached))
        self.get_logger().debug('Target found {}'.format(goal_reached))
        return goal_reached


def main(args=None):
    """Create a node for the rat search pattern and handle the setup."""
    setup_node.init_and_spin(args, RatSearchPattern)


if __name__ == '__main__':
    main()
