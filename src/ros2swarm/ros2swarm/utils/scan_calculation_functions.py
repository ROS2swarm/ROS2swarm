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

import math

import numpy as np
from geometry_msgs.msg import Twist
from rclpy import logging

from sensor_msgs.msg import LaserScan
from typing import List

from enum import Enum, unique


@unique
class ReductionOption(Enum):
    """Reduction options on objects."""

    NEAREST = 0
    MEAN = 1
    FARTHEST = 2


def pol2cart(rho: float, phi: float):
    """
    Converts polar coordinates to cartesian coordinates.
    :param rho: length
    :param phi: angle in radians
    """
    x = rho * math.cos(phi)
    y = rho * math.sin(phi)
    return x, y


class ScanCalculationFunctions:
    """Provides function for calculations around scan values."""

    @staticmethod
    def adjust_ranges(ranges, min_range, max_range):
        """
        Adjust all ranges over max_range to max_range and
        all ranges under min_range to 0.0 value
        returns the adjusted ranges

        :param ranges: The ranges to adjust
        :param min_range: values under this range are set to 0.0
        :param max_range: values over this range are set to max_range
        :return: the ranges with adjusted length
        """
        ranges = [max_range if x > max_range else x for x in ranges]
        ranges = [0.0 if x < min_range else x for x in ranges]
        return ranges

    @staticmethod
    def linear_rating(ranges, max_range):
        """
        Map all ranges to the interval [0,1] using a linear function.

        As nearer the range is to max_range as __less__ important it is.
        Allowed input range for ranges is [0,max_range]
        """
        return [1 - (x / max_range) for x in ranges]

    @staticmethod
    def linear_rating2(ranges, max_range):
        """
        Map all ranges to the interval [0,1] using a linear function.

        As nearer the range is to max_range as __more__ important it is.
        Allowed input range for ranges is [0,max_range]
        """
        return [(x / max_range) for x in ranges]

    @staticmethod
    def calculate_vectors_from_normed_ranges(ranges, sensor_msgs_laserscan: LaserScan):
        """Calculate a vector for each measured range in ranges.
        Before this function is called the values need to be normed to [0.0,1.0]"""
        result_ranges = []
        for i, e in enumerate(ranges):
            if e < 1.0:
                result_ranges.append(
                    pol2cart(e, i * sensor_msgs_laserscan.angle_increment + sensor_msgs_laserscan.angle_min)
                    # (e * math.cos(i * sensor_msgs_laserscan.angle_increment + sensor_msgs_laserscan.angle_min),
                    # e * math.sin(i * sensor_msgs_laserscan.angle_increment + sensor_msgs_laserscan.angle_min))
                )
        return result_ranges

    @staticmethod
    def combine_vectors(vectors):
        """Combine the vectors to a single one by summing them up."""
        vector = np.matrix([[0.0], [0.0]])
        for _, e in enumerate(vectors):
            vector[0] += e[0]
            vector[1] += e[1]
        return vector

    @staticmethod
    def flip_vector(vector):
        """Flip the direction of the given vector."""
        return -1 * vector

    @staticmethod
    def rotate_vector(vector, angle_min):
        """Rotate given vector by the angle offset using the given angle_min."""
        rot = np.matrix(
            [
                [math.cos(-angle_min), -math.sin(-angle_min)],
                [math.sin(-angle_min), math.cos(-angle_min)]
            ]
        )
        return rot * vector

    @staticmethod
    def add_attraction(vector, attraction):
        """
        Add the given attraction to the given vector.

        This attraction only affects the linear part
        """
        vector[0] += attraction
        return vector

    @staticmethod
    def create_normed_twist_message(vector, max_translational_velocity, max_rotational_velocity):
        """
        Create a normed Twist message.

        Norm the given vector to not exceed the given max translational velocity and
        the rotational velocity of the robot.
        """
        direction = Twist()
        vector = vector / np.linalg.norm(vector)
        direction.linear.x = float(max_translational_velocity * vector[0])
        direction.angular.z = float(
            max_rotational_velocity * math.asin(vector[1] / math.hypot(vector[0], vector[1])))
        return direction

    @staticmethod
    def potential_field(front_attraction, max_range, max_rotational_velocity,
                        max_translational_velocity,
                        min_range, sensor_msgs_laserscan, threshold, angle_min=None):
        """
        Create a potential field based on the given LaserScan message and the other parameters.

        The second return value is true if no obstacle within the max range is detected.
        threshold is the number of scans below the max_range which is needed to detect obstacles.
        """
        ranges = ScanCalculationFunctions.adjust_ranges(sensor_msgs_laserscan.ranges, min_range,
                                                        max_range)
        obstacle_free = ScanCalculationFunctions.is_obstacle_free(max_range, ranges, threshold)

        ranges = ScanCalculationFunctions.linear_rating(ranges, max_range)

        vector = ScanCalculationFunctions.calculate_vector_normed(front_attraction, ranges,
                                                                  sensor_msgs_laserscan, angle_min)
        direction = ScanCalculationFunctions.create_normed_twist_message(
            vector,
            max_translational_velocity,
            max_rotational_velocity)

        # if NaN then create stop twist message
        if np.isnan(direction.linear.x):
            logging.get_logger('ScanCalculationFunctions').error(
                'calculated Twist message contains NaN value, '
                'adjusting to a stop message')
            direction = Twist()

        return direction, obstacle_free

    @staticmethod
    def calculate_vector_normed(front_attraction, ranges, sensor_msgs_laserscan, angle_min):
        """
        Calculate a vector from laser scan.

        Calculate a combined and rotated vector from the normed ranges and
        apply the given front attraction to it
        """

        angle_min = sensor_msgs_laserscan.angle_min if angle_min is None else angle_min
        vectors = ScanCalculationFunctions.calculate_vectors_from_normed_ranges(ranges, sensor_msgs_laserscan)
        vector = ScanCalculationFunctions.combine_vectors(vectors)
        vector = ScanCalculationFunctions.flip_vector(vector)
        vector = ScanCalculationFunctions.rotate_vector(vector, angle_min)
        vector = ScanCalculationFunctions.add_attraction(vector, front_attraction)
        return vector

    @staticmethod
    def is_obstacle_free(max_range, ranges, threshold):
        """Return true if no obstacle is detected within the max_range, false otherwise."""
        return ScanCalculationFunctions.sum_ranges(max_range, ranges) <= threshold

    @staticmethod
    def sum_ranges(max_range, ranges):
        """Return the number of ranges shorter than max_range and greater then zero."""
        return sum([1 if 0 < y < max_range else 0 for y in ranges])

    @staticmethod
    def sum_adjusted_ranges(max_range, min_range, scan):
        """Adjust the ranges from the scan and sum up the ones between min and max range."""
        adjusted_ranges = ScanCalculationFunctions.adjust_ranges(scan.ranges, min_range, max_range)
        return ScanCalculationFunctions.sum_ranges(max_range, adjusted_ranges)

    @staticmethod
    def is_adjusted_obstacle_free(max_range, min_range, scan, threshold):
        """
        Adjust the scan and check if it is obstacle free.

        Returns true if there is no number of ranges greater than the threshold between min and
        max range.
        """
        adjusted_ranges = ScanCalculationFunctions.adjust_ranges(scan.ranges, min_range, max_range)
        return ScanCalculationFunctions.is_obstacle_free(max_range, adjusted_ranges, threshold)

    @staticmethod
    def adjusted_sum(max_range, min_range, scan):
        """Returns the sum of the ranges of the scan. Adjust the ranges of the scan to the
        interval given by max_range and min_range."""
        adjusted_ranges = ScanCalculationFunctions.adjust_ranges(scan.ranges, min_range, max_range)
        return ScanCalculationFunctions.sum_ranges(max_range, adjusted_ranges)

    @staticmethod
    def attraction_field(front_attraction, max_range, max_rotational_velocity,
                         max_translational_velocity,
                         min_range, sensor_msgs_laserscan, threshold, angle_min):
        """
        Create a potential field based on the given LaserScan message and the other parameters.

        The second return value is true if no obstacle within the max range is detected.
        threshold is the number of scans below the max_range which is needed to detect obstacles.
        The nearest range is __most__ important.
        """
        ranges = ScanCalculationFunctions.adjust_ranges(sensor_msgs_laserscan.ranges, min_range,
                                                        max_range)
        obstacle_free = ScanCalculationFunctions.is_obstacle_free(max_range, ranges, threshold)

        ranges = ScanCalculationFunctions.linear_rating(ranges, max_range)

        vector = ScanCalculationFunctions.calculate_vector_normed(front_attraction, ranges,
                                                                  sensor_msgs_laserscan, angle_min)
        vector = ScanCalculationFunctions.flip_vector(vector)
        direction = ScanCalculationFunctions.create_normed_twist_message(
            vector,
            max_translational_velocity,
            max_rotational_velocity)

        # if NaN then create stop twist message
        if np.isnan(direction.linear.x):
            logging.get_logger('ScanCalculationFunctions').error(
                'calculated Twist message contains NaN value, '
                'adjusting to a stop message')
            direction = Twist()

        return direction, obstacle_free

    @staticmethod
    def repulsion_field(front_attraction, max_range, max_rotational_velocity,
                        max_translational_velocity,
                        min_range, sensor_msgs_laserscan, threshold, angle_min):
        """
        Create a potential field based on the given LaserScan message and the other parameters.

        The second return value is true if no obstacle within the max range is detected.
        threshold is the number of scans below the max_range which is needed to detect obstacles.
        The nearest range is __least__ important.
        """
        ranges = ScanCalculationFunctions.adjust_ranges(sensor_msgs_laserscan.ranges, min_range,
                                                        max_range)
        obstacle_free = ScanCalculationFunctions.is_obstacle_free(max_range, ranges, threshold)

        ranges = ScanCalculationFunctions.linear_rating2(ranges, max_range)

        # ensures that the front attraction does not become a back attraction in the next step
        flipped_front_attraction = front_attraction * -1
        vector = ScanCalculationFunctions.calculate_vector_normed(flipped_front_attraction, ranges,
                                                                  sensor_msgs_laserscan, angle_min)
        vector = ScanCalculationFunctions.flip_vector(vector)
        direction = ScanCalculationFunctions.create_normed_twist_message(
            vector,
            max_translational_velocity,
            max_rotational_velocity)

        # if NaN then create stop twist message
        if np.isnan(direction.linear.x):
            logging.get_logger('ScanCalculationFunctions').error(
                'calculated Twist message contains NaN value, '
                'adjusting to a stop message')
            direction = Twist()

        return direction, obstacle_free

    @staticmethod
    def combine_twist_messages(twist1, twist2, max_translational_velocity,
                               max_rotational_velocity):
        """Combine the given twist messages and norm the result."""
        temp1 = twist1.linear.x + twist2.linear.x
        temp2 = twist1.angular.z + twist2.angular.z

        vector = np.matrix([[temp1], [temp2]])

        result = ScanCalculationFunctions.create_normed_twist_message(vector,
                                                                      max_translational_velocity,
                                                                      max_rotational_velocity)
        return result

    @staticmethod
    def identify_objects(laser_scan: LaserScan, min_range: float, max_range: float, threshold: float):
        """Identifies objects from the scan. Returns a list of all identified objects,
        which are represented by a list of their rays as [range,angle].
        :param laser_scan: scan of the environment
        :param min_range: minimum length of a ray to consider it
        :param max_range: maximum length of a ray to consider it
        :param threshold: Length difference between two rays to map them to the same object
        :returns: A list of the objects, where each object is represented as a list of the lengths
        and angles of the rays.
        """

        # Adjust the ranges
        adj_ranges = ScanCalculationFunctions.adjust_ranges(laser_scan.ranges, min_range, max_range)

        # enrich matrix [distance, angle]
        ranges = np.asarray(adj_ranges)
        angle = (np.arange(ranges.size) * laser_scan.angle_increment) + laser_scan.angle_min
        ranges_to_angle = np.vstack((adj_ranges, angle)).T

        current_object = []
        object_list = []

        def is_object_ray(ray: float):
            return min_range < ray < max_range

        def within_threshold(obj: list, ray: float):
            """Returns True if the given ray is within the threshold boundary of the last ray of the object OR
            if the current object is empty."""
            return obj[-1][0] - threshold < ray < obj[-1][0] + threshold if obj else True

        def cleanup(li: List[list]):
            """Combine the last and first found object if they are considered as one."""
            if len(li) >= 2:
                first_ray = ranges_to_angle[0][0]
                first_ray_of_first_obj = li[0][0][0]
                if first_ray_of_first_obj == first_ray:
                    # first ray is contained in first object
                    last_ray = ranges_to_angle[-1][0]
                    last_ray_of_last_object = li[-1][-1][0]
                    if last_ray_of_last_object == last_ray:
                        # last ray is contained in last object
                        last_obj = li[-1]
                        if within_threshold(last_obj, first_ray_of_first_obj):
                            # as object are within threshold consider them as one
                            li[0].extend(li.pop())
            return li

        for i, e in enumerate(ranges_to_angle):
            if is_object_ray(e[0]):
                if within_threshold(current_object, e[0]):
                    current_object.append(e)
                else:
                    object_list.append(list(current_object))
                    current_object.clear()
                    current_object.append(e)
            else:
                if current_object:
                    object_list.append(list(current_object))
                    current_object.clear()
                    # TODO here add gap possibility between rays

        if current_object:
            object_list.append(list(current_object))
        return cleanup(object_list)

    @staticmethod
    def identify_robots(laser_scan: LaserScan, min_range: float, max_range: float, threshold: float, min_width: int,
                        max_width: int, reduction: ReductionOption = ReductionOption.NEAREST):
        """Identify robots from scan within given range. Objects are defined by rays that are beside each other and
        within the threshold of the last ray of the current object. An object is identified as robots if the object size
        is between min_width and max_width.
        :param laser_scan: scan of the environment
        :param min_range: minimum length of a ray to consider it
        :param max_range: maximum length of a ray to consider it
        :param threshold: Length difference between two rays to map them to the same robot
        :param min_width: minimum number of rays a robot is wide
        :param max_width: maximum number of rays a robot is wide
        :param reduction: Choose how the rays a robots consists of are reduced to represent the robot
        :return: return the list of all found robots and a list in which each robot is represented by a single ray
         based on the chosen reduction method
        """
        objects = ScanCalculationFunctions.identify_objects(laser_scan, min_range, max_range, threshold)
        # TODO allow the differ how wide a object is if it is near or far
        robots = ScanCalculationFunctions.select_objects(objects, min_width, max_width)

        if reduction is ReductionOption.NEAREST:
            robots_center = ScanCalculationFunctions.reduce_objects_to_nearest(robots)
        elif reduction is ReductionOption.MEAN:
            robots_center = ScanCalculationFunctions.reduce_objects_to_mean(robots)
        else:
            raise RuntimeError('Reduction behaviour not supported.')

        return robots, robots_center

    @staticmethod
    def select_objects(object_list: List[list], min_width: int, max_width: int):
        """Select objects with a width (the number of rays) between [min_width, max_width]
        :param object_list: A list containing the objects, which are lists of their rays
        :param min_width: minimum number of rays an object needs to be wide to be selected
        :param max_width: maximum number of rays an object needs to be wide to be selected
        :return: The selected objects which has the required width
        """
        matching = []
        for obj in object_list:
            if min_width <= len(obj) <= max_width:
                matching.append(obj)
        return matching

    @staticmethod
    def reduce_objects_to_nearest(object_list: List[list]):
        """
        For each given object in the list the list-of-rays object representation is reduced to the nearest ray.
        :param object_list: A list containing the objects, which are lists of their rays
        :return: list of objects only represented by their nearest ray
        """
        centered_objects = []
        distance_component = 0
        for obj in object_list:
            nearest = obj[0]
            for ray in obj:
                nearest = nearest if nearest[distance_component] <= ray[distance_component] else ray
            centered_objects.append(nearest)
        return centered_objects

    @staticmethod
    def reduce_objects_to_mean(object_list: List[list]):
        """For each given object in the list the list-of-rays object representation is reduced to a single ray pointing
         to the center of the object.
        :param object_list: A list containing the objects, which are lists of their rays
        :return: list of objects only represented by a single ray to their center.
        """
        centered_objects = []
        for obj in object_list:
            x = 0.0
            y = 0.0
            dis = 0.0
            for ray in obj:
                x += math.cos(ray[1])
                y += math.sin(ray[1])
                dis += ray[0]
            avg_radiant = math.atan2(y / len(obj), x / len(obj))
            avg_distance = dis / len(obj)
            centered_objects.append(np.array([avg_distance, avg_radiant]))

        return centered_objects

    @staticmethod
    def object_nearer_then(centered_object_list: list, distance: float):
        """
        Checks if one of the objects in the given list is nearer then the given distance value.
        :param centered_object_list: List of objects represented only by a single polar coordinate
        :param distance: The distance to compare with
        :return: True if the distance of an object is nearer then the distance value, False otherwise
        """
        for obj in centered_object_list:
            if obj[0] < distance:
                return True
        return False

        return direction

    @staticmethod
    def create_twist_from_objects(centered_object_list: list,
                                  max_translational_velocity: float,
                                  max_rotational_velocity: float,
                                  front_attraction: float,
                                  max_range: float,
                                  select_target: ReductionOption):
        """Create Twist message pointing away from the nearest object in the given list.
        Select objects by select_target parameter.
        :param centered_object_list: A list containing the objects, which are represented by a single ray
        :param max_rotational_velocity: The maximum rotational velocity of the robot
        :param max_translational_velocity: The maximum translational velocity of the robot
        :param front_attraction: The front attraction to add to the calculated vector
        :param max_range: The maximum range used to normalize the values
        :param select_target: Choose to which target the created Twist message points from NEAREST,
        MEAN (gravity center) and FARTHEST
        :return: Twist message away from the nearest object
        """
        return ScanCalculationFunctions.create_twist_for_objects(centered_object_list,
                                                                 max_translational_velocity,
                                                                 max_rotational_velocity, front_attraction,
                                                                 max_range,
                                                                 direction_pointing_from_object=True,
                                                                 select_target=select_target)

    @staticmethod
    def create_twist_towards_objects(centered_object_list: list,
                                     max_translational_velocity: float,
                                     max_rotational_velocity: float,
                                     front_attraction: float,
                                     max_range: float,
                                     select_target: ReductionOption):
        """Create Twist message pointing towards the selected object in the given list.
        Select objects by select_target parameter.
        :param centered_object_list: A list containing the objects, which are represented by a single ray
        :param max_rotational_velocity: The maximum rotational velocity of the robot
        :param max_translational_velocity: The maximum translational velocity of the robot
        :param front_attraction: The front attraction to add to the calculated vector
        :param max_range: The maximum range used to normalize the values
        :param select_target: Choose to which target the created Twist message points from NEAREST,
        MEAN (gravity center) and FARTHEST
        :return: Twist message towards the nearest object
        """
        return ScanCalculationFunctions.create_twist_for_objects(centered_object_list,
                                                                 max_translational_velocity,
                                                                 max_rotational_velocity, front_attraction,
                                                                 max_range,
                                                                 direction_pointing_from_object=False,
                                                                 select_target=select_target)

    @staticmethod
    def create_twist_for_objects(centered_object_list: list,
                                 max_translational_velocity: float,
                                 max_rotational_velocity: float,
                                 front_attraction: float,
                                 max_range: float,
                                 direction_pointing_from_object: bool,
                                 select_target: ReductionOption = ReductionOption.MEAN):
        """
        Create Twist message for the given objects in the given list.
        Pointing towards or away is set by the direction_pointing_to_object parameter.
        :param centered_object_list: A list containing the objects, which are represented by a single ray
        :param max_rotational_velocity: The maximum rotational velocity of the robot
        :param max_translational_velocity: The maximum translational velocity of the robot
        :param front_attraction: The front attraction to add to the calculated vector
        :param max_range: The maximum range used to normalize the values (expecting that longer values of they should
        not be considered are already removed from the given data)
        :param direction_pointing_from_object: If true the resulting Twist message points from the object
        :param select_target: Choose to which target the created Twist message points from NEAREST,
        MEAN (gravity center) and FARTHEST
        :return: Twist message towards or away from the nearest object depending on inn the direction_pointing_to_parameter
        """
        if select_target is ReductionOption.NEAREST:
            nearest_object = min(centered_object_list, key=lambda d: d[0])
            objects = [nearest_object]
        elif select_target is ReductionOption.MEAN:
            objects = centered_object_list
        elif select_target is ReductionOption.FARTHEST:
            farthest_object = max(centered_object_list, key=lambda d: d[0])
            objects = [farthest_object]
        else:
            raise RuntimeError('Reduction behaviour not supported.')

        cart_coord_objs = []
        for obj in objects:
            obj[0] = 1 - (obj[0] / max_range)
            cart_coord_objs.append(pol2cart(obj[0], obj[1]))
        vector = ScanCalculationFunctions.combine_vectors(cart_coord_objs)

        if direction_pointing_from_object:
            vector = ScanCalculationFunctions.flip_vector(vector)

        vector = ScanCalculationFunctions.add_attraction(vector, front_attraction)
        direction = ScanCalculationFunctions.create_normed_twist_message(vector,
                                                                         max_translational_velocity,
                                                                         max_rotational_velocity)

        return direction
