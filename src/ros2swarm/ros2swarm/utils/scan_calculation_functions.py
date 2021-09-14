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
import math

from geometry_msgs.msg import Twist
from rclpy import logging

from sensor_msgs.msg import LaserScan


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
                    (e * math.cos(i * sensor_msgs_laserscan.angle_increment + sensor_msgs_laserscan.angle_min),
                     e * math.sin(i * sensor_msgs_laserscan.angle_increment + sensor_msgs_laserscan.angle_min))
                )
        return result_ranges

    @staticmethod
    def combine_vectors(vectors):
        """Combine the vectors to a single one by summing them up."""
        vector = numpy.matrix([[0.0], [0.0]])
        for i, e in enumerate(vectors):
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
        rot = numpy.matrix(
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
        vector = vector / numpy.linalg.norm(vector)
        direction.linear.x = float(max_translational_velocity * vector[0])
        direction.angular.z = float(
            max_rotational_velocity * math.asin(vector[1] / math.hypot(vector[0], vector[1])))
        return direction

    @staticmethod
    def potential_field(front_attraction, max_range, max_rotational_velocity,
                        max_translational_velocity,
                        min_range, sensor_msgs_laserscan, threshold):
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
                                                                  sensor_msgs_laserscan)
        direction = ScanCalculationFunctions.create_normed_twist_message(
            vector,
            max_translational_velocity,
            max_rotational_velocity)

        # if NaN then create stop twist message
        if numpy.isnan(direction.linear.x):
            logging.get_logger('ScanCalculationFunctions').error(
                'calculated Twist message contains NaN value, '
                'adjusting to a stop message')
            direction = Twist()

        return direction, obstacle_free

    @staticmethod
    def calculate_vector_normed(front_attraction, ranges, sensor_msgs_laserscan):
        """
        Calculate a vector from laser scan.

        Calculate a combined and rotated vector from the normed ranges and
        apply the given front attraction to it
        """
        vectors = ScanCalculationFunctions.calculate_vectors_from_normed_ranges(ranges, sensor_msgs_laserscan)
        vector = ScanCalculationFunctions.combine_vectors(vectors)
        vector = ScanCalculationFunctions.flip_vector(vector)
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
        """Return true if there is no range in the scan between min and max range."""
        adjusted_ranges = ScanCalculationFunctions.adjust_ranges(scan.ranges, min_range, max_range)
        return ScanCalculationFunctions.sum_ranges(max_range, adjusted_ranges)

    @staticmethod
    def attraction_field(front_attraction, max_range, max_rotational_velocity,
                         max_translational_velocity,
                         min_range, sensor_msgs_laserscan, threshold):
        """
        Create a potential field based on the given LaserScan message and the other parameters.

        The second return value is true if no obstacle within the max range is detected.
        threshold is the number of scans below the max_range which is needed to detect obstacles.
        The most nearest range is __most__ important.
        """
        ranges = ScanCalculationFunctions.adjust_ranges(sensor_msgs_laserscan.ranges, min_range,
                                                        max_range)
        obstacle_free = ScanCalculationFunctions.is_obstacle_free(max_range, ranges, threshold)

        ranges = ScanCalculationFunctions.linear_rating(ranges, max_range)

        vector = ScanCalculationFunctions.calculate_vector_normed(front_attraction, ranges,
                                                                  sensor_msgs_laserscan)
        vector = ScanCalculationFunctions.flip_vector(vector)
        direction = ScanCalculationFunctions.create_normed_twist_message(
            vector,
            max_translational_velocity,
            max_rotational_velocity)

        # if NaN then create stop twist message
        if numpy.isnan(direction.linear.x):
            logging.get_logger('ScanCalculationFunctions').error(
                'calculated Twist message contains NaN value, '
                'adjusting to a stop message')
            direction = Twist()

        return direction, obstacle_free

    @staticmethod
    def repulsion_field(front_attraction, max_range, max_rotational_velocity,
                        max_translational_velocity,
                        min_range, sensor_msgs_laserscan, threshold):
        """
        Create a potential field based on the given LaserScan message and the other parameters.

        The second return value is true if no obstacle within the max range is detected.
        threshold is the number of scans below the max_range which is needed to detect obstacles.
        The most nearest range is __least__ important.
        """
        ranges = ScanCalculationFunctions.adjust_ranges(sensor_msgs_laserscan.ranges, min_range,
                                                        max_range)
        obstacle_free = ScanCalculationFunctions.is_obstacle_free(max_range, ranges, threshold)

        ranges = ScanCalculationFunctions.linear_rating2(ranges, max_range)

        # ensures that the front attraction does not become a back attraction in the next step
        flipped_front_attraction = front_attraction * -1
        vector = ScanCalculationFunctions.calculate_vector_normed(flipped_front_attraction, ranges,
                                                                  sensor_msgs_laserscan)
        vector = ScanCalculationFunctions.flip_vector(vector)
        direction = ScanCalculationFunctions.create_normed_twist_message(
            vector,
            max_translational_velocity,
            max_rotational_velocity)

        # if NaN then create stop twist message
        if numpy.isnan(direction.linear.x):
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

        vector = numpy.matrix([[temp1], [temp2]])

        result = ScanCalculationFunctions.create_normed_twist_message(vector,
                                                                      max_translational_velocity,
                                                                      max_rotational_velocity)
        return result
