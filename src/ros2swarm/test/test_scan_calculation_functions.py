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
import unittest

# PKG = 'ros2swarm'
# NAME = 'dispersion_pattern_test'
import numpy as np
from geometry_msgs.msg import Twist

from sensor_msgs.msg import LaserScan

from ros2swarm.utils.scan_calculation_functions import ReductionOption
from ros2swarm.utils.scan_calculation_functions import ScanCalculationFunctions

SCF = ScanCalculationFunctions


class TestScanCalculationFunctions(unittest.TestCase):

    # adjust_ranges
    def test_adjust_ranges_reduce_to_max_value(self):
        ranges = []
        for x in range(0, 360):
            ranges.append(float('inf'))
        result_ranges = SCF.adjust_ranges(ranges, min_range=0.335, max_range=3.5)
        self.assertTrue(all(3.5 == ele for ele in result_ranges))

    def test_adjust_ranges_keep_max_value(self):
        ranges = []
        for x in range(0, 360):
            ranges.append(float(3.5))
        result_ranges = SCF.adjust_ranges(ranges, min_range=0.335, max_range=3.5)
        self.assertTrue(all(3.5 == ele for ele in result_ranges))

    def test_adjust_ranges_adjust_to_min_value(self):
        ranges = []
        for x in range(0, 360):
            ranges.append(0.2)
        result_ranges = SCF.adjust_ranges(ranges, min_range=0.335, max_range=3.5)
        print(result_ranges)
        self.assertTrue(all(0.0 == ele for ele in result_ranges))

    def test_adjust_ranges_no_change(self):
        ranges = []
        for x in range(0, 360):
            ranges.append(1.0)
        result_ranges = SCF.adjust_ranges(ranges, min_range=0.335, max_range=3.5)
        self.assertTrue(all(1.0 == ele for ele in result_ranges))

    # linear_rating
    def test_linear_rating_min_is_zero(self):
        ranges = SCF.linear_rating([3.5], 3.5)
        self.assertEqual(0, min(ranges))

    def test_linear_rating_max_is_one(self):
        ranges = SCF.linear_rating([0.0], 3.5)
        self.assertEqual(1.0, max(ranges))

    def test_linear_rating_half_range_is_0_5(self):
        ranges = SCF.linear_rating([3.5 / 2], 3.5)
        self.assertEqual(0.5, max(ranges))

    def test_linear_rating_calculation(self):
        ranges = SCF.linear_rating([1.0, 2.0, 3.0], 3.5)
        self.assertEqual(1 - 1 / 3.5, max(ranges))

    def test_linear_rating_one_ray_detects(self):
        ranges = [1.0]
        for x in range(0, 359):
            ranges.append(3.5)
        result_ranges = SCF.linear_rating(ranges, 3.5)
        self.assertEqual(1 - 1 / 3.5, sum(result_ranges))

    # linear_rating2
    def test_linear_rating2_max_is_one(self):
        ranges = SCF.linear_rating2([3.5], 3.5)
        self.assertEqual(1.0, min(ranges))

    def test_linear_rating2_min_is_zero(self):
        ranges = SCF.linear_rating2([0.0], 3.5)
        self.assertEqual(0.0, min(ranges))

    def test_linear_rating2_half_range_is_0_5(self):
        ranges = SCF.linear_rating2([3.5 / 2], 3.5)
        self.assertEqual(0.5, max(ranges))

    # calculate_vectors
    def test_calculate_vectors(self):
        ranges = [0.0, 0.5, 1.0, 0.99]
        laser_scan = LaserScan()
        laser_scan.angle_min = 0.0
        laser_scan.angle_increment = math.pi / 4
        result = SCF.calculate_vectors_from_normed_ranges(ranges, laser_scan)
        np.testing.assert_array_almost_equal([(0.0, 0.0),
                                              (0.353553, 0.353553),
                                              (-0.700036, 0.700036)],
                                             result)

    # calculate_vectors
    def test_calculate_vectors_lower_equals(self):
        ranges = [1.0]
        laser_scan = LaserScan()
        result = SCF.calculate_vectors_from_normed_ranges(ranges, laser_scan)
        self.assertEqual([], result)

    # combine_vectors
    def test_combine_vectors(self):
        vectors = [(0.1, 0.2), (0.3, 0.4)]
        result = SCF.combine_vectors(vectors)
        print(result)
        self.assertEqual(0.4, result[0])

    # add_attraction
    def test_add_attraction(self):
        vector = np.matrix(
            [
                [0.0],
                [0.0]
            ]
        )
        result = SCF.add_attraction(vector, 1.0)
        self.assertEqual(1.0, result[0])

    # is_obstacle_free
    def test_is_obstacle_free_no_obstacles(self):
        ranges = []
        for x in range(0, 360):
            ranges.append(3.5)
        result = SCF.is_obstacle_free(max_range=3.5, ranges=ranges, threshold=0)
        self.assertTrue(result)

    def test_is_obstacle_free_obstacle(self):
        ranges = []
        for x in range(0, 360):
            ranges.append(3.5)
        ranges[0] = 3.0
        result = SCF.is_obstacle_free(max_range=3.5, ranges=ranges, threshold=0)
        self.assertFalse(result)

    # sum_ranges
    def test_sum_ranges(self):
        result = SCF.sum_ranges(max_range=3.5, ranges=[0.0, 1.0, 4.0])
        self.assertEqual(1, result)

    # sum_adjusted_ranges
    def test_sum_adjusted_ranges(self):
        scan = LaserScan()
        scan.ranges = [0.0, 1.0, 4.0]
        result = SCF.sum_adjusted_ranges(max_range=3.5, min_range=0.12, scan=scan)
        self.assertEqual(1, result)

    # is_adjusted_obstacle
    def test_is_adjusted_obstacle_free(self):
        ranges = []
        for x in range(0, 360):
            ranges.append(3.5)
        # ranges[0] = 3.0
        scan = LaserScan()
        scan.ranges = ranges
        result = SCF.is_adjusted_obstacle_free(3.5, min_range=0.12, scan=scan, threshold=0)
        self.assertTrue(result)

    def test_is_adjusted_obstacle_free_detect_obstacle(self):
        ranges = []
        for x in range(0, 360):
            ranges.append(3.5)
        ranges[0] = 3.0
        scan = LaserScan()
        scan.ranges = ranges
        result = SCF.is_adjusted_obstacle_free(3.5, min_range=0.12, scan=scan, threshold=0)
        self.assertFalse(result)

    # adjusted_sum
    def test_adjusted_sum(self):
        scan = LaserScan()
        scan.ranges = [0.0, 1.0, 2.0, 4.0]
        result = SCF.adjusted_sum(max_range=3.5, min_range=0.12, scan=scan)
        self.assertEqual(2, result)

    # select_objects
    def test_select_objects_remove_too_small(self):
        # ray = [distance, radiant]
        ray1 = np.array([2.0, 1.0])
        ray2 = np.array([2.5, 1.1])
        ray3 = np.array([3.0, 1.2])

        obj1 = [ray1, ray2, ray3]
        obj2 = [ray1, ray2]
        obj_list = [obj1, obj2]
        result = SCF.select_objects(obj_list, 3, 5)
        self.assertEqual(1, len(result))
        self.assertEqual(obj1, result[0])

    # select_objects
    def test_select_objects_remove_too_wide(self):
        # ray = [distance, radiant]
        ray1 = np.array([2.0, 1.0])
        ray2 = np.array([2.5, 1.1])
        ray3 = np.array([3.0, 1.2])
        ray4 = np.array([3.0, 1.3])
        ray5 = np.array([3.0, 1.4])
        ray6 = np.array([3.0, 1.5])

        obj1 = [ray1, ray2, ray3]
        obj2 = [ray1, ray2, ray3, ray4, ray5, ray6]
        obj_list = [obj1, obj2]
        result = SCF.select_objects(obj_list, 3, 5)
        self.assertEqual(1, len(result))
        self.assertEqual(obj1, result[0])

    # select_objects
    def test_select_objects_keep_all(self):
        # ray = [distance, radiant]
        ray1 = np.array([2.0, 1.0])
        ray2 = np.array([2.5, 1.1])
        ray3 = np.array([3.0, 1.2])
        ray4 = np.array([3.0, 1.3])
        ray5 = np.array([3.0, 1.4])
        ray6 = np.array([3.0, 1.5])
        ray7 = np.array([3.0, 1.6])
        ray8 = np.array([3.0, 1.7])
        ray9 = np.array([3.0, 1.8])

        obj1 = [ray1, ray2, ray3]
        obj2 = [ray4, ray5, ray6]
        obj3 = [ray7, ray8, ray9]
        obj_list = [obj1, obj2, obj3]
        result = SCF.select_objects(obj_list, 3, 5)
        self.assertEqual(3, len(result))
        self.assertEqual(obj1, result[0])
        self.assertEqual(obj2, result[1])
        self.assertEqual(obj3, result[2])

    # reduce_objects_to_nearest
    def test_reduce_objects_to_nearest_same_number_of_objects(self):

        ray1 = np.array([2.0, 1.0])
        ray2 = np.array([2.5, 1.1])
        ray3 = np.array([3.0, 1.2])
        ray4 = np.array([2.0, 1.3])
        ray5 = np.array([2.0, 1.4])
        ray6 = np.array([2.0, 1.5])
        ray7 = np.array([3.0, 1.6])
        ray8 = np.array([3.0, 1.7])
        ray9 = np.array([1.0, 1.8])

        obj1 = [ray1, ray2, ray3]
        obj2 = [ray4, ray5, ray6]
        obj3 = [ray7, ray8, ray9]

        object_list = [obj1, obj2, obj3]
        result = SCF.reduce_objects_to_nearest(object_list)
        self.assertEqual(len(object_list), len(result))

    # reduce_objects_to_nearest
    def test_reduce_objects_to_nearest(self):

        ray1 = np.array([2.5, 1.0])
        ray2 = np.array([2.0, 1.1])
        ray3 = np.array([3.0, 1.2])
        ray4 = np.array([2.0, 1.3])
        ray5 = np.array([2.0, 1.4])
        ray6 = np.array([2.0, 1.5])
        ray7 = np.array([3.0, 1.6])
        ray8 = np.array([3.0, 1.7])
        ray9 = np.array([1.0, 1.8])

        obj1 = [ray1, ray2, ray3]
        obj2 = [ray4, ray5, ray6]
        obj3 = [ray7, ray8, ray9]

        object_list = [obj1, obj2, obj3]
        result = SCF.reduce_objects_to_nearest(object_list)

        np.testing.assert_array_equal([ray2, ray4, ray9], result)

    # reduce_objects_to_mean
    def test_reduce_objects_to_mean(self):

        ray1 = np.array([2.5, 1.0])
        ray2 = np.array([2.0, 1.1])
        ray3 = np.array([3.0, 1.2])
        ray4 = np.array([2.0, 1.3])
        ray5 = np.array([2.0, 1.4])
        ray6 = np.array([2.0, 1.5])
        ray7 = np.array([3.0, 1.6])
        ray8 = np.array([3.0, 1.7])
        ray9 = np.array([1.0, 1.8])

        obj1 = [ray1, ray2, ray3]
        obj2 = [ray4, ray5, ray6]
        obj3 = [ray7, ray8, ray9]

        object_list = [obj1, obj2, obj3]
        result = SCF.reduce_objects_to_mean(object_list)

        expected = [np.array([2.5, 1.1]), np.array([2.0, 1.4]), np.array([7 / 3, 1.7])]
        np.testing.assert_array_equal(expected, result)

    # identify_objects
    def test_identify_objects(self):

        laser_scan = LaserScan()
        laser_scan.angle_min = 1.0
        laser_scan.angle_increment = 0.1
        laser_scan.ranges = [2.0, 1.9, 2.1, 3.1, 3.2, 3.3, 1.4, 1.3, 1.2]
        result = SCF.identify_objects(laser_scan=laser_scan,
                                      min_range=0.12, max_range=3.5,
                                      threshold=0.2)

        np.testing.assert_array_almost_equal(
            [[np.array([2.0, 1.0]), np.array([1.9, 1.1]), np.array([2.1, 1.2])],
             [np.array([3.1, 1.3]), np.array([3.2, 1.4]), np.array([3.3, 1.5])],
             [np.array([1.4, 1.6]), np.array([1.3, 1.7]), np.array([1.2, 1.8])]],
            result)

    def test_identify_objects_cleanup(self):
        laser_scan = LaserScan()
        laser_scan.angle_min = 0.0
        laser_scan.angle_increment = math.pi / 180
        # create ranges
        ranges = np.append(np.append(np.full(shape=1, fill_value=2.0),
                                     np.full(shape=358, fill_value=3.5)),
                           np.full(shape=1, fill_value=2.0)).tolist()
        laser_scan.ranges = ranges
        result = SCF.identify_objects(laser_scan=laser_scan,
                                      min_range=0.12, max_range=3.5,
                                      threshold=0.2)
        self.assertEqual(1, len(result))
        self.assertEqual(2, len(result[0]))
        np.testing.assert_array_almost_equal([[np.array([2.0, 0.0]),
                                               np.array([2.0, laser_scan.angle_increment * 359])]],
                                             result)

    def test_identify_objects_within_threshold(self):
        laser_scan = LaserScan()
        laser_scan.angle_min = 0.0
        laser_scan.angle_increment = 0.1
        # create ranges
        laser_scan.ranges = [1.0, 1.1, 1.2,
                             2.0, 2.1, 1.9,
                             1.0, 1.19,
                             3.0]
        result = SCF.identify_objects(laser_scan=laser_scan,
                                      min_range=0.12,
                                      max_range=2.9,
                                      threshold=0.2)
        self.assertEqual(3, len(result))
        self.assertEqual(3, len(result[0]))
        self.assertEqual(3, len(result[1]))
        self.assertEqual(2, len(result[2]))

        np.testing.assert_array_almost_equal([np.array([1.0, 0.0]),
                                              np.array([1.1, 0.1]),
                                              np.array([1.2, 0.2])],
                                             result[0])
        np.testing.assert_array_almost_equal([np.array([2.0, 0.3]),
                                              np.array([2.1, 0.4]),
                                              np.array([1.9, 0.5])],
                                             result[1])
        np.testing.assert_array_almost_equal([np.array([1.0, 0.6]),
                                              np.array([1.19, 0.7])],
                                             result[2])

    # identify_robots
    def test_identify_robots_reduction_default(self):

        laser_scan = LaserScan()
        laser_scan.angle_min = 0.0
        laser_scan.angle_increment = 0.1
        laser_scan.ranges = [2.0, 1.9, 2.1, 3.1, 3.2, 3.3, 1.4, 1.3, 1.2]
        _, result = SCF.identify_robots(laser_scan=laser_scan,
                                        min_range=0.12, max_range=3.5,
                                        min_width=2, threshold=0.2,
                                        max_width=4)

        np.testing.assert_array_almost_equal([[1.9, 0.1], [3.1, 0.3], [1.2, 0.8]], result)

    # identify_robots
    def test_identify_robots_reduction_nearest(self):

        laser_scan = LaserScan()
        laser_scan.angle_min = 0.0
        laser_scan.angle_increment = 0.1
        laser_scan.ranges = [2.0, 1.9, 2.1, 3.1, 3.2, 3.3, 1.4, 1.3, 1.2]
        _, result = SCF.identify_robots(laser_scan=laser_scan,
                                        min_range=0.12, max_range=3.5,
                                        min_width=2, threshold=0.2,
                                        max_width=4,
                                        reduction=ReductionOption.NEAREST)

        np.testing.assert_array_almost_equal([[1.9, 0.1], [3.1, 0.3], [1.2, 0.8]], result)

    # reduce_identify_robots
    def test_identify_robots_reduction_mean(self):

        laser_scan = LaserScan()
        laser_scan.angle_min = 0.0
        laser_scan.angle_increment = 0.1
        laser_scan.ranges = [2.0, 1.9, 2.1, 3.1, 3.2, 3.3, 1.4, 1.3, 1.2]
        _, result = SCF.identify_robots(laser_scan=laser_scan,
                                        min_range=0.12, max_range=3.5,
                                        min_width=2, threshold=0.2,
                                        max_width=4,
                                        reduction=ReductionOption.MEAN)

        np.testing.assert_array_almost_equal([[2.0, 0.1], [3.2, 0.4], [1.3, 0.7]], result)

    def test_create_twist_for_nearest_object(self):
        robots_center = [[2.0, 0.1], [3.2, 0.4], [1.0, 0.7]]

        result = ScanCalculationFunctions.create_twist_for_objects(
            centered_object_list=robots_center,
            max_rotational_velocity=1.82,
            max_translational_velocity=0.26,
            max_range=3.5,
            front_attraction=0.0,
            direction_pointing_from_object=True)

        expected = Twist()
        expected.linear.x = -0.198858968693967
        expected.angular.z = -1.274
        self.assertEqual(result, expected)
