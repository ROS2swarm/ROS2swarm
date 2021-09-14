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

import unittest
import numpy

# PKG = 'ros2swarm'
# NAME = 'dispersion_pattern_test'
from sensor_msgs.msg import LaserScan

from src.ros2swarm.ros2swarm.utils.scan_calculation_functions import \
    ScanCalculationFunctions

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
        ranges = [0.0, 0.5, 1.0]
        result = SCF.calculate_vectors_from_normed_ranges(ranges)
        self.assertEqual([(0.0, 0.0), (0.49992384757819563, 0.008726203218641756)], result)

    # calculate_vectors
    def test_calculate_vectors_lower_equals(self):
        ranges = [1.0]
        result = SCF.calculate_vectors_from_normed_ranges(ranges)
        self.assertEqual([], result)

    # combine_vectors
    def test_combine_vectors(self):
        vectors = [(0.1, 0.2), (0.3, 0.4)]
        result = SCF.combine_vectors(vectors)
        print(result)
        self.assertEqual(0.4, result[0])

    # add_attraction
    def test_add_attraction(self):
        vector = numpy.matrix(
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
