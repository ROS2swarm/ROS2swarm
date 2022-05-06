#!/usr/bin/env python3
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

import numpy

from geometry_msgs.msg import Twist
from ros2swarm.utils import setup_node
from sensor_msgs.msg import MagneticField
from rclpy.qos import qos_profile_sensor_data
from ros2swarm.movement_pattern.movement_pattern import MovementPattern


class MagnetometerPattern(MovementPattern):
    """
    Pattern to reach an orientation of the robot according to the given parameter.

    The magnetometer pattern node creates drive commands based on the magnetometer messages it
    receive

    pattern_node >> publishing to the self.get_namespace()/drive_command topic
    """

    def __init__(self):
        """Initialize the magnetometer pattern node."""
        super().__init__('magnetometer_pattern')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('magnetometer_target_direction_x', None),
                ('magnetometer_front_attraction', None),
                ('magnetometer_stop_when_target_direction_reached', None),
                ('magnetometer_allowed_delta_in_target_direction', None),
                ('magnetometer_rotational_velocity', None),
                ('max_translational_velocity', None),
                ('max_rotational_velocity', None),
                ('lidar_config', None)
            ])

        self.magnetometer_subscription = self.create_subscription(
            MagneticField,
            self.get_namespace() + '/magnetic_field',
            self.swarm_command_controlled(self.magnetic_callback),
            qos_profile=qos_profile_sensor_data
        )

        self.test_output = self.create_publisher(Twist, self.get_namespace() + '/test_output', 10)

        self.param_target_direction_x = float(
            self.get_parameter(
                "magnetometer_target_direction_x").get_parameter_value().double_value)
        self.param_front_attraction = float(
            self.get_parameter("magnetometer_front_attraction").get_parameter_value().double_value)
        self.param_stop_when_direction_reached = self.get_parameter(
            "magnetometer_stop_when_target_direction_reached").get_parameter_value().bool_value
        self.param_allowed_delta = float(
            self.get_parameter(
                "magnetometer_allowed_delta_in_target_direction")
            .get_parameter_value().double_value)
        self.param_rotational_velocity = float(
            self.get_parameter(
                "magnetometer_rotational_velocity").get_parameter_value().double_value)
        self.param_max_translational_velocity = self.get_parameter(
            "max_translational_velocity").get_parameter_value().double_value
        self.param_max_rotational_velocity = self.get_parameter(
            "max_rotational_velocity").get_parameter_value().double_value
        # TODO replace magic number '3'
        self.lidar_config = self.get_parameter(
            "lidar_config").get_parameter_value().double_value if self.get_parameter(
            "lidar_config").get_parameter_value().type == 3 else None

    def magnetic_callback(self, magnetometer_msg):
        """Call back if a new magnetometer msg is available."""
        # command to publish magnetometer message in terminal manually
        # ros2 topic pub --once /robot_namespace_0/magnetic_field sensor_msgs/msg/MagneticField
        # "{
        # header: {stamp: {sec: 1600091285, nanosec: 569285985}, frame_id: imu_link},
        # magnetic_field:
        # {x: -3.5999998999614036e-06, y: 0.0004931999719701707, z: -2.6399999114801176e-05},
        # magnetic_field_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        #  }"

        direction = self.vector_calc(magnetometer_msg)
        self.command_publisher.publish(direction)

    def vector_calc(self, incoming_msg):
        """Calculate the direction vector for the current message."""
        if incoming_msg is None:
            # or incoming_msg.magnetic_field.x = 0:
            return Twist()

        x = incoming_msg.magnetic_field.x
        z = incoming_msg.magnetic_field.z

        # TODO magnetic cancelation
        # rad2degree = 180 / math.pi
        # current_direction = abs(math.atan2(x, z)) * rad2degree
        # current_direction = math.degrees(math.atan2(x, z))
        # numpy.arctan2

        # fix to avoid negative values (nd jups in values
        current_direction = math.degrees(numpy.arctan2(x, z)) + 180.0
        # if current_direction < 0.0:
        #    current_direction = 180

        # current_direction = abs(math.degrees(numpy.arctan2(x, z)))
        # current_direction = math.degrees(numpy.arctan2(x, z)) + 180
        # current_direction = math.degrees(numpy.arctan2(x, z))

        diff = current_direction - self.param_target_direction_x

        direction = Twist()

        # TODO fix max_rotational_velocity is exceeded and 0.26 rad transactional is not reached
        # vector = numpy.matrix([[0.0], [0.0]])
        # vector[0] = 1
        # vector[1] = diff
        # vector = vector / numpy.linalg.norm(vector)
        # direction.linear.x = float(max_translational_velocity * 1)

        # if abs(diff) < self.allowed_delta:
        #    direction.angular.z = 0.0
        # else:
        # direction.angular.z = float(max_rotational_velocity * math.asin(diff
        # / math.hypot(0.26, diff)))
        # TODO reduce to max if max is exceeded
        # direction.linear.x = float(max_translational_velocity * vector[0])
        # direction.angular.z = float(max_rotational_velocity
        # * math.asin(vector[1] / math.hypot(vector[0], vector[1])))
        # direction.linear.x = float(self.param_max_translational_velocity * vector[0])
        # direction.angular.z = float(self.param_max_rotational_velocity
        # * math.asin(vector[1] / math.hypot(vector[0], vector[1])))

        if abs(self.param_front_attraction) > abs(self.param_max_translational_velocity):
            direction.linear.x = 0.0
        else:
            direction.linear.x = self.param_front_attraction

        if diff >= 0:
            direction.angular.z = self.param_rotational_velocity
        else:
            direction.angular.z = -self.param_rotational_velocity

        test = Twist()
        test.linear.x = current_direction
        test.linear.y = diff
        test.angular.x = direction.linear.x
        test.angular.y = direction.angular.z
        self.test_output.publish(test)

        if self.param_stop_when_direction_reached and abs(diff) <= self.param_allowed_delta:
            direction = Twist()

        return direction


def main(args=None):
    """Create a node for the magnetometer pattern, spins it and handles the destruction."""
    setup_node.init_and_spin(args, MagnetometerPattern)


if __name__ == '__main__':
    main()

# Example readouts from sensor:
# ---
# header:
#   stamp:
#     sec: 1600091285
#     nanosec: 569285985
#   frame_id: imu_link
# magnetic_field:
#   x: -3.5999998999614036e-06
#   y: 0.0004931999719701707
#   z: -2.6399999114801176e-05
# magnetic_field_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# ---
# header:
#   stamp:
#     sec: 1600091285
#     nanosec: 619173989
#   frame_id: imu_link
# magnetic_field:
#   x: -2.4000000848900527e-06
#   y: 0.000499200017657131
#   z: -2.9400000130408444e-05
# magnetic_field_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# ---
# header:
#   stamp:
#     sec: 1600091285
#     nanosec: 669291888
#   frame_id: imu_link
# magnetic_field:
#   x: 3.5999998999614036e-06
#   y: 0.0004914000164717436
#   z: -1.6199999663513154e-05
# magnetic_field_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# ---
# header:
#   stamp:
#     sec: 1600091285
#     nanosec: 719295621
#   frame_id: imu_link
# magnetic_field:
#   x: 1.7999999499807018e-06
#   y: 0.0004949999856762588
#   z: -1.9799999790848233e-05
