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
from rclpy.node import Node
from communication_interfaces.msg import Int8Message
from ros2swarm.utils.swarm_controll import SwarmState


class AbstractPattern(Node):
    """The abstract pattern base node."""

    def __init__(self, node_name):
        """Init the super node with the name of the node."""
        super().__init__(node_name)

        self.start_flag = False

        self.swarm_command_subscription = \
            self.create_subscription(Int8Message, '/swarm_command',
                                     self.swarm_command_callback, 10)

    def swarm_command_callback(self, msg: Int8Message):
        """
        Set the start flag to true if on the /swarm_command topic a SwarmState.START
        and to false if SwarmState.STOP is revised.If the flag is ture the callback is executed.

        ros2 topic pub --once /swarm_command communication_interfaces/msg/Int8Message "{data: 1}"

        ros2 topic pub --once /swarm_command communication_interfaces/msg/Int8Message "{data: 0}"
        """
        self.get_logger().debug('Robot "{}" received command "{}"'.format(self.get_namespace(), msg.data))
        if msg.data == int(SwarmState.START):
            self.start_flag = True
        if msg.data == int(SwarmState.STOP):
            self.start_flag = False

    def swarm_command_controlled(self, callback_func):
        """
        The callback function is only called if the start flag is set to true, which is done by publishing
        start message to the /swarm_command topic

        Only working for callbacks with **one** parameter
        """
        def callb(x):
            if self.start_flag:
                callback_func(x)
            else:
                self.swarm_command_false_case()
        return lambda x: callb(x)

    def swarm_command_controlled_timer(self, callback_func):
        """
        The callback function is only called if the start flag is set to true, which is done by publishing
        start message to the /swarm_command topic

        Only working for callbacks with **zero** parameters
        """
        def callb():
            if self.start_flag:
                callback_func()
            else:
                self.swarm_command_false_case()
        return lambda: callb()

    def swarm_command_false_case(self):
        """Defines the general behavior if the swarm command ist false."""
        pass

