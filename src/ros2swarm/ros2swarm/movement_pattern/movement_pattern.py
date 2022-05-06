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
from geometry_msgs.msg import Twist
from ros2swarm.abstract_pattern import AbstractPattern


class MovementPattern(AbstractPattern):
    """The base class for patterns which include movement."""

    def __init__(self, node_name):
        """
        Initialize the common movement pattern functions.

        Passes the node name to the super node and creates the drive command topic,
        which is available for all movement patterns
        """
        super().__init__(node_name)
        self.command_publisher = self.create_publisher(Twist,
                                                       self.get_namespace() + '/drive_command', 10)

    def swarm_command_false_case(self):
        """If swarm command is false the robot should not move."""
        self.command_publisher.publish(Twist())

    def destroy_node(self):
        """Send a stop twist message and calls the super destroy method."""
        self.command_publisher.publish(Twist())
        super().destroy_node()
