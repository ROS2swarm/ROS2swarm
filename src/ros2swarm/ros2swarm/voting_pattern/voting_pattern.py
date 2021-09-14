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
from ros2swarm.abstract_pattern import AbstractPattern
import re


class VotingPattern(AbstractPattern):
    """The base class for patterns which include voting."""

    def __init__(self, node_name):
        """Initialize the voting node."""
        super().__init__(node_name)

    def destroy_node(self):
        """Call the super destroy method."""
        super().destroy_node()

    def get_robot_id(self):
        # get robot id
        matching_pattern = r"(robot_namespace_)(\d+)"  # r means raw string
        p = re.compile(matching_pattern)
        m = p.search(str(self.get_namespace()))
        return int(m.group(2))
