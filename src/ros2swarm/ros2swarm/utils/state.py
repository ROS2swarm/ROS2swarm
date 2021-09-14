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

from enum import Enum, unique


@unique
class State(Enum):
    """Defines states for state machine."""

    INIT = 0
    EXPLORE = 1
    JOIN_GROUP = 2
    STAY_IN_GROUP = 3
    LEAVE_GROUP = 4
    DISPERSION = 5
    STAY_ON_ROUTE = 6
    ATTRACTION = 7
    AVOID = 8 
    ATTRACTION_BACK = 9
    TUNNEL = 10
    CROSSING = 11
    CROSSING_LEFT = 12
    CROSSING_RIGHT = 13
    ENDING = 14
    START_CHAMBER = 15
    SEARCH_WALL = 16
    TUNNEL_CORNER = 17
