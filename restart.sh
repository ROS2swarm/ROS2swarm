#!/bin/bash
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
colcon build --symlink-install &&
 source ./install/setup.bash &&
 ros2 launch launch_turtlebot_gazebo create_enviroment.launch.py -w arena_large.world -p aggregation_pattern -n 9 --log_level info -r waffle_pi -v 2
# -w arena_large.world | arena.world | empty.world | turtle.world | 560x540m.world | Ymaze.world | Ymaze_camber.world | Ymaze_camber_top.world
# -p pattern_name
## movement pattern:
#  * drive_pattern
#  * dispersion_pattern
#  * aggregation_pattern
#  * attraction_pattern
#  * attraction_pattern2
#  * magnetometer_pattern
#  * minimalist_flocking_pattern
#  * rat_search_pattern
#  * flocking_pattern
#  * flocking_pattern2
#  * random_walk_pattern
#  * discussed_dispersion_pattern
## voting_pattern:
#  * voter_model_pattern
#  * voter_model_with_limiter_pattern
#  * majority_rule_pattern
#
# -n num_robots
# -r robot: burger | waffle_pi | jackal
# -v ros_version: 1 | 2
