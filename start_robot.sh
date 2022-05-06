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
 ROS_DOMAIN_ID=42 ros2 launch ros2swarm bringup_robot.launch.py \
 pattern:=dispersion_pattern \
 log_level:=info \
 robot:=waffle_pi \
 robot_number:=NUM_CHANGE_ME
# pattern_name:
## movement pattern: drive_pattern | dispersion_pattern | aggregation_pattern | flocking_pattern | flocking_pattern2 | attraction_pattern | attraction_pattern2 | magnetometer_pattern | minimalist_flocking_pattern | discussed_dispersion_pattern
## voting_pattern voter_model_pattern | voter_model_with_limiter_pattern | majority_rule_pattern
# robot_nummer: num_robots
# robot: waffle_pi | burger | jackal
