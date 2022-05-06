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
#    limitations under the License.colcon build --symlink-install &&
colcon build --symlink-install &&
 source ./install/setup.bash &&
 ros2 launch ros2swarm bringup_turtle.launch.py -p discussed_dispersion_pattern -n NUM_CHANGE_ME --log_level info -r waffle_pi -v 2
# -p pattern_name
## movement pattern: drive_pattern | dispersion_pattern | aggregation_pattern | flocking_pattern | flocking_pattern2 | attraction_pattern | attraction_pattern2 | magnetometer_pattern | minimalist_flocking_pattern | discussed_dispersion_pattern
## voting_pattern voter_model_pattern | voter_model_with_limiter_pattern | majority_rule_pattern
# -n num_robots
# -r robot waffle_pi | burger
