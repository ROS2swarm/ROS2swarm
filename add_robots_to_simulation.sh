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
source ./install/setup.bash &&
 ROS_DOMAIN_ID=42 ros2 launch launch_gazebo add_robot.launch.py \
 start_index:=4 \
 gazebo_world:=arena_large.world \
 pattern:=random_walk_pattern \
 number_robots:=2 \
 log_level:=info \
 robot:=burger \
 sensor_type:=lidar \
 version:=2 \
 x_start:=1.0 \
 x_dist:=0.5 \
 y_start:=0.0 \
 y_dist:=1.0 \
 driving_swarm:=True 
 
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
# -r robot: burger | waffle_pi | jackal | thymio
