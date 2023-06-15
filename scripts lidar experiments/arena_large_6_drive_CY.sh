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


ROS_DOMAIN_ID=42 ros2 launch launch_gazebo create_enviroment.launch.py \
 gazebo_world:=arena_large.world \
 pattern:=drive_pattern \
 number_robots:=6 \
 total_robots:=6 \
 log_level:=info \
 robot:=waffle_pi \
 sensor_type:=lidar \
 x_start:=-0.0 \
 x_dist:=0.0 \
 y_start:=-2.5 \
 y_dist:=1.0 \
 driving_swarm:=False \
 logging:=True  \
 run_timeout:=60.0 \
 init_timeout:=0.0 \
 gui:=true &
 
 sleep 5
 
ROS_DOMAIN_ID=42 ros2 topic pub --once /swarm_command communication_interfaces/msg/Int8Message "{data: 1}"

 
# to add heterogeneous swarm / robots  
# bash add_robots_to_simulation.sh 
# total_robots: int - total number of robots when using add_robots to create heterogeneous swarm 
 
# gazebo_world arena_large.world | arena.world | empty.world | turtle.world | 560x540m.world | Ymaze.world | Ymaze_camber.world | Ymaze_camber_top.world

# pattern: pattern_name
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

# number_robots: num_robots
# log_level: info | DEBUG
# robot: burger | waffle_pi | jackal | thymio
# sensor_type: lidar | ir | ir_tf
# driving_swarm: true | false - use driving swarm framework by OVGU Magdeburg

