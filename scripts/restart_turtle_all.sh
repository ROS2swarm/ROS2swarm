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

for NUM in 1 2 3 4 5 6 7
do
  gnome-terminal -e 'ssh ubuntu@turtle$NUM.local "cd ./ROS2swarm && source /opt/ros/foxy/setup.bash && export ROS_DOMAIN_ID=30 && export TURTLEBOT3_MODEL=waffle_pi && source ~/turtlebot3_ws/install/setup.bash && colcon build --symlink-install && source ./install/setup.bash && ros2 launch ros2swarm bringup_turtle.launch.py -p dispersion_pattern -n $NUM" --log_level info -r waffle_pi -v 2 && read -p "wait"'
done
# -p pattern_name
## movement pattern: drive_pattern | dispersion_pattern | aggregation_pattern | flocking_pattern | attraction_pattern | magnetometer_pattern
## voting_pattern voter_model_pattern | voter_model_with_limiter_pattern | majority_rule_pattern
# -n num_robots
#-r robot waffle_pi | burger
 #&& read -p "Press any key to stop..." && killall ros2
