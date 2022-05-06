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
  ssh pi@ros2turtle$NUM.local "bash ./ROS2swarm/restart_turtle.sh"
done
# -p pattern_name
## movement pattern: drive_pattern | dispersion_pattern | aggregation_pattern | flocking_pattern | attraction_pattern | magnetometer_pattern
## voting_pattern voter_model_pattern | voter_model_with_limiter_pattern| majority_rule_pattern
# -n num_robots
 #&& read -p "Press any key to stop..." && killall ros2