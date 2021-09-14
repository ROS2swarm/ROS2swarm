#!/bin/bash
#    Copyright 2021 Marian Begemann
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

colcon build --symlink-install && source ./install/setup.bash

for NUM in {1..100};
do
  echo -----------------------
  echo testbatch Starting $NUM
  echo -----------------------
  timeout -s SIGINT 64m ros2 launch launch_turtlebot_gazebo create_enviroment.launch.py -w Ymaze_camber_top_goal4.world -p rat_search_pattern -n 1 --log_level info
  echo -----------------------
  echo testbatch end $NUM
  echo -----------------------
  sleep 20
done
