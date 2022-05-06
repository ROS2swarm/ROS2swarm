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
echo delete old package on VM &&
# on VM
echo delete old package &&
rm ./../../ROS2swarm.tar.gz
echo create new package &&
tar -czf ./../../ROS2swarm.tar.gz ../../ROS2swarm &&
for NUM in 1 2 3 4 5 6 7
do
  # Bot
  echo delete old package on bot and remove old bot code &&
  ssh ubuntu@turtle$NUM.local 'rm ./ROS2swarm.tar.gz && rm -rf ./ROS2swarm'
  echo copy package to bot &&
  scp ./../../ROS2swarm.tar.gz ubuntu@turtle$NUM.local:~/ &&
  echo unpack package &&
  ssh ubuntu@turtle$NUM.local "tar -xzf ./ROS2swarm.tar.gz --warning=no-timestamp &&
   echo update robot number &&
   sed -i 's/robot_namespace_NUM_CHANGE_ME/robot_namespace_$NUM/g' ./ROS2swarm/src/ros2swarm/param/waffle_pi.yaml &&
   sed -i 's/robot_namespace_NUM_CHANGE_ME/robot_namespace_$NUM/g' ./ROS2swarm/src/ros2swarm/param/burger.yaml &&
   sed -i 's/NUM_CHANGE_ME/$NUM/g' ./ROS2swarm/start_robot.sh &&
    echo update robot number in start_robot script &&
     sed -i 's/NUM_CHANGE_ME/$NUM/g' ./ROS2swarm/start_robot.sh &&
      echo remove old /build /install /log directory &&
       rm -rf ./ROS2swarm/build ./ROS2swarm/install ./ROS2swarm/log &&
        echo source ros2 && source /opt/ros/foxy/setup.bash &&
         echo rebuild && cd ./ROS2swarm &&
          colcon build && source ./install/setup.bash" &&
  echo done with $NUM
done
