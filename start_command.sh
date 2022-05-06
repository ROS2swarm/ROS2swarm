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
ROS_DOMAIN_ID=42 ros2 topic pub --once /swarm_command communication_interfaces/msg/Int8Message "{data: 1}"
ROS_DOMAIN_ID=42 ros2 topic pub --once /swarm_command communication_interfaces/msg/Int8Message "{data: 1}"
ROS_DOMAIN_ID=42 ros2 topic pub --once /swarm_command communication_interfaces/msg/Int8Message "{data: 1}"
ROS_DOMAIN_ID=42 ros2 topic pub --once /swarm_command communication_interfaces/msg/Int8Message "{data: 1}"
ROS_DOMAIN_ID=42 ros2 topic pub --once /swarm_command communication_interfaces/msg/Int8Message "{data: 1}"

