#!/bin/bash
. /opt/ros/melodic/setup.bash
. /opt/ros/dashing/setup.bash
export ROS_MASTER_URI=http://localhost:11311
#. ~/ros1_bridge-dashing/install/setup.bash  #if build from source
ros2 run ros1_bridge dynamic_bridge 
 
