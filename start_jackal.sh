colcon build --symlink-install &&
 source ./install/setup.bash &&
ros2 launch launch_turtlebot_gazebo jackal_environment.launch.py -w arena_large.world -p attraction_pattern -n 1 __log_level:=debug -r burger