## **Manual ROS2swarm Version 1.1.0**

ROS2swarm is a ROS 2 dashing package that provides swarm behavior patterns. 
It is developed by the **Institute of Computer Engineering** with support from the Institute of Robotics and the Institute for Electrical Engineering in Medicine of the **University of Lübeck, Germany**. 

Project contributors: Tanja Katharina Kaiser, Marian Johannes Begemann, Tavia Plattenteich, Lars Schilling, Georg Schildbach, Heiko Hamann, Vincent Jansen, Daniel Tidde, Steffen Fleischmann  

The current ROS2swarm version is 1.1.0. 
The ICRA 2022 paper "ROS2swarm - A ROS 2 Package for Swarm Robot Behaviors" refers to version 1.0.0 (Dashing), which can be found [here](https://gitlab.iti.uni-luebeck.de/ROS2/ros2swarm/-/tags/ICRA22). 

### **Existing patterns**

The following table gives an overview the current implemented patterns.
The patterns are separated into movement and voting patterns.
Every pattern can either be a basic pattern or a combined one, which make use of one or more other patterns to create more complex behaviors.


| Pattern                  | Domain   | Type     | Simulation          | Robot  |
| ------                   | ------   | ------   | ------              | ------ |
| drive                    | Movement | Basic    | :heavy_check_mark:  | :heavy_check_mark: |
| dispersion               | Movement | Basic    | :heavy_check_mark:  | :heavy_check_mark: |
| attraction               | Movement | Basic    | :heavy_check_mark:  | :heavy_check_mark: |
| magnetometer             | Movement | Basic    | :x:                 | :heavy_check_mark: |
| minimalist flocking      | Movement | Basic    | :heavy_check_mark:  | :heavy_check_mark: |
| random walk      | Movement | Basic    | :heavy_check_mark:  | :heavy_check_mark: |
| discussed dispersion pattern                 | Movement | Combined | :heavy_check_mark:  | :heavy_check_mark: |
| voter model              | Voting   | Basic    | :heavy_check_mark:  | :heavy_check_mark: |
| majority rule            | Voting   | Basic    | :heavy_check_mark:  | :heavy_check_mark: |

In addition, a hardware protection layer is started to prevent collisions.

### **Supported Robot Platforms**
ROS2swarm supports the following robot platforms:
* TurtleBot3 Waffle Pi
* TurtleBot3 Burger
* Jackal UGV

### **Required Software**
* Ubuntu 18.04 LTS
* ROS 2 Dashing Diademata
* ROS 2 Turtlebot3 package 
* Python 3.6
* Gazbeo 9 for simulation

#### **Contained Packages**
* ros2swarm
    * The main package containing the behavior patterns and their configuration and launch files.
* communication_interfaces
    * Interfaces for special ROS messages used by the patterns
* launch_turtlebot_gazebo
    * Scripts to start the Gazebo simulation 

#### **Installation Guide** 

To see a full installation guide for the ROS2swarm package please see the INSTALL_GUIDE.md file.

#### **Run the existing patterns**

Launch the patterns with the launch scripts on the robot with **bringup_turtle.launch.py** or
in Gazebo with **create_enviroment.launch.py**.
This launch scripts get called in the script **restart_turtle.sh** (on turtle) 
and **restart.sh** (Gazebo). T
The scripts contain parameters to set the desired pattern, robot platform, ROS version and, for the simulation, the number of robots and the used Gazebo world.

All robots can be started at the same time after restart.sh was executed by publishing a start message to the robots.
First, a new terminal has to be opened and setup.bash has to be sourced

```
source ./install/setup.bash
```

and afterwards the following line has to be executed

```
 ros2 topic pub --once /swarm_command communication_interfaces/msg/Int8Message "{data: 1}"
```

#### **Adding new patterns**
To add a new pattern remember to add it also here:
* setup.py
* provide a pattern.launch.py launch description
* add a .param file if needed 

#### **Using the modified TurtleBot3 models**
We added meshes with modifications of the Turtlebot3 models. To use them they needed to be 
copied to the workspace of the turtlebot3 package as described in the following.

To select the waffle_pi robot use "waffle_pi" in the start scripts.
To select the a robot modifed from the waffle_pi use "waffle_pi_name_of_modification" in the start scripts.
The configuration of the waffle_pi is used as the name is shorted in the launch script where needed. 
The mesh for Gazebo is automatically selected by the use of our launch scripts.
The TURTLEBOT3_MODEL environment variable is no longer used in the scripts.

The same applys for the "burger" model.
```
export "TURTLEBOT3_MODEL="waffle_pi_invisible_sensors""
```

###### LED:
1) copy the WaffelPi_Model_Extension/turtlebot3_waffle_pi_led directory to the model directory of the turtlebot3_ws 
(turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models)
2) build turtlebot3_ws
```
cd ~/turtlebot3_ws && colcon build --symlink-install
```
3) use robot selection parameter in restart.sh 
```
-r waffle_pi_led
```

###### Turtlebot3 with no sensor visualisation in Gazebo
For burger version replace "waffle_pi" with "burger" in the following.
1) copy
```
WaffelPi_Model_Extension/turtlebot3_waffle_pi_invisible_sensors
```
to
```
turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models
```
2) copy
```
WaffelPi_Model_Extension/turtlebot3_waffle_pi_invisible_sensors.urdf
```
to
```
turtlebot3_ws/src/turtlebot3/turtlebot3_description
```
3) build turtlebot3_ws
```
cd ~/turtlebot3_ws && colcon build --symlink-install
```
4) use robot selection parameter in restart.sh 
```
-r waffle_pi_invisible_sensors
```
5)
```
source ~/turtlebot3_ws/install/setup.bash
```
