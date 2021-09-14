## **ROS2swarm Version 1.0.0**

ROS2swarm is a ROS 2 Dashing Diademata package. It provides a framework for launching and combining swarm behavior patterns. 
It is developed by the **Institute of Computer Engineering** with support from the Institute of Robotics and the Institute for Electrical Engineering in Medicine of the **University of Lübeck, Germany**. 

Project contributors: Tanja Katharina Kaiser, Marian Johannes Begemann, Tavia Plattenteich, Lars Schilling, Georg Schildbach, Heiko Hamann, Vincent Jansen, Daniel Tidde, Steffen Fleischmann  

### **Existing patterns**

The following table gives an overview of the current implemented behaviors. 
These are seperated into movement and voting patterns.
Every pattern can either be a basic pattern or a combined one, which make use of one or more other patterns.

| Pattern                  | Domain   | Type | 
| ------                   | ------   | ------  |
| drive                    | Movement | Basic    | 
| dispersion               | Movement | Basic    | 
| attraction               | Movement | Basic    | 
| minimalist flocking      | Movement | Basic    | 
| discussed dispersion | Movement | Combined | 
| random walk | Movement | Basic | 
| ------                   | ------   | ------   | 
| voter model              | Voting   | Basic    | 
| majority rule            | Voting   | Basic    | 

### **Supported Robot Platforms**
	
ROS2swarm supports the following robot platforms:
	
* Turtlebot3 Waffle Pi
* Turtlebot3 Burger
* Jackal

#### **Required Software**
* Ubuntu 18.04 LTS
* ROS 2 Dashing Diademata
* ROS 2 Turtlebot3 package 
* Python 3.7
* Gazebo 9 for simulation

#### **Contained Packages**
* ros2swarm
    * Pattern and pattern specific config files and launch scripts
* communication_interfaces
    * Interfaces for ROS messages used by the patterns
* launch_turtlebot_gazebo
    * scripts to run patterns in the gazebo simulator 

#### **Installation Guide** 
	
To see a full installation guide for the ROS2swarm package please see the INSTALL_GUIDE.md file.

#### **Run the existing patterns**
Launch the patterns with the launch scripts on the robot with **bringup_turtle.launch.py** or
in gazebo with **create_enviroment.launch.py**.
This launch scripts get called in the script **restart_turtle.sh** (on TurtleBot3 robot platforms) 
and **restart.sh** (gazebo). 
The scripts contain parameters to set the desired pattern, robot platform, ROS version and, for the simulation, the number of robots and the used gazebo world. 

All robots can be started at the same time after restart.sh was executed by publishing a start message to the robots. 
First, a new terminal has to be opened and setup.bash has to be sources 

```
source ./install/setup.bash
```

and afterwards the following line has to be executed:
```
 ros2 topic pub --once /swarm_command communication_interfaces/msg/Int8Message "{data: 1}"
```

Also needed is a dashing version of the turtlebot package - see Install Guide.

For the Jackal robot, you have to execute the following steps before running restart.sh: 

1. start a roscore 
2. run the rosbridge script (https://github.com/ros2/ros1_bridge)
3. start the jackal simulation (https://gitlab.iti.uni-luebeck.de/plattenteich/jackal-swarm-addition)
4. execute restart.sh 

#### **Add new patterns**
To add an new pattern remember to add it also here:
* setup.py
* provide an pattern.launch.py launch description
* add an .param file if needed 



