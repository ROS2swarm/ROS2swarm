## **ROS2swarm Package Version 1.0.0**

ROS2swarm is a ROS 2 dashing package that provides an environment for launching and combining swarm behavior patterns. 
It is developed by the **Institute of Computer Engineering** with support from the Institute of Robotics and the Institute for Electrical Engineering in Medicine of the **University of Lübeck, Germany**. 

Project contributors: Tanja Katharina Kaiser, Marian Johannes Begemann, Tavia Plattenteich, Lars Schilling, Georg Schildbach, Heiko Hamann, Vincent Jansen, Daniel Tidde, Steffen Fleischmann  

### **Existing patterns**

The patterns are seperated into movement and voting patterns.
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



#### **Required Software**
* Python 3.7

#### **Contained Packages**
* ros2swarm
    * Pattern and pattern specific config files and launch scripts
* communication_interfaces
    * Interfaces needed by some of the patterns
* launch_turtlebot_gazebo
    * scripts to run patterns in the gazebo simulator 

#### **Run the existing patterns**
Launch the patterns with the launch scripts on the robot with **bringup_turtle.launch.py** or
in gazebo with **create_enviroment.launch.py**.
This launch scripts get called in the script **restart_turtle.sh** (on turtle) 
and **restart.sh** (gazebo). In this scripts the parameter are already set and 
you can switch between the launched patterns by changing -p parameter in the script.

To activate the patterns after startup run the following line:
```
 ros2 topic pub --once /swarm_command communication_interfaces/msg/Int8Message "{data: 1}"
```

Also needed is a dashing version of the turtlebot package - see Install Guide.

#### **Add new patterns**
To add an new pattern remember to add it also here:
* setup.py
* provide an pattern.launch.py launch description
* add an .param file if needed 

#### **Install Guide** 

see INSTALL_GUIDE.md file

