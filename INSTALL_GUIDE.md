# Install guide ROS2swarm

This guide is a detailed step by step instruction to install the ROS2swarm package on top of a Ubuntu 18.04 OS.
It is based on the guide to step up a turtlebot3 development environment and uses the manual install of
- https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/
- https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/


## Prerequirements

This guide expects that the OS Ubuntu Bionic 18.04 is already installed.

<!--
In order to work with the test python 3.6 is used now.
TODO: check if no functionality requires python 3.7

### Install python 3.7 and set it as default

```
python --version
sudo apt-get update
sudo apt-get install python3.7
sudo update-alternatives --install /usr/bin/python python /usr/bin/python2.7 1
sudo update-alternatives --install /usr/bin/python python /usr/bin/python3.7 2
sudo update-alternatives --config python
python --version
```

-->

### Install ROS2 Dashing
Install ROS2 desktop version following: https://docs.ros.org/en/dashing/Installation/Ubuntu-Install-Debians.html

Set locale
```
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```
Setup Sources
```
sudo apt update && sudo apt install curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
Install ROS 2 packages
```
sudo apt update
sudo apt install ros-dashing-desktop
echo 'source /opt/ros/dashing/setup.bash' >> ~/.bashrc
```
Install argcomplete (optional)
```
sudo apt install -y python3-pip
pip3 install -U argcomplete
```
Test if installation were successful
new Terminal
```
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_py listener
```
### Install Dependent ROS 2 Packages, including gazebo9

Install colcon
```
sudo apt install python3-colcon-common-extensions
```

Install Gazebo9
```
curl -sSL http://get.gazebosim.org | sh
```

Uninstall Gazebo11 if installed previously
```
sudo apt remove gazebo11 libgazebo11-dev
sudo apt install gazebo9 libgazebo9-dev
sudo apt install ros-dashing-gazebo-ros-pkgs
```

Install Cartographer
```
sudo apt install ros-dashing-cartographer
sudo apt install ros-dashing-cartographer-ros
```

Install Navigation2
```
sudo apt install ros-dashing-navigation2
sudo apt install ros-dashing-nav2-bringup
```

Install vcstool
```
sudo apt install python3-vcstool
```

### Install turtlebot3

The ROS2swarm package is supported for the following commit states of the turtlebot3 packages, therefore a this commits needs to be checkout from the downloaded git repositories.

TurtleBot3 packages with source code:
```
sudo apt remove ros-dashing-turtlebot3-msgs
sudo apt remove ros-dashing-turtlebot3
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src/
git clone -b dashing-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b dashing-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
sudo apt install ros-dashing-dynamixel-sdk
cd ~/turtlebot3_ws && colcon build --symlink-install
```

Environment Configuration
```
echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
```

Install turtlebot3_simulation package
```
cd ~/turtlebot3_ws/src/
git clone -b dashing-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
```

### switch to supported commits
turtlebot3:
```
cd  ~/turtlebot3_ws/src/turtlebot3
git checkout 1ff16b4
```
turtlebot3_msgs:
```
cd ~/turtlebot3_ws/src/turtlebot3_msgs
git checkout 348b3260
```
turtlebot3_simulations:
```
cd ~/turtlebot3_ws/src/turtlebot3_simulations
git checkout 9ecdd65e6
```

Set the gazebo model path
```
echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models' >> ~/.bashrc
```

build turtlebot3 packages
```
cd ~/turtlebot3_ws && colcon build --symlink-install
echo 'export TURTLEBOT3_MODEL=waffle_pi' >> ~/.bashrc
source ~/.bashrc
```

Test if example simulation works
```
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

## Install ROS2swarm package

place the project folder in your home directory (also required for using the scripts)
```
git clone git@gitlab.iti.uni-luebeck.de:kaiser/ROS2swarm.git
cd ~/ROS2swarm
colcon build --symlink-install
echo 'source ~/ROS2swarm/install/setup.bash' >> ~/.bashrc
```

Test if ROS2swarm package starts a simulation
```
source ~/.bashrc
bash ~/ROS2swarm/restart.sh
```

## Additional installation based on template


install PyCharm as IDE
```
sudo snap install pycharm-educational --classic
```
introduce ROS to PyCharm
```
sudo nano /var/lib/snapd/desktop/applications/pycharm-educational_pycharm-educational.desktop
```
replace /snap/bin/pycharm-educational with commands in Exec line
```
bash -i -c "/snap/bin/pycharm-educational"
```
or if that turns out not to work revert changes and start IDE from console with sourced ROS2 using
```
snap run pycharm-educational
```

choose python3.7 as default (only in older template required)
```
sudo update-alternatives --install /usr/bin/python python /usr/bin/python3.7 3
```

install pip
```
sudo apt install python-pip
```

install upgrade pip
```
pip install --upgrade pip
```

install open_cv
```
pip install opencv-python
```


