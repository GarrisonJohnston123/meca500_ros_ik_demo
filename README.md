# meca500_ros_ik_demo
This repository contains a demonstration of solving the inverse kinematics of the Mecademic Meca500 using PyKDL and visualizing the robot in RVIZ. This repository was created as part of a ROS lecture for Intro to Robotics at Vanderbilt University.

## System Information
This code has been tested using [http://wiki.ros.org/melodic ROS Melodic] running on [https://releases.ubuntu.com/18.04/ Ubuntu 18.04].

## Branches
In this repository, the **main** branch has the ROS portions of the code missing for pedagogical reasons. The **solution** branch contains the full version of the code.    

## Installation Instructions
1. Install dependices:
```
sudo apt install ros-melodic-joint-state-publisher-gui
sudo apt-get install ros-melodic-rqt-ez-publisher
rm ~/.config/ros.org/rqt_gui.ini
```
2. Create a catkin workspace:
```
mkdir -p ~/catkin_ws/src
```
3. Download code: 
```
cd ~/catkin_ws/src
git clone https://github.com/GarrisonJohnston123/meca500_ros_ik_demo.git
git clone https://github.com/Mecademic/ROS.git
```
If you want to download the **solution** branch, replace the second line in the above block with:
```
git clone https://github.com/GarrisonJohnston123/meca500_ros_ik_demo.git -b solution
```

4. build code:
```
cd ..
catkin_make
```

## Running the Code
Run the following commands
```
cd ~/catkin_ws
source devel/setup.bash
roslaunch meca_ik_demo meca_ik_demo.launch
```
Then, in *rqt_ez_publisher* select */joint_pos_desired*. You can then use the sliders to set the desired pose of the robot and see the result in RVIZ.




