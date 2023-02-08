# **Control of gopigo3 in RaspberryPi**

In a hospital, a delivery robot carries samples or food from one room to another.

The main objectives are:

- Assemble a real robot (gopigo3)
- Control the gopigo3 robot movement
- use SLAM (Simultaneous Localization and Mapping) techniques to generate and store a map of the room
- use Navigation ROS package to find an optimal trajectory to reach a speciffic target position
- Define a route to be followed by the gopigo3 robot using Navigation package

let's see how to fulfill these objectives

## **1. Gopigo3 robot prototype setup**

This tobot prototype is based on:

- On-board computer based on raspberrypi3 board
- 2 DC-motor with encoder for differential drive controller
- RaspiCAM RGB camera
- LIDAR sensor

![Getting Started](./Images/1_gopigo3_UB.png)

The raspberrypi3 onboard is preinstalled with:

- raspbian OS
- ROS source installation with rviz
- a "master_ws_original_copy" repository. You have NOT to use this repository. This could be used only in case you need to restart the repository with the original settings
- a "gopigo3_rbpi3_ws" repository. This will be used by the students to perform the project with gopigo3 robot. This folder will be placed in the raspberrypi3 Desktop.

This repository is located in /home/pi/Desktop folder and it is already compiled.

### **Raspberrypi configuration and setup**

The raspberrypi4 is configured:

- to generate a hotspot
- LIDAR activated
- raspicam activated

When powering the raspberrypi4, generates a hotspot you have to connect to:

- SSID name: rubot_10
- password "CorrePiCorre"

Once you are connected to this network you will be able to connect your computer to the raspberrypi4 using Nomachine Display:

- download and install the Nomachine for windows at:
- Select the raspberrypi IP address: 10.42.0.1
- you have to specify:
  - user: pi
  - password: ubuntu0ubuntu1
- You will have the raspberrypi desktop on your windows nomachine screen

![](./Images/2_vnc1.png)

The first time you have to clone the "rUBot_gopigo_ws" repository to the home folder.

```shell
cd /home
git clone https://github.com/manelpuig/rUBot_gopigo_ws
cd rUBot_gopigo_ws
catkin_make
```

> Carefull!: Some actions have to be done:

- review the ~/.bashrc: source to the ws and delete the environment variables
- make executable the c++ and python files

### **1. gopigo bringup**

To properly perform a especific movement control we have first to install some HW packages:

> Information is in: https://forum.dexterindustries.com/t/setup-python3-gopigo3-and-di-sensors-on-ubuntu-server-20-04-64-bit-for-non-root-access/8305

#### **1.1 gopigo3_node**

This node is responsible to drive the wheels and obtain the odometry

You have to clone (or copy) the repository in src folder

```shell
cd src
git clone https://github.com/ros-gopigo/gopigo3_node
```

#### **1.2 LIDAR**

This node is responsible to see the obstacle distances 360º arround. We have 2 different LIDAR models (RPLIDAR and YDLIDAR) and we will install the 2 packages

a) RPLIDAR

This can be installed with apt (in opt/ros/noetic/share folder)

```shell
sudo apt install ros-noetic-rplidar-ros
```

b) YDLIDAR

This can be installed clonning (or copying) the package in src folder:

```shell
cd src
git clone https://github.com/YDLIDAR/ydlidar_ros_driver
```

Follow instructions in readme to install YDlidar SDK. This is already made in raspberrypi4

#### **1.3. RASPICAM**

This node is responsible to view images from camera

You can install package following instructions in:https://github.com/UbiquityRobotics/raspicam_node

```shell
cd src
git clone https://github.com/UbiquityRobotics/raspicam_node
```

#### **1.4. OPEN CV**

This package is used for image processing with OpenCV and ROS

This can be installed with apt (in opt/ros/noetic/share folder)

```shell
sudo apt install ros-noetic-vision-opencv
```

#### **1.5. Teleop-tools**

This package is used to control robot with keyboard or joypad

This can be installed with apt (in opt/ros/noetic/share folder)

```shell
sudo apt install ros-noetic-teleop-tools
```

Once all drivers are installed, you can compile the ws and proceed for the bringup

#### **1.6. Bringup**

Now you can bringup our robot:

- launch the gopigo3 node: able to control de 2 motors and measure the odometry
- launch the raspicam node
- launch the LIDAR sensor node

This is done in the

![Getting Started](./Images/2_nodes_cam.png)

A launch file is made to automatically make the bringup hardware:

```shell
roslaunch gopigo3_control gopigo3_bringup_hw.launch
```

> Specify in launch file if you have YDlidar or RPlidar

## **2. gopigo3 first control movements**

First, let's control the gopigo3 movement to perform:

- movement control using keyboard
- movement control with specific python script
- autonomous navigation
- autonomous navigation following right or left wall
- Navigation to speciffic POSE

We have created a specific package "gopigo_control" where all these controls are programed.
You can review from the "gopigo3_rbpi3_ws" workspace the src/gopigo_control folder where there are 2 new folders:

- scrip folder: with the python programs for specific movement control
- launch folder: with programs to launch the movement control

![](./Images/2_vnc2.png)

### **2.1. Keyboard movement control**

To control the gopigo robot with keyboard, we need to install "teleop_tools" package. This is already installed in our master_ws as you can see in the previous figure.

Open a  terminal and type:

```shell
roslaunch gopigo3_control gopigo3_bringup_hw.launch
```

Open a new terminal and type:

```shell
rosrun key_teleop key_teleop.py /key_vel:=/cmd_vel
```

Carefull: if there are problems, make a source in each terminal

![Getting Started](./Images/2_key.png)

Open a new terminal and see all the nodes and topics involved:

```shell
rqt_graph
```

Pres q and crtl+C to close the terminals

### **2.2. Movement control with specific python script**

You want to perform the movement control in "move3_gopigo_distance.py" simulation ws.

You will only need to:

- copy this python file to the script folder
- create a new "move_gopigo3.launch" file to launch gopigo3 and the control node

> Carefull!: be sure the new python file is executable.

Type:

```shell
roslaunch gopigo3_control gopigo3_bringup_hw.launch
roslaunch gopigo3_control node_nav.launch
```

### **2.3. Autonomous navigation**

For autonomous navigation you need the LIDAR sensor.

First test your LIDAR angles with the program: rubot_lidar_test.py

Type the following commands to each terminal:

```shell
roslaunch gopigo3_control gopigo3_bringup_hw.launch
rosrun gopigo3_control node_lidar_test.launch
```

You need to verify that the forward direction corresponds to the zero angle. Is it true???

Now you can perform the autonomous navigation defined in "rubot_self_nav.py"

Carefull!: be sure the new python file is executable

Type in different terminals:

```shell
roslaunch gopigo3_control gopigo3_bringup_hw.launch
roslaunch gopigo3_control rubot_self_nav.launch
```

In order to see the rubot with the topics information we will use rviz. Open rviz in a new terminal.

In rviz, select the fixed frame to "base_scan", and add Camera and LaserScan with the corresponding topics names.

You can then save the config file as laserscan.rviz name and use it in the launch file
![Getting Started](./Images/2_self_nav.png)

### **2.4. Wall Follower**

This control task consist on find a wall and follow it at a certain distance. We will see that this is an important control task because this will be used later to make accurate maps of working environments.

We have developed a geometrical method for wall follower:

The instructions to perform the python program are in the notebook:

https://github.com/Albert-Alvarez/ros-gopigo3/blob/lab-sessions/develop/ROS%20con%20GoPiGo3%20-%20S4.md

<img src="./Images/2_wall_follower1.png">

We have created a launch file to start the node responsible to wall follower process.

```shell
roslaunch gopigo3_control gopigo3_bringup_hw.launch
roslaunch gopigo3_control node_wall_follower_gm.launch
```

You can see the video result:

[![Watch the video](https://img.youtube.com/vi/z5sAyiFs-RU/maxresdefault.jpg)](https://youtu.be/z5sAyiFs-RU)

### **2.5. Go to POSE**

The objective is to program the robot to reach a speciffic target POSE defining:

- x position
- y position
- angle orientation (from 0º to 180º)

We can take the same python script you have programed for simulated Gazebo environment

Type:

```shell
roslaunch gopigo3_control gopigo3_bringup_hw.launch
roslaunch gopigo_control node_go2pose.launch
```
