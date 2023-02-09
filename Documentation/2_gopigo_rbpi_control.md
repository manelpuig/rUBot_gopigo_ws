# **Control of gopigo3 in RaspberryPi**

The main objectives are:
- Describe the gopigo robot
- Describe the gopigo bringup installation process
- Control the gopigo robot to perform different movements


## **1. Gopigo3 robot prototype setup**

This tobot prototype is based on:

- On-board computer based on raspberrypi3 board
- 2 DC-motor with encoder for differential drive controller
- RaspiCAM RGB camera
- LIDAR sensor

![Getting Started](./Images/1_gopigo3_UB.png)


## **2. Gopigo3 bringup**

To properly perform a especific movement control we have first to install some HW packages:

### **2.1 gopigo3_node**

This node is responsible to drive the wheels and obtain the odometry

You have to clone (or copy) the repository in src folder

```shell
cd src
git clone https://github.com/ros-gopigo/gopigo3_node
```

### **2.2 LIDAR**

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

### **2.3. RASPICAM**

This node is responsible to view images from camera

You can install package following instructions in:https://github.com/UbiquityRobotics/raspicam_node

```shell
cd src
git clone https://github.com/UbiquityRobotics/raspicam_node
```

### **2.4. OPEN CV**

This package is used for image processing with OpenCV and ROS

This can be installed with apt (in opt/ros/noetic/share folder)

```shell
sudo apt install ros-noetic-vision-opencv
```

### **2.5. Teleop-tools**

This package is used to control robot with keyboard or joypad

This can be installed with apt (in opt/ros/noetic/share folder)

```shell
sudo apt install ros-noetic-teleop-tools
```

Once all drivers are installed, you can compile the ws and proceed for the bringup

### **2.6. Bringup**

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

## **3. gopigo3 first control movements**

First, let's control the gopigo3 movement to perform:

- movement control using keyboard
- Navigation
- autonomous navigation
- autonomous navigation following right wall
- Navigation to speciffic POSE

We have created a specific package "gopigo_control" where all these controls are programed.
You can review the src/gopigo_control folder where there are 2 new folders:

- scrip folder: with the python nodes for specific movement control
- launch folder: with programs to launch the movement control

### **3.1. Keyboard movement control**

To control the gopigo robot with keyboard, we have installed "teleop_tools" package. 

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

### **3.2. Navigation**

You want to perform the movement control:
- with a desired v and w
- up to a x distance "d"

This is made in a speciffic python node "move3_gopigo_distance.py".

Type:

```shell
roslaunch gopigo3_control gopigo3_bringup_hw.launch
roslaunch gopigo3_control node_nav.launch
```
### **Lab Activity:**

You want to perform the movement control:
- with a desired v and w
- during a time interval "t"

### **3.3. Autonomous navigation**

For autonomous navigation you need the LIDAR sensor.

First test your LIDAR angles with the program: rubot_lidar_test.py

Type the following commands to each terminal:

```shell
roslaunch gopigo3_control gopigo3_bringup_hw.launch
rosrun gopigo3_control node_lidar_test.launch
```
### **Lab Activity**
Verify that the forward direction corresponds to the zero angle. Is it true???
- Modify the "rubot_lidar_test.py" node to be independent of laser beams
- The program will take into account only the angle

### **Lab Activity**
Now you can perform the autonomous navigation defined in node "rubot_self_nav.py"

You will have to modify the python node to take into account:
- the zero angle if you have RP or YD lidar
- To be independent of the number of laser beams

To verify, type in different terminals:

```shell
roslaunch gopigo3_control gopigo3_bringup_hw.launch
roslaunch gopigo3_control rubot_self_nav.launch
```

In order to see the rubot with the topics information we will use rviz. 

In rviz, verify the fixed frame to "base_scan", and add Camera and LaserScan with the corresponding topics names.

You can then save the config file as laserscan.rviz name and use it in the launch file
![Getting Started](./Images/2_self_nav.png)

### **3.4. Wall Follower**

This control task consist on find a wall and follow it at a certain distance. We will see that this is an important control task because this will be used later to make accurate maps of working environments.

We have developed a geometrical method for wall follower:

The instructions to perform the python program are in the notebook:

https://github.com/Albert-Alvarez/ros-gopigo3/blob/lab-sessions/develop/ROS%20con%20GoPiGo3%20-%20S4.md

<img src="./Images/2_wall_follower1.png">

We have created a launch file to start the node responsible to wall follower process.

### **Lab Activity**
You will have to adjust the different parameters to obtain a soft mobement

To verify the performances, type:

```shell
roslaunch gopigo3_control gopigo3_bringup_hw.launch
roslaunch gopigo3_control node_wall_follower_gm.launch
```

You can see the video result:

[![Watch the video](https://img.youtube.com/vi/z5sAyiFs-RU/maxresdefault.jpg)](https://youtu.be/z5sAyiFs-RU)

### **3.5. Go to POSE**

The objective is to program the robot to reach a speciffic target POSE defining:

- x position
- y position
- angle orientation (from 0º to 180º)

We can take the same python node you have programed for simulated Gazebo environment

### **Lab Activity**
You will have to adjust the resolution to reach the POSE with enough accuracy.

To verify the performances, open a terminal and type:

```shell
roslaunch gopigo3_control gopigo3_bringup_hw.launch
roslaunch gopigo_control node_go2pose.launch
```
