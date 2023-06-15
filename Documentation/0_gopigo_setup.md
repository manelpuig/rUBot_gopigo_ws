# **gopigo3 setup**

Gopigo3 robot prototype is based on:
- On-board computer based on raspberrypi4 board
- 2 DC-motor with encoder for differential drive controller
- RaspiCAM RGB camera
- LIDAR sensor

The main objectives of this chapter are:
- Getting started with gopigo in simulation environment
- Getting started with gopigo in real raspberrypi4 based robot

## **1. Getting started with gopigo in simulation environment**

The simulation environment is properly installed in "ROS1_Noetic_osrf" docker container.


### **1.1. Connection to Docker container**

Follow the instructions:
- Run the "ROS1_Noetic_osrf" docker container
- Connect the Visual Studio Code to the docker container
- Run the XLaunch program with the speciffic settings

You are ready to work with ROS Noetic

### **1.2. Clone a repository**

The first time you have to clone the "rUBot_gopigo_ws" repository to the home folder.
```shell
cd /home
git clone https://github.com/manelpuig/rUBot_gopigo_ws
cd rUBot_gopigo_ws
catkin_make
```

Review the ~/.bashrc: Verify the last lines:
```shell
source /opt/ros/noetic/setup.bash
source /home/rUBot_gopigo_ws/devel/setup.bash
```

### **1.3. gopigo bringup**

Now you can bringup our robot:
- launch the gopigo3 node: able to control de 2 motors and measure the odometry
- launch the raspicam node
- launch the LIDAR sensor node

A launch file is made to automatically make the bringup hardware:
```shell
roslaunch gopigo3_description gopigo3_bringup_sw.launch
```
> Specify in launch file the final urdf file model


## **2. Getting started with gopigo in real raspberrypi4 based robot**

The raspberrypi4 onboard is preinstalled with:
- Ubuntu20.04
- ROS1 Noetic

When connected to power, it is configured to:
- generate a hotspot "rubot_XX"
- virtual monitor installed
- LIDAR activated 
- raspicam activated 

### **2.1. Robot connection from PC**

To connect your PC to the Robot, we have to:
- select the rubot hotspot:
    - SSID name: rubot_XX 
    - password "CorrePiCorre"

#### **2.1.1. Using nomachine remote desktop**
To connect your computer to the robot using Nomachine:
- Plug the pendrive to USB port of PC in Lab IE
- Execute "nxplayer.exe" 

Add new robot and edit the connection:
- Name: rUBot_XX
- Host: 10.42.0.1
- Port: 4000
- Protocol: NX

Connect
- user: pi
- password: ubuntu0ubuntu1

You will have the rUBot desktop on your windows nomachine screen

#### **2.1.2. Using VS Code remote connection**
To connect your computer to the robot using VS Code remote connection:
- You need the latest version (1.75.1).

Install the extensions:
- Remote development
- Git Extension Pack (in your remote 10.42.0.1 board)

The **first time** you connect the VS Code to the Remote machine:

- You will need to sign autorization to VS code to access github (will be in left-side bar menu, accounts symbol)
- When sync the changes, you will have to type your email and user name (following the git output information)
- You can work without internet connection, but When you want to sync your repository you will need the ethernet cable connected.

For **succesive connections**, follow the instructions:

- In windows open VS Code as administrator
- Select the rUBot_XX wifi network
- From "remote Explorer" (left-side bar menu) select "Remote" and type the IP
- Select linux system for remote connection
- type the password

You will have the VS Code attached to remote machine!

### **2.2. Clone a repository**

The first time you have to clone the "rUBot_gopigo_ws" repository to the home folder.
```shell
cd /home
git clone https://github.com/manelpuig/rUBot_gopigo_ws
cd rUBot_gopigo_ws
catkin_make
```
> If you have not internet connection you can copy the folder from a pendrive on the raspberrypi4 USB port

Review the ~/.bashrc: Verify the last lines:
```shell
source /opt/ros/noetic/setup.bash
source /home/rUBot_gopigo_ws/devel/setup.bash
```

### **2.3. gopigo bringup**

Now you can bringup our robot:
- launch the gopigo3 node: able to control de 2 motors and measure the odometry
- launch the raspicam node
- launch the LIDAR sensor node

A launch file is made to automatically make the bringup hardware:
```shell
roslaunch gopigo3_description gopigo3_bringup_hw.launch
```
> Specify in launch file the robot urdf file model you have used
