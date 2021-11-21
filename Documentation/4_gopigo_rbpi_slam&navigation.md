# **SLAM & Navigation gopigo3 in RaspberryPi**
Autonomous navigation refers that the robot is able to move autonomously around the environment avoiding any obstacle.

In a hospital, a delivery robot carries samples or food from one room to another. 

The main objectives are:
- use SLAM (Simultaneous Localization and Mapping) techniques to generate and store a map of the hospital flor
- use Navigation techniques to find an optimal trajectory to reach a speciffic hospital target position

let's see how to fulfill these objectives

![Getting Starter](./Images/1_gopigo3_UB.png)

## **Create a slam&navigation package**

To perform SLAM & Navigation, we need to create a specific "gopigo_slam" package. You can take this package as a frame for other slam packages related to other robot models. Take care about the URDF model path and some yaml parameters.

This package is already created and you will use it to:
- generate the map of your maze
- navigate to speciffic target points within the map

### **1. Generate the map**

To generate the map we need first to launch:
- gopigo3_node
- ydlidar or rplidar
- raspicam

Open diferent terminals to launch the diferent nodes:
```shell
roslaunch gopigo3_node gopigo3.launch
roslaunch ydlidar lidar.launch (or roslaunch rplidar_ros rplidar.launch)
roslaunch raspicam_node camerav2_1280x960_10fps.launch enable_raw:=true camera_frame_id:="base_scan"
```
You can also launch a bringup launch file to launch gopigo3 and LIDAR (do not launch the raspicam to speed up the gmapping process).

Type:
```shell
roslaunch gopigo_control gopigo3yd_bringup.launch
```
> Select the corresponding launch file for the rp lidar version

we can see now the nodes that are running in our ROS workspace:
<img src="./Images/2_nodes_cam.png">

Now we need to launch the slam_gmapping node.

If we use new values of "gmapping.launch" parameters:
- delta: 0.01m 
- map_update_interval: 1s

And from the gopigo3.urdf model in differentialdrive pluggin:
- Acceleration: 0.5
- Torque: 1

we obtain a more accurate movement and map

Open another terminal and type:
```shell
roslaunch gopigo_control rubotYD_wall_follower_gm.launch
roslaunch gopigo_slam gopigo3yd_slam.launch
```
We need to move the gopigo3 around the map either with keyboard or with wall_follower python program
```shell
rosrun key_teleop key_teleop.py /key_vel:=/cmd_vel
or
roslaunch wall_follower wall_follower.launch
```
Once you have finish the map, you need to launch the map_saver program from map_server package:
```shell
rosrun map_server map_saver -f hospital1map
```
You will get two files in the specified folder of your workspace: maze.pgm and maze.yaml.

Provided with the map, we are ready to perform robot navigation with the GoPiGo3.

### **2. navigate to speciffic target points within the map**

To navigate within the map we need first to launch:
- gopigo3_node
- ydlidar or rplidar
- raspicam

Open diferent terminals to launch the diferent nodes:
```shell
roslaunch gopigo3_node gopigo3.launch
roslaunch ydlidar lidar.launch (or roslaunch rplidar_ros rplidar.launch)
roslaunch raspicam_node camerav2_1280x960_10fps.launch enable_raw:=true camera_frame_id:="base_scan"
```
Open Navigation launch file including the map location:
```shell
roslaunch gopigo_slam gopigo3_navigation_maze.launch
or
roslaunch gopigo_slam gopigo3rp_navigation_maze.launch
```
You need first to locate the robot in the initial position and choose the target destination.

You can see the optimized trajectory. The gopigo starts to follow this trajectory and if an obstacle appears, the robot avoid this obstacle and find in realtime the new optimal trajectory to the target point. 