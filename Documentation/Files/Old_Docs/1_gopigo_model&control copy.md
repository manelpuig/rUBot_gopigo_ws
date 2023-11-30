# **gopigo3 model & control**
The objectives of this chapter are:
- Create a model of our gopigo3 robot 
- Create a model of the virtual environment
- Bringup the robot in our virtual environment
- Control the robot movement with obstacle avoidance


The final model represents the real gopigo3 robot we will use in the laboratory

Interesting complementary information could be found:

- http://wiki.ros.org/Robots/gopigo3

- https://robots.ros.org/gopigo3/

- https://github.com/ros-gopigo3/gopigo3


<img src="./Images/1_gopigo3_UB.png" />

## **1. rUBot gopigo3 model generation**

We have already created the "gopigo3_description" package to generate our robot model.
We remind you the needed instructions to create the package if you want to do it from zero:
```shell
cd /home/user/rUBot_gopigo_ws/src
catkin_create_pkg gopigo3_description rospy
cd ..
catkin_make
```
Then open the .bashrc file and verify the environment variables and source to the proper workspace:
```shell
source /home/user/rUBot_gopigo_ws/devel/setup.bash
```
To create our robot model, we use **URDF files** (Unified Robot Description Format). URDF file is an XML format file for representing a robot model (http://wiki.ros.org/urdf/Tutorials).

We have created 2 folders for model description:
- URDF: folder where different URDF models are located
- meshes: folder where 3D body models in stl format are located.

The main parts of URDF model are:
- links: diferent bodies/plastic elements
- joints: connection between 2 links 
- sensors & actuators plugins (2D camera, LIDAR and DC motors)

The link definition contains:
- visual properties: the origin, geometry and material
- collision properties: the origin and geomnetry
- inertial properties: the origin, mass and inertia matrix

The joint definition contains:
- joint Type (fixed, continuous)
- parent and child frames
- origin frame
- rotation axis

In the case or wright wheel:
```xml
<!-- Right Wheel -->
  <link name="right_wheel">
    <visual>
      <origin xyz="0 0 0" rpy="1.570795 0 0" />
      <geometry>
          <cylinder length="0.05" radius="0.1" />
      </geometry>
      <material name="orange"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="1.570795 0 0" />
      <geometry>
          <cylinder length="0.05" radius="0.1" />
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0" rpy="1.570795 0 0" />
      <mass value="0.15"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>
  
  <!-- Right Wheel joint -->
  <joint name="joint_right_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.30 0.025" rpy="0 0 0" /> 
    <axis xyz="0 1 0" />
  </joint>
```
The gopigo3 model includes different sensors and actuators:

Sensors:
- a two-dimensional camera: correspondas to RBPi camera
- a 360º LIDAR sensor: 
  - the EAI YDLIDAR X4 (https://www.robotshop.com/es/es/escaner-laser-360-ydlidar-x4.html) 
  - or RPLidar A1M8 (https://www.robotshop.com/es/es/rplidar-a1m8-kit-desarrollo-escaner-laser-360-grados.html)

Actuator:
- Differential drive actuator: based on 2 DC motors with encoders to obtain the Odometry information

The full model contains also information about the sensor and actuator controllers using specific **Gazebo plugins** (http://gazebosim.org/tutorials?tut=ros_gzplugins#Tutorial:UsingGazebopluginswithROS). 

Gazebo plugins give your URDF models greater functionality and compatiblility with ROS messages and service calls for sensor output and motor input. 

These plugins can be referenced through a URDF file, and to insert them in the URDF file, you have to follow the sintax:
- **Plugin for Differential drive actuator**:
```xml
 <!-- Differential Drive Controller -->
  <gazebo>
    <plugin filename="libgazebo_ros_diff_drive.so" name="gopigo3_controller">
      <commandTopic>cmd_vel</commandTopic>
      <odometryTopic>odom</odometryTopic>
      <odometryFrame>odom</odometryFrame>
      <odometrySource>world</odometrySource>
      <publishOdomTF>true</publishOdomTF>
      <robotBaseFrame>base_footprint</robotBaseFrame>
      <publishWheelTF>false</publishWheelTF>
      <publishTf>true</publishTf>
      <publishWheelJointState>true</publishWheelJointState>
      <legacyMode>false</legacyMode>
      <updateRate>30</updateRate>
      <leftJoint>wheel_left_joint</leftJoint>
      <rightJoint>wheel_right_joint</rightJoint>
      <wheelSeparation>0.116</wheelSeparation>
      <wheelDiameter>0.066</wheelDiameter>
      <wheelAcceleration>0.5</wheelAcceleration>
      <wheelTorque>1</wheelTorque>
      <rosDebugLevel>na</rosDebugLevel>
    </plugin>
  </gazebo>
  ```

- **Plugin for Raspicam sensor**:
```xml
  <!-- 2D Camera controller -->
  <gazebo reference="camera_rgb_frame">
    <sensor name="camera1" type="camera">
      <always_on>true</always_on>
      <visualize>false</visualize>
      <camera name="front">
        <horizontal_fov>1.3962634</horizontal_fov>
        <image>
          <width>800</width>
          <height>800</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.02</near>
          <far>300</far>
        </clip>
      </camera>
      <plugin filename="libgazebo_ros_camera.so" name="camera_controller">
        <alwaysOn>true</alwaysOn>
        <updateRate>30.0</updateRate>
        <cameraName>gopigo/camera1</cameraName>
        <imageTopicName>image_raw</imageTopicName>
        <cameraInfoTopicName>camera_info</cameraInfoTopicName>
        <frameName>camera_rgb_frame</frameName>
        <hackBaseline>0.07</hackBaseline>
        <distortionK1>0.0</distortionK1>
        <distortionK2>0.0</distortionK2>
        <distortionK3>0.0</distortionK3>
        <distortionT1>0.0</distortionT1>
        <distortionT2>0.0</distortionT2>
      </plugin>
    </sensor>
  </gazebo>
  ```
- **Plugin for LIDAR sensor**:
```xml
  <!-- Laser Distance Sensor YDLIDAR X4 controller-->
  <gazebo reference="base_scan">
    <material>Gazebo/FlatBlack</material>
    <sensor name="lds_lfcd_sensor" type="ray">
      <pose>0 0 0 0 0 0</pose>
      <visualize>false</visualize>
      <update_rate>10</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>720</samples>
            <resolution>1</resolution>
            <min_angle>0.0</min_angle>
            <max_angle>6.28319</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.120</min>
          <max>10</max>
          <resolution>0.015</resolution>
        </range>
        <noise>
          <type>gaussian</type>
          <!-- Noise parameters based on published spec for YDLIDAR X4
               is 1.5% at half range 4m (= 60mm, "+-160mm" accuracy at max. range 8m).
               A mean of 0.0m and stddev of 0.020m will put 99.7% of samples
               within 0.16m of the true reading. -->
          <mean>0.0</mean>
          <stddev>0.02</stddev>
        </noise>
      </ray>
      <plugin filename="libgazebo_ros_laser.so" name="gazebo_ros_lds_lfcd_controller">
        <topicName>scan</topicName>
        <frameName>base_scan</frameName>
      </plugin>
    </sensor>
  </gazebo>
  ```
#### **Generated models in URDF**
The diferent model files we have created in urdf folder are:
- gopigo3rp.urdf: with RPlidar
- gopigo3yd.urdf: with YDlidar

  > - Robots with RPlidar have been assembled with the motor in front position and then have the zero angle in its back
  > - Robots with YDlidar have been assembled with the motor in back position and then have the zero angle in its front
>
We use a specific "display.launch" launch file where we specify the robot model we want to open in rviz with a configuration specified in "urdf.rviz":
```xml
<?xml version="1.0"?>
<launch>
  <!-- set these parameters on Parameter Server -->
  <param name="robot_description" textfile="$(find gopigo3_description)/urdf/gopigo3rp.urdf" />
  <!-- send fake joint values -->
  <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
    <param name="use_gui" value="False"/>
  </node>
  <!-- Combine joint values -->
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"/>
  <!-- Show in Rviz   -->
  <node name="rviz" pkg="rviz" type="rviz" args="-d $(find gopigo3_description)/rviz/urdf.rviz"/>
</launch>
```
Launch the ROS visualization tool to check that the model is properly built. 
RViz only represents the robot visual features. You have available all the options to check every aspect of the appearance of the model
```shell
roslaunch gopigo3_description display.launch
```

![](./Images/1_gopigo_stl.png)


In robotics research, always before working with a real robot, we simulate the robot behaviour in a virtual environment close to the real one. 

Gazebo is an open source 3D robotics simulator and includes an ODE physics engine and OpenGL rendering, and supports code integration for closed-loop control in robot drives. This is sensor simulation and actuator control.

We will create a new spawn.launch file to spawn the robot in an empty world:
```shell
roslaunch gopigo3_description spawn.launch
```
```xml
<launch>
 <!-- We resume the logic in gazebo_ros package empty_world.launch, -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch"/>
 <!-- Spawn gopigo3 robot into Gazebo -->
    <!-- Robot URDF definition -->
    <arg name="model" default="gopigo3rp.urdf" />
    <param name="robot_description" textfile="$(find gopigo3_description)/urdf/$(arg model)"/>
  <node name="spawn_model" pkg="gazebo_ros" type="spawn_model" output="screen"
      args="-urdf -model gopigo3 -param robot_description"/>
</launch>
```
You can create a very simple world "gopigo3.world" using gazebo:
- Type: sudo gazebo
- add some objects in the empty world
- save the final world to "worlds" folder

The robot could be spawn in a predefined position inside this new created world using this new spawn_world.launch file:
```xml
<launch>
    <!-- Define the needed parameters -->
    <arg name="world" default="gopigo3.world"/> 
    <arg name="model" default="gopigo3rp.urdf" />
    <arg name="x_pos" default="0.5"/>
    <arg name="y_pos" default="0.5"/>
    <arg name="z_pos" default="0.0"/>
  
    <include file="$(find gazebo_ros)/launch/empty_world.launch">
        <arg name="world_name" value="$(find gopigo3_description)/worlds/$(arg world)"/>
    </include>
    <!-- Spawn gopigo3 robot into Gazebo -->
       <!-- Robot URDF definition -->
       <param name="robot_description" textfile="$(find gopigo3_description)/urdf/$(arg model)"/>
    <node name="spawn_model" pkg="gazebo_ros" type="spawn_model" output="screen"
        args="-urdf -model gopigo3 -param robot_description -x $(arg x_pos) -y $(arg y_pos) -z $(arg z_pos)"/>
   </launch>
   ```
Type the following to spawn the robot inside the world and test the sensor values using rviz:
```shell
roslaunch gopigo3_description spawn_world.launch
roslaunch gopigo3_description display.launch
```
![Getting Started](./Images/1_gopigo3_spawn_rviz.png)

You can see the nodes and topics generated using rqt_graph

![](./Images/01_rubot_spawn_rqt.png)


### **Exercise**
Modify the robot model to change the color (in RVIZ and Gazebo) of LIDAR sensor in function of the model:
- Brawn for RP Lidar
- BLUE for YD lidar

## **2. Design the Project world**

Here we have first to design the project world, for exemple a maze from where our rUBot gopigo3 has to navigate autonomously.

There is a very useful and simple tool to design a proper world: "Building editor" in gazebo.

- Open gazebo as superuser:
```shell
sudo gazebo
```
- build your world using "Building Editor" in Edit menu

![](./Images/1_BuildingWorld1_1.png)

- save the generated model in a model folder (without extension)

- Close the Builder Editor, modify the model position and add other elements if needed. 
- save the generated world (with extension .world) in the world folder.

Once you finish is better to close the terminal you have worked as superuser

#### ***Modify a created world***
To modify a previously created world:
- Open a terminal where you have the world you want to modify
- type: sudo gazebo ./maze1.world
- make modifications
- save your world in the desired directory
- close gazebo and the terminal

#### **Create world with model parts**
You can create model parts like walls of 1m or 0,5m with a geometry and color, using building editor. These parts can be saved in "home/ubuntu/building_editor_models/" and you will have acces in gazebo insert section. Then you can construct your world adding these model parts.

This is an exemple:
![](./Images/1_BuildingEditor.png)

### **Exercise:**
Generate a proper world corresponding to the real world we want to spawn our gopigo3 robot in. For exemple a maze.

Use model parts in size 90cmx30cmx1cm and 60cmx30cmx1cm

Save this world as maze.world

## **3. Bringup the gopigo3 robot in project world**

Now, you can bringup the gopigo3 robot in our generated world. 
You have to create a "bringup.launch" file:

``` shell
roslaunch gopigo3_description bringup.launch
```
![](./Images/1_maze1.png)
> To properly kill a gazebo process, type
>killall gzserver && killall gzclient
>or
>pkill gzserver && pkill gzclient
### **Exercise:**
Generate a proper bringup launch file to:
- Spawn your gopigo model in the generated world
- Take into account the Lidar model
- Open RVIZ
- Specify the desired POSE

## **4. gopigo3 control**

### **4.1 gopigo3 navigation**

Once the world has been generated we will create a ROS Package "gopigo3_control" to perform the autonomous navigation.

This package is already created, but we will remind you how it is created:
```shell
cd ~/Desktop/rUBot_gopigo_ws/src
catkin_create_pkg gopigo3_control rospy std_msgs sensor_msgs geometry_msgs nav_msgs
cd ..
catkin_make
```

### **3.1. gopigo3 bringup**

Usually we first bring up the robot in a virtual environment.

A speciffic "gopigo3_bringup_sw.launch" file is created considering if you have rp or yd lidar installed.
```shell
roslaunch gopigo3_control gopigo3_bringup_sw.launch
```
The closed loop control structure is depicted below:

![](./Images/1_rubot_control.png)

### **3.2. gopigo3 control with keyboard**

To control the robot with the Keyboard you have to install the "teleop-tools" package:

```shell
sudo apt-get install ros-noetic-teleop-tools
sudo apt-get install ros-noetic-teleop-twist-keyboard
```
Then you will be able to control the robot with the Keyboard typing:
```shell
roslaunch gopigo3_control gopigo3_bringup_sw.launch
rosrun key_teleop key_teleop.py /key_vel:=/cmd_vel
```
### **3.3. gopigo3 control with custom node**

To control the robot with a custom designed node, We will create a navigation python file in "src" folder:
- gopigo3_nav.py: to specify a twist message and a maximum distance

Specific launch file has been created to launch the node and python file created above:
```shell
roslaunch gopigo3_control gopigo3_bringup_sw.launch
roslaunch gopigo3_control node_nav.launch
```
![Getting Started](./Images/1_rubot_move3.png)

## **3.4. gopigo3 autonomous navigation and obstacle avoidance**
In order to navigate autonomously and avoid obstacles, we have created diferent python files in "src" folder:
- rubot_lidar_test.py: to test the LIDAR distance readings and angles
- rubot_self_nav.py: to perform a simple algorithm for navigation with obstacle avoidance
- rubot_wall_follow.py: to follow a wall preciselly for mapping purposes
- rubot_go2pose.py: to reach speciffic position and orientation

we will create also a "launch" folder including the corresponding launch files

#### **1. LIDAR test**
We have created a world to test the rubot model. This world is based on a square to verify that the LIDAR see the obstacle in the correct angle. We have to launch the "rubot_lidar_test.launch" file in the "gopigo3_control" package.

To launch this node type:
```shell
roslaunch gopigo3_control gopigo3_bringup_sw.launch
roslaunch gopigo3_control rubot_lidar_test.launch
rosrun key_teleop key_teleop.py /key_vel:=/cmd_vel
```
![](./Images/1_lidar_test.png)

#### **2. Autonomous navigation with obstacle avoidance**
We will use now the created world to test the autonomous navigation with obstacle avoidance performance. 

We have to launch the "rubot_self_nav.launch" file in the "rubot_control" package.
```shell
roslaunch gopigo3_control gopigo3_bringup_sw.launch
roslaunch gopigo3_control rubot_self_nav.launch
```
>Be careful:
>- Verify in rviz if you have to change the fixed frame to "odom" frame

![](./Images/1_rubot_self_nav.png)
The algorithm description functionality is:
- "rubot_self_nav.py": The Python script makes the robot go forward. 
    - LIDAR is allways searching the closest distance and the angle
    - when this distance is lower than a threshold, the robot goes backward with angular speed in the oposite direction of the minimum distance angle.

#### **3. Wall Follower**
Follow the wall accuratelly is an interesting challenge to make a map with precision to apply SLAM techniques for navigation purposes.

There are 2 main tasks:
- Create a python file "rubot_wall_follower.py" to perform the wall follower in the maze of our gopigo3 robot
- Create a launch file to initialyse all the needed nodes in our system for this control task

We have developed 2 different methods for wall follower:
- Geometrical method
- Lidar ranges method

##### **a) Geometrical method**

Follow the instructions to perform the rubot_wall_follower_gm.py python program are in the notebook: 
https://github.com/Albert-Alvarez/ros-gopigo3/blob/lab-sessions/develop/ROS%20con%20GoPiGo3%20-%20S4.md
![](./Images/2_wall_follower1.png)
A rubot_wall_follower_gm.launch is generated to test the node within a specified world
```shell
roslaunch gopigo3_control gopigo3_bringup_sw.launch
roslaunch gopigo3_control rubot_wall_follower_gm.launch
```
![](./Images/1_wall_follower_gm.png)

You can see a video for the Maze wall follower process in: 
[![IMAGE_ALT](https://img.youtube.com/vi/z5sAyiFs-RU/maxresdefault.jpg)](https://youtu.be/z5sAyiFs-RU)


##### **b) Lidar ranges method**

We have created another rubot_wall_follower_rg.py file based on the reading distances from LIDAR in the ranges: front, front-right, front-left, right, left, back-right and back-left, and perform a specific actuation in function of the minimum distance readings.

Follow the instructions to create the rubot_wall_follower_rg.py python file: https://www.theconstructsim.com/wall-follower-algorithm/

![](./Images/1_wall_follower.png)
The algorith is based on laser ranges test and depends on the LIDAR type:
![](./Images/1_wall_follower2.png)

```shell
roslaunch gopigo3_control gopigo3_bringup_sw.launch
roslaunch gopigo3_control rubot_wall_follower_rg.launch
```
![](./Images/1_wall_ranges.png)

#### **4. Go to POSE**
Define a specific Position and Orientation as a target point to gopigo3 robot

x target point
y target point
f yaw orientation angle in deg

Modify the python script developed in turlesim control package according to the odom message type

For validation type:
```shell
roslaunch gopigo3_control gopigo3_bringup_sw.launch
roslaunch gopigo3_control rubot_go2pose.launch
```
![](./Images/1_rubot_go2point.png)
