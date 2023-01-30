
#### **b) ranges method**
In src folder you create the python file for wall follower purposes

The algorith is based on laser ranges test and depends on the LIDAR type:

<img src="./Images/1_wall_follower2.png">

Take into account that:
- RP LIDAR has 180º rotation
- YDlidar in front direction has 2 different ranges [660:719] and [0:60]
- YDlidar sends some 0 values due to wrong readings. They have to be changed to high value to be able to take the minimum falue from the desired range.

The "rubot_wall_follower_rg_filter.py" has to integrate this in callback function

``` python
def clbk_laser(msg):
    global regions_, LIDAR
    if LIDAR == "RP":
	    regions_ = {
		'left':  min(min(msg.ranges[539:541]), 3),
		'fleft': min(min(msg.ranges[421:538]), 3),
		'front':  min(min(msg.ranges[300:420]), 3),
		'fright':  min(min(msg.ranges[182:300]), 3),
		'right':   min(min(msg.ranges[179:181]), 3),
	    }
	    print ("LIDAR: RP")
    else:
        for i, val in msg.ranges:
            if val == 0:
                msg.ranges[i]=3 # YD Lidar sends some 0 values and then the minimum is allways 0
	    regions_ = {
		'left':  min(min(msg.ranges[179:181]), 3),
		'fleft': min(min(msg.ranges[60:178]), 3),
		'front':  min(min(msg.ranges[661:719]),min(msg.ranges[0:59]), 3),
		'fright':  min(min(msg.ranges[542:660]), 3),
		'right':   min(min(msg.ranges[539:541]), 3),
	    }
	    print ("LIDAR: YD")
 
    print ("front distance: "+ str(regions_["front"]))
    print ("right distance: "+ str(regions_["right"]))
    print ("front-right distance: "+ str(regions_["fright"]))

    take_action()
```
Type:
```shell
roslaunch gopigo_control rubotRP_wall_follower_rg.launch
```
```xml
<launch>
  <!-- launch gopigo3   -->
  <include file="$(find gopigo3_node)/launch/gopigo3.launch"/>
  <!-- launch ydlidar   -->
  <include file="$(find rplidar_ros)/launch/rplidar.launch"/>

  <!-- launch raspicam   -->
  <!--<include file="$(find raspicam_node)/launch/camerav2_1280x960_10fps.launch">
	<arg name="enable_raw" value="true"/>
	<arg name="camera_frame_id" value="base_scan"/>
  </include>-->
  <!-- launch follow wall   -->
	<arg name="LIDAR" default="RP" />
  <node name="wall_follow" pkg="gopigo_control" type="rubot_wall_follower_rg.py" output="screen" >
	<param name="LIDAR" value="$(arg LIDAR)"/>
  </node>
  <!-- Show in Rviz   -->
  <!--<node name="rviz" pkg="rviz" type="rviz" args="-d $(find obstacle_avoidance)/rviz/laserscan.rviz"/>-->
</launch>
```