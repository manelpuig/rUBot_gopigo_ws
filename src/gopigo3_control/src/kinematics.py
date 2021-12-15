#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import numpy as np
import time
import Tkinter as tk
#from Tkinter import ttk

def fkine(v,w):
    b=1 # wheel distance
    R=0.3 # wheel radious
    wl=(v-(b/2)*w)/R
    wr=(v+(b/2)*w)/R
    return(wl,wr)

def odom_callback(data):
    global x_odom
    x_odom=data.pose.pose.position.x
    y_odom=data.pose.pose.position.y
    rospy.loginfo("Robot Odometry x= %f\t y= %f\n",x_odom,y_odom)
    xdata.append(x_odom)
    ydata.append(y_odom)

def move_rubot(lin_vel,ang_vel,distance):
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/odom',Odometry, odom_callback)
    rate = rospy.Rate(10) # 10hz
    vel = Twist()
    while x_odom < distance:
        vel.linear.x = lin_vel
        vel.linear.y = 0
        vel.linear.z = 0
        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = ang_vel
        pub.publish(vel)
        rate.sleep()
    else:
        rospy.loginfo("Robot Reached boundary")
        rospy.logwarn("Stopping robot")
        vel.linear.x = 0
        vel.angular.z = 0
        pub.publish(vel)
        #rospy.spin()
        
def close_window(): 
    window.destroy()

if __name__ == '__main__':# needed when this file is used as external library
    try:
        
        x_odom = 0
        xdata, ydata = [], []
        
        rospy.init_node('rubot_nav', anonymous=False) # init node has to be made before get_param
        v= rospy.get_param("~v") # ~ because param is inside node in launch file 
        w= rospy.get_param("~w") # this is important to distinguish the different robots
        d= rospy.get_param("~d")
        move_rubot(v,w,d)
        wl,wr=fkine(v,w)
        # Create a window using the Tk class
        window = tk.Tk()
        # Set the window title
        window.title("rUBot Kinematics control")

        # Quit button
        exit_button = tk.Button(window, text = "Exit", 
        command=close_window) 

        plt.close('all')
        fig = plt.figure()
        ax1 = plt.subplot(221)
        ax2 = plt.subplot(223)
        ax3 = plt.subplot(122)
        
        ax1.plot([1,wl])
        ax1.set_xlabel('x-label', fontsize=15)
        ax1.set_ylabel('wl', fontsize=15)
        ax1.set_title('Title', fontsize=20)
        ax2.plot([1,wr])
        ax3.plot(xdata,ydata)
        ax3.set_xlabel('x', fontsize=15)
        ax3.set_ylabel('y', fontsize=15)
        ax3.set_title('Odometry', fontsize=20)
        # show the plot
        plt.tight_layout()
        plt.show()
         
        window.mainloop()
    except rospy.ROSInterruptException:
        pass