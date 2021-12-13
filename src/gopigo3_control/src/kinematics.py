#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import numpy as np

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
    xdata1.append(x_odom)
    ydata1.append(y_odom)
    # set/update the x and y axes data
    #line1.set_data(xdata1, ydata1)

def move_rubot(lin_vel,ang_vel,distance):
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/odom',Odometry, odom_callback)
    rate = rospy.Rate(10) # 10hz
    vel = Twist()
    while not rospy.is_shutdown():
        vel.linear.x = lin_vel
        vel.linear.y = 0
        vel.linear.z = 0
        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = ang_vel

        if(x_odom >= distance):
            rospy.loginfo("Robot Reached boundary")
            rospy.logwarn("Stopping robot")
            vel.linear.x = 0
            vel.angular.z = 0
            pub.publish(vel)
            break # needed to exit the while loop
        else:
            pub.publish(vel)
            rate.sleep()

if __name__ == '__main__':# needed when this file is used as external library
    try:
        
        x_odom = 0
        # lists to store x and y axis points
        xdata1, ydata1 = [], []

        rospy.init_node('rubot_nav', anonymous=False) # init node has to be made before get_param
        v= rospy.get_param("~v") # ~ because param is inside node in launch file 
        w= rospy.get_param("~w") # this is important to distinguish the different robots
        d= rospy.get_param("~d")
        move_rubot(v,w,d)
        wl,wr=fkine(v,w)

        # setting a style to use
        plt.style.use('fivethirtyeight')
        # create a figure, axis and plot element
        fig = plt.figure()
        # define subplots and their positions in figure
        plt1 = fig.add_subplot(221)
        plt1.set_title('Odometry')
        ax1 = plt.axes(xlim=(-1, 1), ylim=(-1, 1))
        line1, = ax1.plot([], [], lw=2)
        line1.set_data(xdata1, ydata1)
        #plt1.plot(xdata1, ydata1, color ='r')
        
        plt2 = fig.add_subplot(222)
        ax2 = plt.axes(xlim=(0, 1), ylim=(-1, 1))
        plt2.plot(0.5, wl, color ='b')
        plt2.set_title('wl')
        plt3 = fig.add_subplot(223)
        ax3 = plt.axes(xlim=(0, 1), ylim=(-1, 1))
        plt3.plot(0.8, wr, color ='g')
        plt3.set_title('wr')
 
        # adjusting space between subplots
        fig.subplots_adjust(hspace=.5,wspace=0.5)
 
        # show the plot
        plt.show()
    except rospy.ROSInterruptException:
        pass