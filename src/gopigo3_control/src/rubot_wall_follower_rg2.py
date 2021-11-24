#! /usr/bin/env python

from sys import flags
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations

import math

pub_ = None
flag1=0
regions_ = {
    'right': 0,
    'fright': 0,
    'front': 0,
    'fleft': 0,
    'left': 0,
    'rleft': 0,
    'rright': 0,
}
state_ = 0
state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'turn right',
    3: 'back',
    4: 'follow the wall',
}

def clbk_laser(msg):
    global regions_
    regions_ = {
        'left':  min(min(msg.ranges[176:184]), 3),
        'fleft': min(min(msg.ranges[100:175]), 3),
        'front':  min(msg.ranges[0], 3),
        'fright':  min(min(msg.ranges[545:619]), 3),
        'right':   min(min(msg.ranges[539:541]), 3),
        'rright':  min(min(msg.ranges[380:490]), 3),
        'rleft':  min(min(msg.ranges[230:340]), 3),
    }
    print ("front distance: "+ str(regions_["front"]))
    print ("right distance: "+ str(regions_["right"]))
    print ("front-right distance: "+ str(regions_["fright"]))
    print ("front-left distance: "+ str(regions_["fleft"]))
    print ("rear-right distance: "+ str(regions_["rright"]))
    print ("rear-left distance: "+ str(regions_["rleft"]))

    take_action()

#test

def change_state(state):
    global state_, state_dict_
    if state is not state_:
        print ('Wall follower - [%s] - %s' % (state, state_dict_[state]))
        state_ = state


def take_action():
    global regions_
    regions = regions_
    msg = Twist()
    linear_x = 0
    angular_z = 0
   


    state_description = ''

    d = 1/2
    dmin=0.65/2
    dmax=1

    if regions['front'] > d and regions['fleft'] > d and regions['fright'] > d: #and regions['rright'] > d and regions['rleft'] > d:
        state_description = 'case 1 - nothing'
        change_state(0)     #caso inicial no ha detectado nada aun 

    #elif regions['rleft'] > dmax :
    #    state_description = 'case 11 - fleft and fright'
    #    change_state(1) #caso de seguridad para evitar colisiones
    #elif regions['rright'] > dmax :
    #    state_description = 'case 11 - fleft and fright'
    #    change_state(2) #caso de seguridad para evitar colisiones
    
    elif regions['front'] < dmin or regions['left'] < dmin or regions['right'] <dmin or regions['fleft'] < dmin or regions['fright'] <dmin:
            state_description = 'case 2 - back to avoid colision'
            change_state(3) #caso de seguridad para evitar colisiones

    elif regions['front'] < d and regions['fright'] > d and regions['fleft'] > d:
        state_description = 'case 3 - front detected '
        change_state(1)     #detecta front --gira izquierda para orientarse

    elif regions['front'] < d and regions['fright'] < (d+15) and regions['fleft'] > d:
        state_description = 'case 4 - front and front right detected'
        change_state(1) #detecta front --gira izquierda +15

    elif regions['fright'] < d and regions['right'] < d and regions['fleft'] > d:
        state_description = 'case 5 - following wall '
        change_state(4) #detecta right i siguie la pared







    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)


def find_wall():        #0
    global flag1
    msg = Twist()
    msg.linear.x = 0.2
    #msg.angular.z = 0.01
    print ("buscando wall")
    if flag1>0:
        msg.angular.z=-0.5

    return msg

def turn_left():        #1
    msg = Twist()
    msg.angular.z = 0.5
    print ("girando izquierda")
    return msg

def turn_right():       #2
    global regions_
    regions = regions_

    msg = Twist()
    msg.angular.z = -0.5
    if regions['fright'] > 1.5:
        msg.linear.z = -0.3
    print("correcion")
    return msg

def back():             #3
    msg = Twist()
    msg.linear.x = -0.2
    msg.angular.z = 0.4
    print ("Back Back Back")
    return msg

def follow_the_wall():      #4
    global regions_
    global flag1
    regions = regions_
    flag1=1
    msg = Twist()
    msg.linear.x = 0.2
    msg.linear.z = 0.01
    print("follow wall")
    if regions['rright'] > 1.2:
        msg.linear.z = -0.3
        print("correcion")

    
    return msg


def main():
    global pub_

    rospy.init_node('wall_follower')
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        msg = Twist()
        if state_ == 0:
            msg = find_wall()
        elif state_ == 1:
            msg = turn_left()
        elif state_ == 2:
            msg = turn_right()
        elif state_ == 3:
            msg = back()  
        elif state_ == 4:
            msg = follow_the_wall()
            pass
        else:
            rospy.logerr('Unknown state!')

        pub_.publish(msg)

        rate.sleep()


if __name__ == '__main__':
    main()