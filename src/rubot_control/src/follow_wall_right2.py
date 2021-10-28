#! /usr/bin/env python

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
    3: 'follow the wall',
}

def clbk_laser(msg):
    global regions_
    regions_ = {
        'left':  min(min(msg.ranges[178:182]), 3),
        'fleft': min(min(msg.ranges[1:177]), 3),
        'rleft':  min(min(msg.ranges[230:340]), 3),
        'front':  min(msg.ranges[0], 3),
        'fright':  min(min(msg.ranges[543:719]), 3),
        'right':   min(min(msg.ranges[538:542]), 3),
        'rright':  min(min(msg.ranges[380:490]), 3),
        
    }
    print ("front distance: "+ str(regions_["front"]))
    print ("right distance: "+ str(regions_["right"]))
    print ("front-right distance: "+ str(regions_["fright"]))
    print ("front-left distance: "+ str(regions_["fleft"]))
    print ("rear-right distance: "+ str(regions_["rright"]))
    print ("rear-left distance: "+ str(regions_["rleft"]))

    take_action()


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

    d = 0.5  #1
    if regions['front'] > d and regions['fleft'] > d and regions['fright'] > d and regions['rright'] > d and regions['rleft'] > d:
        state_description = 'case 1 - nothing'
        change_state(0)     #caso inicial no detecta nada 
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] > d and regions['rright'] > d and regions['rleft'] > d:
        state_description = 'case 2 - front'
        change_state(1)     # detecta front y giramos izquierda
    #'''regions['front'] > d and''' el margen del 0,2 es para evitar que entren en el caso y salga por variacion en la medida
    elif regions['front'] > (d+0.2) and  regions['fleft'] > d and regions['fright'] < d and regions['rright'] > d and regions['rleft'] > d:
        state_description = 'case 3 - fright'
        change_state(1)   # margen en front  detecta fright giras mas izquierda        #2 follow wall  
    elif regions['front'] > (d+0.2) and regions['fleft'] < d and regions['fright'] > d and regions['rright'] > d and regions['rleft'] > d:
        state_description = 'case 4 - fleft'
        change_state(2)  # margen en front detecta izquierda gira derecha 
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] < d and regions['rright'] > d and regions['rleft'] > d: 
        state_description = 'case 5 - front and fright'
        change_state(1) #detecta front y front right 
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] > d and regions['rright'] > d and regions['rleft'] > d:
        state_description = 'case 6 - front and fleft'
        change_state(2) #detecta front y front left 
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d and regions['rright'] > d and regions['rleft'] > d:
        state_description = 'case 7 - front and fleft and fright'
        change_state(1) # detecta todos "encagado " --back pero no esta
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d and regions['rright'] > d and regions['rleft'] > d:
        state_description = 'case 8 - fleft and fright'
        change_state(0) #detecta los laterales pero fron libre
        

    elif regions['front'] > d and regions['fleft'] > d and regions['fright'] > d and regions['rright'] < d and regions['rleft'] > d:
        state_description = 'case 9 - rright'
        change_state(2)
    elif regions['front'] > d and regions['fleft'] > d and regions['fright'] > d and regions['rright'] > d and regions['rleft'] < d:
        state_description = 'case 10 - rfleft '
        change_state(1)

    elif regions['front'] > d and regions['left'] < d :
            state_description = 'case 11 - fleft and fright'
            change_state(3) #detecta lateral mas margen rleft
    elif regions['front'] > d and regions['right'] < d :
            state_description = 'case 12 - fleft and fright'
            change_state(3) #detecta lateral mas margen rleft

    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)


def find_wall():
    global flag1
    msg = Twist()
    if flag1==1:
        msg.angular.z=-0.5
        msg.linear.x = 0.2
    else:
        msg.linear.x = 0.2
        msg.angular.z = 0.01
    return msg


def turn_left():
    msg = Twist()
    msg.angular.z = 0.3
    return msg

def turn_right():
    global regions_
    regions = regions_

    msg = Twist()
    msg.angular.z = -0.3
    '''if regions['fright'] > 1.5:
        msg.linear.z = -0.3'''
    print("correcion")
    return msg


def follow_the_wall():
    global regions_
    global flag1
    regions = regions_
    flag1=1
    msg = Twist()
    msg.linear.x = 0.1
    print("follow wall")
    if regions['fright'] <0.15: #0.3
        msg.angular.z = -0.3
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
            msg = follow_the_wall()
            pass
        else:
            rospy.logerr('Unknown state!')

        pub_.publish(msg)

        rate.sleep()


if __name__ == '__main__':
    main()