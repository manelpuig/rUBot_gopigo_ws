#!/usr/bin/env python
 
import rospy
import numpy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class WallFollower:

    def __init__(self):

        rospy.init_node("wall_follower_controller", anonymous=False)

        self._kp = rospy.get_param("~kp", 1)
        self._noWallDistance = rospy.get_param("~no_wall_distance", 0.3)
        self._distanceReference = rospy.get_param("~distance_reference", 0.2)
        self._lookaheadDistance = rospy.get_param("~lookahead_distance", 0.2)
        self._theta = rospy.get_param("~theta", 20.0)
        self._forwardSpeed = rospy.get_param("~forward_speed", 0.001)

        self._error_z = 0
        self._error_z_1 = 0
	self._futureWallDistance = 0
        self._msg = Twist()
        self._msg.linear.x = 0
        self._msg.angular.z = 0

        self._a = 0
        self._b = 0
        self._state = 0

        self._cmdVel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        rospy.Subscriber("/scan", LaserScan, self.callbackLaser)

        rospy.on_shutdown(self.shutdown)

    def start(self):

        rospy.spin()

    def callbackLaser(self, scan):

        correctionFactor = int(len(scan.ranges) / 360)

        a = scan.ranges[int((-90 + self.__unWrapAngle(self._theta)) * correctionFactor)]
        b = scan.ranges[self.__unWrapAngle(-90) * correctionFactor]

        if numpy.isfinite(a) and 0.1 <= a <= 16: self._a = a
        if numpy.isfinite(b): self._b = numpy.clip(b, 0.1, 16) 

        theta = numpy.deg2rad(self._theta)

        a = self._a
        b = self._b

        if self._a > 0.8 and self._state == 0: self._state = 1
        elif self._b > 0.5 and self._state == 1: self._state = 2
        elif self._a < 0.3 and self._state == 2: self._state = 3
        elif self._b < 0.5 and self._state ==  3: self._state = 0


        if self._state == 3:
            rospy.loginfo("A a: %.2f, b: %.2f", self._a, self._b)
            self._msg.angular.z = 0
            self._cmdVel.publish(self._msg)
	elif self._state == 2 :
            # Giramos a la derecha con un radio igual a self._distanceReference
            rospy.loginfo("G a: %.2f, b: %.2f", self._a, self._b)
            self._msg.linear.x = self._forwardSpeed 
            self._msg.angular.z = -self._forwardSpeed / (self._distanceReference)
            self._cmdVel.publish(self._msg) 
	elif self._state == 1 :
            # Seguimos recto            rospy.loginfo("Girando a: %.2f, b: %.2f", a, b)
            rospy.loginfo("S a: %.2f, b: %.2f", self._a, self._b)
            self._cmdVel.publish(self._msg)
        elif self._state == 0:
            rospy.loginfo("W a: %.2f, b: %.2f", self._a, self._b)

            # rospy.loginfo("%.1f, %.1f", a, b)

            alpha =numpy.clip(numpy.arctan((a * numpy.cos(theta) - b) / (a * numpy.sin(theta))),-numpy.pi/2, numpy.pi/2)

            # rospy.loginfo("Alpha angle: %5.1f degrees.", numpy.rad2deg(alpha))

            wallDistance = b * numpy.cos(alpha)

            # rospy.loginfo("Wall distance: %5.2f m.", wallDistance)

	
            futureWallDistanceNew = wallDistance + self._lookaheadDistance * numpy.sin(alpha)
            if (futureWallDistanceNew>=0 and numpy.isfinite(futureWallDistanceNew)) : self._futureWallDistance =futureWallDistanceNew
            futureWallDistance = self._futureWallDistance

            self._error_z = futureWallDistance - self._distanceReference

#            rospy.loginfo("Future wall distance: %5.2f m, error: %5.2f m, angle: %5.2f rad", futureWallDistance, self._error_z, alpha)

            angVel = -self._kp * self._error_z
            if numpy.isfinite(angVel): self._msg.angular.z = angVel

       
            if abs(self._msg.angular.z) > 1:
                self._msg.linear.x = 0
            elif abs(self._msg.angular.z) > 0.03:
                self._msg.linear.x = 0.5 * self._forwardSpeed
            else:
                self._msg.linear.x = self._forwardSpeed

            self._error_z_1 = self._error_z

            self._cmdVel.publish(self._msg)

    def __sign(self, val):

        if val >= 0:
            return 1
        else:
            return -1

    def __wrapAngle(self, angle):
        if 0 <= angle <= 180:
            return angle
        else:
            return angle - 360

    def __unWrapAngle(self, angle):
        if angle < 0:
            return angle + 360
        else:
            return angle

    def shutdown(self):
        self._msg.linear.x = 0
        self._msg.angular.z = 0
        self._cmdVel.publish(self._msg)

if __name__ == '__main__':
    try:
        wf = WallFollower()
        wf.start()
    except rospy.ROSInterruptException: pass
