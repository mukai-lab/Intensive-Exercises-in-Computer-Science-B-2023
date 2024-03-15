#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan

_odom_x, _odom_y, _odom_theta = 0.0, 0.0, 0.0
ranges = []
speed = Twist()

def callback_odom(msg):
        global _odom_x, _odom_y, _odon_theta
        _odom_x = msg.pose.pose.position.x
        _odom_y = msg.pose.pose.position.y
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        q = (qx, qy, qz, qw)
        e = euler_from_quaternion(q)
        _odom_theta = e[2]

def callback_laser(msg):
        global ranges
        global speed
        global r1,r2,r3,r4
        global w0, w1, w2, w3, w4, w5, w6, w7
        ranges = msg.ranges

        speed.linear.x = 0.05
        speed.angular.z = 0.0

        r1 = ranges[151]
        r2 = ranges[187]
        r3 = ranges[213]
        r4 = ranges[249]
        
        w0 = 0.02
        w1 = 0.03
        w2 = 0.03
        w3 = 0.02

        w4 = 0.1
        w5 = 0.5
        w6 = -0.5
        w7 = -0.1
        
        speed.linear.x = r1*w0 + r2*w1 + r3*w2 + r4*w3

        speed.angular.z = (1/r1)*w4 + (1/r2)*w5 + (1/r3)*w6 + (1/r4)*w7
        if speed.angular.z > 0.349:
           speed.angular.z = 0.349
        if speed.angular.z < -0.349:
           speed.angular.z = -0.349
      

def pos_controller():
        global speed
        


def rover_controller():
        global speed

        rospy.init_node('rover_controller', anonymous=True)

        pub = rospy.Publisher('rover_drive',Twist, queue_size=1)

        rate = rospy.Rate(5)

        odom_subscriber = rospy.Subscriber('odom', Odometry, callback_odom)

        lidar_subscriber = rospy.Subscriber('scan', LaserScan, callback_laser)

        while not rospy.is_shutdown():
                pos_controller()
                pub.publish(speed)
                rate.sleep()

if __name__ == '__main__':
        rover_controller()
