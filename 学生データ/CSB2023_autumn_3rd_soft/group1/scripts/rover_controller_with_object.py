#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan

_odom_x, _odom_y, _odom_theta = 0.0, 0.0, 0.0
ranges = [100 for _ in range(250)]
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
        ranges = msg.ranges


def pos_controller():
        global speed, ranges
        l = [ranges[162],ranges[174],ranges[186],ranges[198]]
        print(l)
        w = [0.01, 0.02, 0.02, 0.01, -1.3, -1.1, 1.1, 1.3]
        speed.linear.x,speed.angular.z=0,0
        for i in range(4):
                speed.linear.x+=l[i]*w[i]
                speed.angular.z+=l[i]*w[i+4]
        speed.linear.x=min(speed.linear.x,0.15)
        speed.angular.z=0.35 if speed.angular.z>0.35 else -0.35 if speed.angular.z<-0.35 else speed.angular.z


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
