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
        ranges = msg.ranges

        r1 = msg.ranges[175]
        r2 = msg.ranges[160]
        r3 = msg.ranges[185]
        r4 = msg.ranges[200]
        print(r1, r2, r3, r4)
        

        we = [0.1, 0.2, 0.2, 0.1]
        sp = [0.3, 0.9, -0.6, -0.3]

        lin = (r1*we[0] + r2*we[1] + r3*we[2] + r4*we[3])*0.1
        print(lin)
        if (lin < 0.0):
            speed.linear.x = -0.05
        else:
            speed.linear.x = lin
        speed.angular.z = -(r1*sp[0] + r2*sp[1] + r3*sp[2] + r4*sp[3])


              
         


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
