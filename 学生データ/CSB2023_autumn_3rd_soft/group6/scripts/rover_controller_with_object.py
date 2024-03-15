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

        #for i in range(0,230,10):
        #        print(i, " ", ranges[i])
        w_z=[0.1,0.1,0.1,0.1]
        w_x=[0.01,0.01,0.01,0.01]
        f = 200
        o = 20
        i = 10
        a=0.25
        
        speed.angular.z = a/(ranges[f-o]+a) * w_z[0] + a/(ranges[f-i]+a) * w_z[1] + a/(ranges[f+i]+a) * w_z[2] + a/(ranges[f+o]+a) * w_z[3]
        speed.linear.x = ranges[f-o] * w_x[0] + ranges[f-i] * w_x[1] + ranges[f+i] * w_x[2]  + ranges[f+o] * w_x[3]

        if(speed.linear.x > 0.2):
                speed.linear.x = 0.2
        if(speed.angular.z > 0.4):
                speed.angular.z = 0.4
        if(speed.angular.z <-0.4):
                speed.angular.z = -0.4

        print("x speed: ",speed.linear.x, ",  z speed: ",speed.angular.z)

        #if(ranges[150] != 0.0) and (ranges[150] < 0.3):
        #        speed.angular.z =0.24
        #elif(ranges[170] != 0.0) and (ranges[170] < 0.3):
        #        speed.angular.z = 0.40
        #elif(ranges[190] != 0.0) and (ranges[190] < 0.3):
        #        speed.angular.z =- 0.40
        #elif(ranges[210] != 0.0) and(ranges[210] < 0.3):
        #        speed.angular.z =-0.24
        #else:
        #        speed.angular.z = 0
        


def pos_controller():
        global speed
        #speed.linear.x = 0.05
        


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
