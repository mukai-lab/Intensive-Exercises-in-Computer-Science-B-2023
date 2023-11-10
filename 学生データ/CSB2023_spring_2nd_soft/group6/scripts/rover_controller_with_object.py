#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
import numpy as np
import math

_odom_x, _odom_y, _odom_theta = 0.0, 0.0, 0.0
ranges = []
speed = Twist()
Time = time.time()
turn_mode = -1

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
        global Time
        global turn_mode
        ranges = msg.ranges
        speed.linear.x = 0.1

        count = 0
        min = 100
        rad = 0
        for i in ranges:
            if(i < min and i != 0):
                min = i
                rad = count
            count = count + 1

        if(turn_mode == 1 and 60 < rad and rad < 180 and Time + 11.0 < time.time()):
            turn_mode = turn_mode * -1
            Time = time.time()
        if(turn_mode == -1 and ((0 < rad and rad < 90) or 180 < rad) and Time + 11.0 < time.time()):
            turn_mode = turn_mode * -1
            Time = time.time()
        if(abs(ranges[120] - ranges[0]) < 0.15 and ranges[120] != 0 and ranges[0] != 0 and Time + 11.0 < time.time()):
            turn_mode = turn_mode * -1
            Time = time.time()

        if(turn_mode == 1):
            if(10 < rad and rad < 60):
                speed.angular.z = 1
                if(min > 0.13):
                    speed.angular.z += 0.35
            elif(180 < rad and rad < 230):
                speed.angular.z = 0
            else:
                speed.angular.z = 0.7
                if(min > 0.13):
                    speed.angular.z += 0.35

        if(turn_mode == -1):
            if(60 < rad and rad < 110):
                speed.angular.z = -1
                if(min > 0.13):
                    speed.angular.z -= 0.35
            elif(130 < rad and rad <= 180):
                speed.angular.z = 0
            else:
                speed.angular.z = -0.7
                if(min > 0.13):
                    speed.angular.z -= 0.35


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
