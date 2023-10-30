#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan

odom_x, odom_y, odom_theta = 0.0, 0.0, 0.0
ranges = []
speed = Twist()
length = 0
count = 0
step = 0
wall_right_length = 0
wall_left_length = 0
ms = 0

def callback_odom(msg):
    global odom_x, odom_y, odom_theta 
    odom_x = msg.pose.pose.position.x  
    odom_y = msg.pose.pose.position.y  
    qx = msg.pose.pose.orientation.x
    qy = msg.pose.pose.orientation.y
    qz = msg.pose.pose.orientation.z
    qw = msg.pose.pose.orientation.w
    q = (qx, qy, qz, qw)
    e = euler_from_quaternion(q)
    odom_theta = e[2]
    
def callback_laser(msg):
    global ranges
    global length
    global step
    global wall_right_length
    global wall_left_length
    global ms
    global speed
    ranges = msg.ranges
    print(step)
    if(step == 0):
        wall_right_length = ranges[147]
        wall_left_length = ranges[231]
        print('wall_right_length')
        print(wall_right_length)
        print('wall_left_length')
        print(wall_left_length)
        step = 1
    if(step == 1):
        speed.linear.x = 0.1
        speed.angular.z = 0.0
        if(wall_left_length <= wall_right_length):
            step = 5
        else:
            step = 2
    if(step == 2):
        print(ranges[189])
        if(ranges[189] <= 0.245):
            if(ranges[189] != 0.0):
                speed.linear.x = -0.04
                step = 6
    if(step == 5):
        if(ranges[189] <= 0.245):
            if(ranges[189] != 0.0):
                speed.linear.x = -0.04
                step = 7
    if(step == 6):
        if(ranges[147] <= wall_right_length - 0.07):
            if(ranges[147] != 0.0):
                speed.linear.x = -0.04
                speed.angular.z = 0.3
                step = 8
    if(step == 7):
        if(ranges[231] <= wall_left_length - 0.07):
            if(ranges[231] != 0.0):
                speed.linear.x = -0.04
                speed.angular.z = -0.3
                step = 9
    if(step == 8):
        ms += 1
        print(ms)
        if(ms >= 42):
            speed.angular.z = 0.0
            step = 10
            ms = 0
    if(step == 9):
        ms += 1
        print(ms)
        if(ms >= 42):
            speed.angular.z = 0.0
            step = 11
            ms = 0
    if(step == 10):
        if(ranges[98] <= 0.13):
            if(ranges[98] != 0.0):
                speed.angular.z = -0.349
                step = 12
    if(step == 11):
        if(ranges[49] <= 0.13):
            if(ranges[49] != 0.0):
                speed.angular.z = 0.349
                step = 12
    if(step == 12):
        ms += 1
        print(ranges[63])
        if(ranges[63] <= 0.13):
            if(ranges[63] != 0.0):
                speed.linear.x = 0.0
                speed.angular.z = 0.0
                step = 13
        if(ms >= 42):
            speed.angular.z = 0.0
            
                

def controller():
    global speed
    global ranges
    global count
    global length
    #speed.linear.x = 0.05
    #1 speed.angular.z = 0.0
    #1 print(odom_x)
    #1 if odom_x > 0.1:
        #1 speed.linear.x = 0





def rover_controller():
    global speed

    rospy.init_node('rover_controller', anonymous=True)

    pub = rospy.Publisher('rover_drive',Twist, queue_size=1)

    rate = rospy.Rate(5)
        #ロボットの位置測定？
    odom_subscriber = rospy.Subscriber('odom', Odometry, callback_odom)
        
    lidar_subscriber = rospy.Subscriber('scan', LaserScan, callback_laser)

    while not rospy.is_shutdown():
        rate.sleep()
        controller()
        pub.publish(speed)

if __name__ == '__main__':
    rover_controller()
