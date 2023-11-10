#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
import math
import random
odom_x, odom_y, odom_theta = 0.0, 0.0, 0.0
ranges = []
speed = Twist()
move_pattern=1

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
    global move_pattern
    ranges = msg.ranges
    m = 1
    min_i = 0
    for i , x in enumerate(ranges):
        if(x != 0.0):
            if(m > x):
                m = x
                min_i = i
    if(move_pattern==1):
        #ranges=[i for i in ranges_old if not i==0.0]
        #print(random.uniform(1,100))
        if(int(random.uniform(1,100))%20==0):
            move_pattern=3
        print(ranges.index(min(ranges)))
        print(min(ranges))
        if(ranges[min_i] >= 0.7):
            speed.linear.x = 0.0
            speed.angular.z = 0.0
        elif(ranges[min_i] <= 0.25 and ranges[min_i] >= 0.249):
            speed.linear.x = 0.0
            speed.angular.z = 0.0
            print(int(random.uniform(1,100)))
            if(int(random.uniform(1,100))%4==0):
                move_pattern=4
            elif(int(random.uniform(1,100))%4==1):
                move_pattern=6
            else:
                move_pattern=2
        elif(ranges[min_i] <= 0.25):
            speed.linear.x = -0.06
            speed.angular.z = (min_i-205)*3.14/270
        else:
            speed.linear.x = 0.06
            speed.angular.z = (min_i-205)*3.14/270
    elif(move_pattern==2):
        print("2")
        speed.linear.x=0
        speed.angular.z =1.8
        if(ranges[min_i]>=0.4):
            print(ranges[min_i])
            print(min_i)
            move_pattern=1
    elif(move_pattern==3):
        print("3")
        if(ranges[min_i] <= 0.25):
            speed.linear.x = 0.0
            speed.angular.z = 0.0
            move_pattern=2
        else:
            speed.linear.x=(random.uniform(0.05,0.1))
            speed.angular.z = (min_i-205)*3.14/270
        if(int(random.uniform(1,100))%20==0):
            move_pattern=1
    elif(move_pattern==4):
        print("4")
        speed.linear.x=-0.2
        speed.angular.z = 0.0
        if(int(random.uniform(1,100))%10==0):
            move_pattern=1
        else:
            move_pattern=5
    elif(move_pattern==5):
        print("5")
        speed.linear.x=0.2
        speed.angular.z =0.0
        if(int(random.uniform(1,100))%10==0):
            move_pattern=1
        else:
            move_pattern=4
    elif(move_pattern==6):
         print("6")
         speed.angular.z = 1.57
         move_pattern=7
    elif(move_pattern==7):
        print("7")
        speed.linear.x=0.2
        speed.angular.z =0.0
        if(int(random.uniform(1,100))%10==0):
            move_pattern=1
        else:
            move_pattern=8
    elif(move_pattern==8):
        print("8")
        speed.linear.x=-0.2
        speed.angular.z =0.0
        if(int(random.uniform(1,100))%10==0):
            move_pattern=1
        else:
            move_pattern=7
            '''
        speed.linear.x=0.0
        print(ranges[138])
        if(ranges[138] <= 0.25):
            speed.angular.z = 0.0
            speed.linear.x=0.05
        else:
            speed.angular.z = 0.5
'''

def controller():
    global speed
    #if(odom_x > 0.1):
        #speed.linear.x = 0.0
        #speed.angular.z = 0.0
    #else:
        #speed.linear.x = 0.025
    #speed.linear.x = 0.1






def rover_controller():
    global speed

    rospy.init_node('rover_controller', anonymous=True)

    pub = rospy.Publisher('rover_drive',Twist, queue_size=1)

    rate = rospy.Rate(5)
        
    odom_subscriber = rospy.Subscriber('odom', Odometry, callback_odom)
        
    lidar_subscriber = rospy.Subscriber('scan', LaserScan, callback_laser)

    while not rospy.is_shutdown():
        rate.sleep()
        controller()
        pub.publish(speed)

if __name__ == '__main__':
    rover_controller()
