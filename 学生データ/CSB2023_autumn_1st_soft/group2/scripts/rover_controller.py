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

rotating = False
circle_pos = -1
delay = 0
speedi = 0.15
corner_cnt = 0

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
    ranges = msg.ranges

    global rotating
    global circle_pos
    global delay
    global speedi
    global corner_cnt

    dist = 1000
    disti = -1
    for i in range(140, 240):
        if ranges[i] != 0.0:
            if dist > ranges[i]:
                dist = ranges[i]
                disti = i

    if rotating == True:
        dist = 1000
        disti = -1
        for i in range(0, 120):
            if(ranges[i] != 0.0):
                if(dist > ranges[i]):
                    dist = ranges[i]
                    disti = i
        for i in range(245,len(ranges)):
            if ranges[i] != 0.0:
                if dist > ranges[i]:
                   dist = ranges[i]
                   disti = i
            
        if min(260 - disti + circle_pos, abs(disti - circle_pos)) < 8:
            rotating = False
            speed.angular.z = 0.0
            speed.linear.x = speedi
            delay = 25
            speedi = speedi * 0.9
            if speedi < 0.03:
                speedi = 0
                exit()
    elif delay <= 5 and dist < 0.2:
        rotating = True
        speed.linear.x = 0.0
        if disti >= 200 and disti <= 210:
            circle_pos = 40
            speed.angular.z = -0.349
        elif disti >= 210:
            circle_pos = 250 - disti - 10
            speed.angular.z = -0.349
        else:
            circle_pos = 120 - (disti - 120)
            speed.angular.z = 0.349

    delay = max(0, delay - 1)



def controller():
    global speed
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
    speed.linear.x = 0.1
    rover_controller()
