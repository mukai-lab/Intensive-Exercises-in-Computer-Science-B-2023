1#!/usr/bin/env python3
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
    n=5000
    p=0
    ranges = msg.ranges
    speed.linear.x=0.05
    '''
    if(ranges[200]<0.24):
        speed.linear.x=-0.05
    if(ranges[200]>0.26):
        speed.linear.x=0.05
    if(ranges[200]<=0.26 and ranges[200]>=0.24):
        speed.linear.x=0
    
    '''
'''
    for i in range (0,len(ranges)):
        if(ranges[i]<=n and ranges[i] != 0):
            n=ranges[i]
            p=i   
            
    if(ranges[p]<0.24):
        speed.linear.x=-0.05
    if(ranges[p]>0.26):
        speed.linear.x=0.05
    if(ranges[200]<=0.26 and ranges[200]>=0.24 or ranges[p]>=0.5):
        speed.linear.x=0
                 
    if(p<210):
        speed.angular.z=-0.2
    if(p>190):
        speed.angular.z=0.2
    if(p<=190 and p>=210 or ranges[p]>=0.5):
        speed.angular.z=0
'''


def controller():
    global speed
'''    
    speed.linear.x =0.1
    speed.angular.z=0
    if (odom_x >= 0.1):
        speed.linear.x =0
'''   
    


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
