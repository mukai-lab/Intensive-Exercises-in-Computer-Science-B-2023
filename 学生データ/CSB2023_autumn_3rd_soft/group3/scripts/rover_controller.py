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
    global flag
    ranges = msg.ranges
    speed.linear.x = 0.05
    speed.angular.z = 0.00
    
    
    if(ranges[215] == 0 or ranges[215] >= 0.3):
        speed.linear.x = 0.05
        speed.angular.z = -0.27
    '''
    global ranges
    ranges = msg.ranges
    global speed
    speed.linear.x = 0.00
    speed.angular.z = 0.00
    num = 1000
    idx = 0

    global ranges
    ranges = msg.ranges
    global speed
    speed.linear.x = 0.00
    speed.angular.z = 0.00
    for range in ranges:
        if range > 0 and range < 0.25:
            print (ranges. index (range), range)

    if  ranges[idx] < 0.5 and ranges[idx] > 0 and ranges[idx] > 0.25:
        speed.linear.x = 0.03
        speed.angular.z = 0.01 * (idx-180)
    elif ranges[idx] == 0.25:
        speed.linear.x = 0.00
        speed.angular.z = 0.00
    elif  ranges[idx] < 0.25:
        speed.linear.x = -0.03
        speed.angular.z = 0.01 * (idx-180)

'''
        
    

    
          
          

def controller():

    global speed
    '''
    speed.linear.x = 0.05
    speed.angular.z = 0.00

    if odom_x >= 0.05 :
        speed.linear.x = 0.01
    if odom_x >= 0.10:
        speed.linear.x = 0.00
        print(odom_x)'''
    global ranges
    global flag
    #ranges = msg.ranges
    speed.linear.x = 0.05
    speed.angular.z = 0.00
    
    
    if(ranges[215] == 0 or ranges[215] >= 0.3):
        speed.linear.x =0.05
        speed.angular.z = -0.36
    #print(speed.linear.x)
    


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
