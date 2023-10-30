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

init_x = 0
init_y = 0
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

    #Q1-2
    '''
    print(ranges[front])
    '''

    #Q1-3
    '''
    if ranges[front] > 0.25:
        speed.linear.x = 0.1
    else:
        speed.linear.x = 0.0
    '''

    #Q1-5
    '''
    if ranges[front] > 0.25:
        speed.linear.x = 0.1
    else:
        speed.linear.x = -0.1
    '''

    #Q1-6
    '''
    min_dis = 100
    min_idx = 0
    for i, dis in enumerate(ranges):
        if dis != 0 and dis < min_dis:
            min_dis = dis
            min_idx = i
    if min_dis < 0.25:
        speed.linear.x = -0.1
    else:
        speed.linear.x = 0.1
    if min_idx < front:
        speed.angular.z = -1.0
    else:
        speed.angular.z = 1.0
    '''

def controller():
    global speed

    #Q1-1
    '''
    if odom_x < 0.1:
        speed.linear.x = 0.1
        print('odom_x :', odom_x, 'odom_y :',odom_y, 'odom_theta :',odom_theta)
        print('初期値x :', init_x, '初期値y :',init_y)
    else:
        speed.linear.x = 0.0
    speed.angular.z = 0
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
