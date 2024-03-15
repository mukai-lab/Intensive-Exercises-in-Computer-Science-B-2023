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
init_x, init_y = 0.0, 0.0
flag = True
def callback_odom(msg):
    global odom_x, odom_y, odom_theta, init_x, init_y
    odom_x = msg.pose.pose.position.x
    odom_y = msg.pose.pose.position.y
    if not init_x or not init_y:
        init_x = odom_x
        init_y = odom_y
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
    


def controller():
    global speed, flag
    if odom_x < 0.1:
        speed.linear.x = 0.1
        print('odom', odom_x, odom_y, odom_theta)
    else:
        speed.linear.x = 0
        if flag:
            print('init', init_x, init_y)
            flag = False


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
