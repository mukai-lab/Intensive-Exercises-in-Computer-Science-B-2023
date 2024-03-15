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
    global speed
    speed.linear.x = 0.10
    speed.angular.z = 0
    """
    ranges = msg.ranges
    min_num = 1e9
    min_index = -1
    for index,list in enumerate(ranges):
        if(abs(ranges[index] - 0.25) < min_num):
            min_index = index
            min_num = abs(ranges[index] - 0.25)
    print(min_index,ranges[min_index])
    if(abs(ranges[min_index] - 0.25) > 0.01):
        if(ranges[min_index] > 0.25):
            speed.linear.x = 0.05
            if(204 > min_index):
                speed.angular.z = -abs(204 - min_index)/100
            else:
                speed.angular.z = abs(204 - min_index)/100
        elif(ranges[min_index] != 0):
            speed.linear.x = -0.05
            if(204 > min_index):
                speed.angular.z = -abs(204 - min_index)/100
            else:
                speed.angular.z = abs(204 - min_index)/100
        else:
            speed.linear.x = 0
            speed.angular.z = 0
    #for index,list in enumerate(ranges):
    #   print(index,list)
    """



def controller():
    global speed
    #speed.linear.x = 0.001
    #speed.angular.z = 0
    #if odom_x > 0.1:
    #    speed.linear.x = 0
    #print(odom_x)
    


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
