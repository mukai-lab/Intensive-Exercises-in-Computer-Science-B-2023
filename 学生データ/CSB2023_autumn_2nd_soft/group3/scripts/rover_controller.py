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

#def turn_right():
 #   speed.angular.z = 0.15

#def turn_left():
 #   speed.angular.z = -0.15

#def turn_back():
 #   speed.angular.z = 0.3

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
    #print("odo", odom_x, odom_y)
    
def callback_laser(msg):
    global ranges
    ranges = msg.ranges
    center = ranges[175]
    if (center <= 0.25):
        speed.linear.x = 0.0
        right = ranges[140]
        left = ranges[220]
        frontleft = ranges[200]
        frontright = ranges[165]
        print('{:.3f}'.format(left), '{:.3f}'.format(center), '{:.3f}'.format(right))
        if right >= 0.22:
            speed.angular.z = -0.3
        elif left >= 0.22:
            speed.angular.z = 0.3
        elif frontleft >= 0.22:
            speed.angular.z = 0.3
        elif frontright >=0.22:
            speed.angular.z = -0.3

    elif(center > 0.35):
        speed.angular.z = 0
        speed.linear.x = 0.075

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
    rover_controller()
