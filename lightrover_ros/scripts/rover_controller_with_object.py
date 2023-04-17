#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan

_odom_x, _odom_y, _odom_theta = 0.0, 0.0, 0.0
ranges = []
speed = Twist()

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
        rospy.loginfo("Odomery: x=%s y=%s theta=%s", _odom_x, _odom_y, _odom_theta)
    
    
def callback_laser(msg):
        global ranges
        ranges = msg.ranges

        #if ranges[180] > 0.25:
        #   speed.linear.x = 0.1
        #elif ranges[180] < 0.25:
        #   speed.linear.x = -0.1
        #else:
        #   speed.linear.x = 0.01
        #speed.angular.z = 0.0
        
        
        if (ranges[230] < 0.40 and ranges[230] > 0.0) or (ranges[220] < 0.40 and ranges[220] > 0.0):
           speed.linear.x = 0.05
           speed.angular.z = -0.3
        elif (ranges[180] < 0.40 and ranges[180] > 0.0) or (ranges[190] < 0.40 and ranges[190] > 0.0):
           speed.linear.x = 0.05
           speed.angular.z = 0.3
        else:
           speed.linear.x = 0.1
           speed.angular.z = 0.0
           
        
        
        #w1 = 0.02
        #w2 = 0.02
        #w3 = 0.02
        #w4 = 0.02
        #w5 = 0.4
        #w6 = 0.4
        #w7 = -0.4
        #w8 = -0.4

        #speed.linear.x = ranges[230]*w1 + ranges[220]*w2 + ranges[190]*w3 + ranges[180]*w4
        #speed.angular.z = ranges[230]*w5 + ranges[220]*w6 + ranges[190]*w7 + ranges[180]*w8



def pos_controller():
        global speed
        
        #if _odom_x < 0.1:
        #   speed.linear.x = 0.1
        #else:
        #   speed.linear.x = 0.0
        #speed.angular.z = 0.0
        

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
