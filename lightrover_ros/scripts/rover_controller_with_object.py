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
        global  speed
        ranges = msg.ranges
        
        #Q3-5
        '''
        if (0 < ranges[210] < 0.30):
            speed.linear.x = 0.0
            speed.angular.z = 0.5
        elif (0 < ranges[190] < 0.30):
            speed.linear.x = 0.0
            speed.angular.z = 0.5
        elif (0 < ranges[240] < 0.30):
            speed.linear.x = 0.1
            speed.angular.z = -0.5
        elif (0 < ranges[160] < 0.30):
            speed.linear.x = 0.1
            speed.angular.z = 0.5
        else:
            speed.linear.x = 0.15
            speed.angular.z = 0.0
        '''
        
        #Q3-6
        '''
        x_w = [0.02, 0.02, 0.02, 0.02]
        speed.linear.x = x_w[0] * ranges[160] + x_w[1] * ranges[190] + x_w[2] * ranges[210] + x_w[3] * ranges[240]
        z_w = [-0.6, -0.6, 0.6, 0.6]
        speed.angular.z = z_w[0] * ranges[160] + z_w[1] * ranges[190] + z_w[2] * ranges[210] + z_w[3] * ranges[240]
        '''

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
