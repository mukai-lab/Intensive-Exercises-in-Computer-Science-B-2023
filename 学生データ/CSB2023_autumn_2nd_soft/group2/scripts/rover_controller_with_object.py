#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
import time

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

def callback_laser(msg):
        global ranges
        ranges = msg.ranges
        


def pos_controller():
        if ranges:
                l = ranges[220]
                cl = ranges[210]
                cr = ranges[190]
                r = ranges[180]
                l_ratio_z = -0.6 * max(0.4-l,0)
                cl_ratio_z = -1.4 * max(0.4-cl,0)
                cr_ratio_z = 0.7 * max(0.4-cr,0)
                r_ratio_z = 0.3 * max(0.4-r,0)

                l_ratio_x = 0.7 * (max(0.4-l,0)/0.4)
                cl_ratio_x = 0.7 * (max(0.4-cl,0)/0.4)
                cr_ratio_x = 0.7 * (max(0.4-cr,0)/0.4)
                r_ratio_x = 0.7 * (max(0.4-r,0)/0.4)
                
                speed.linear.x = 0.1 - 0.1 * (l_ratio_x + cl_ratio_x + cr_ratio_x + r_ratio_x)
                speed.angular.z = l_ratio_z + cl_ratio_z + cr_ratio_z + r_ratio_z
                if abs(speed.linear.x < 0.01) and abs(speed.angular.z < 0.02):
                        speed.linear.x = 0.0
                        speed.angular.z = -0.3
                        time.sleep(1)
                        
                print(speed.linear.x,speed.angular.z)
                """
                print(l != 0.0 and l < 0.3,cl != 0.0 and cl < 0.3,cr != 0.0 and cr < 0.3, r != 0.0 and r < 0.3)
                if (l != 0.0 and l < 0.3) and (cl != 0.0 and cl < 0.3) and (cr != 0.0 and cr < 0.3) and (r != 0.0 and r < 0.3):
                        speed.linear.x = 0.00
                        speed.angular.z = -0.3
                elif (l != 0.0 and l < 0.3) and (cl != 0.0 and cl < 0.3) and (cr != 0.0 and cr < 0.3):
                         speed.linear.x = 0.00
                         speed.angular.z = -0.3
                elif (cl != 0.0 and cl < 0.3) and (cr != 0.0 and cr < 0.3) and (r != 0.0 and r < 0.3):
                        speed.linear.x = 0.00
                        speed.angular.z = -0.3
                elif (l != 0.0 and l < 0.3) and (cr != 0.0 and cr < 0.3) and (r != 0.0 and r < 0.3):
                        speed.linear.x = 0.02
                        speed.angular.z = -0.3
                elif (l != 0.0 and l < 0.3) and (cl != 0.0 and cl < 0.3) and (r != 0.0 and r < 0.3):
                        speed.linear.x = 0.02
                        speed.angular.z = -0.3
                elif (l != 0.0 and l < 0.3) and (cl != 0.0 and cl < 0.3):
                        speed.linear.x = 0.00
                        speed.angular.z = -0.3
                elif (cl != 0.0 and cl < 0.3) and (cr != 0.0 and cr < 0.3):
                        speed.linear.x = 0.00
                        speed.angular.z = -0.3
                elif (cr != 0.0 and cr < 0.3) and (r != 0.0 and r < 0.3):
                        speed.linear.x = 0.00
                        speed.angular.z = 0.3
                elif (l != 0.0 and l < 0.3) and (r != 0.0 and r < 0.3):
                        speed.linear.x = 0.00
                        speed.angular.z = -0.3
                elif l != 0.0 and l < 0.3:
                        if l < 0.15:
                                speed.linear.x = 0.00
                                speed.angular.z = -0.2
                        else:
                                speed.linear.x = 0.00
                                speed.angular.z = -0.2
                elif cl != 0.0 and cl < 0.3:
                        if cl < 0.15:
                                speed.linear.x = 0.00
                                speed.angular.z = -0.3
                        else:
                               speed.linear.x = 0.00
                               speed.angular.z = -0.3 
                elif cr != 0.0 and cr < 0.3:
                        if cr < 0.15:
                                speed.linear.x = 0.00
                                speed.angular.z = 0.3
                        else:
                               speed.linear.x = 0.00
                               speed.angular.z = 0.3 
                elif r != 0.0 and r < 0.3:
                        if r < 0.15:
                                speed.linear.x = 0.00
                                speed.angular.z = 0.2
                        else:
                                speed.linear.x = 0.05
                                speed.angular.z = 0.2
                                
                else:
                        speed.linear.x = 0.05
                        speed.angular.z = 0.0
        else:
                speed.linear.x = 0.05
                speed.angular.z = 0.0
        
"""

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
