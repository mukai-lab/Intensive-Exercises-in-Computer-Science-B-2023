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


def callback_laser(msg):
        global ranges
        ranges = msg.ranges

        speed.linear.x = 0.03
        speed.angular.z = 0.00

        center_index = 195

        i_0 = 170
        i_1 = 190
        i_2 = 200
        i_3 = 220

        w_0_0 = 0.01
        w_1_0 = 0.03
        w_2_0 = 0.03
        w_3_0 = 0.01

        speed.linear.x = w_0_0 * ranges[i_0] + w_1_0 * ranges[i_1] +  w_2_0 * ranges[i_2]+ w_3_0 * ranges[i_3]

        w_0_1 = -1
        w_1_1 = -0.5
        w_2_1 = 0.5
        w_3_1 = 1

        speed.angular.z =  w_0_1 * ranges[i_0] + w_1_1 * ranges[i_1] +  w_2_1 * ranges[i_2]+ w_3_1 * ranges[i_3]
        
       # if ranges[i_0] <= 0.30:
       #         speed.angular.z = 0.349
       # elif ranges[i_3] <= 0.30:
       #         speed.angular.z = -0.349
       # elif ranges[i_1] <=0.30 or ranges[i_2] <= 0.30:
       #         speed.linear.x = 0.01
       #         speed.angular.z = 0.349
       # else:
       #         speed.linear.x = 0.03
       #         speed.angular.z = 0.00

def pos_controller():
        global speed



      

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
