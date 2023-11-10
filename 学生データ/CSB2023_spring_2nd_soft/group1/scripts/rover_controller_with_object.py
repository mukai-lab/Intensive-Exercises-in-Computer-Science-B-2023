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
        #min_i = 0
        #for i in range(len(ranges)):
        #    if(ranges[i] <= 0.2):
        #        min_i = i
        #speed.linear.x = 0.05
        #if(min_i >= 160 and min_i <= 200):
        #    speed.angular.z = 1.0
        #elif(min_i >= 0 and min_i <= 50):
        #    speed.angular.z = 1.0
        #elif(min_i>= 220 and min_i <= 270): 
        #    speed.angular.z = -1.0
        
        r1=ranges[160]
        r2=ranges[180]
        r3=ranges[190]
        r4=ranges[210]
        
        w1_1=0.05
        w1_2=0.01
        w1_3=0.02
        w1_4=0.04

        w2_1 = -3.0
        w2_2 = -1.0
        w2_3 = 1.0
        w2_4 = 3.0

        speed.linear.x = r1*w1_1 + r2*w1_2 + r3*w1_3 + r4*w1_4
        speed.angular.z = r1*w2_1 + r2*w2_2 + r3*w2_3 + r4*w2_4
        print(speed.linear.x, speed.angular.z)


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
