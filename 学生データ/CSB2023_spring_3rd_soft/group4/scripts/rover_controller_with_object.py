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
        
        min = 100
        index = 0
        ranges = msg.ranges
        print(ranges[205])
        speed.linear.x = 0.045
        for i in range(156, 235, 26):
            if ranges[i] > 0 and min > ranges[i]:
                min = ranges[i]
                index = i
        print(index)
        if index == 156 and min < 0.25:
            speed.angular.z = 0.7
        elif index == 182 and min < 0.25:
            speed.angular.z = 1.1
        elif index == 208 and min < 0.25:
            speed.angular.z = -1.1
        elif index == 234 and min < 0.25:
            speed.angular.z = -0.7
        else:
            speed.angular.z = 0.0

"""
        range2 = [ranges[156], ranges[182], ranges[208], ranges[234]]
        #w = [[-1.75, -1.2, 0.95, 1.5],[0.06, 0.03, 0.03, 0.06]]
        w = [[-1.2, -1.75, 1.75, 1.2],[0.03, 0.015, 0.015, 0.03]]
        result = [0.01, 0]
        for i in range(0, 2):
            for j in range(0, 4):
                result[i] = result[i] + w[i][j] * range2[j]
        speed.angular.z = result[0]
        """
"""
        if result[1] > 1:
            speed.linear.x = 0
        else:
"""
"""
        speed.linear.x = result[1]
        print("angle")
        print(result[0])
        print("linear")
        print(result[1])





        
"""
"""
        if ranges[162] > 0 and min > ranges[162]:
            min = ranges[162]
            index = 162
        if ranges[194] > 0 and min > ranges[194]:
            min = ranges[194]
            index = 194
        if ranges[216] > 0 and min > ranges[216]:
            min = ranges[216]
            index = 216
        if ranges[248] > 0 and min > ranges[248]:
            min = ranges[248]
            index = 248

        if min < 0.15:
            if index == 162:
                speed.angular.z = 0.5
            if index == 194:
                speed.angular.z = 0.9
            if index == 216:
                speed.angular.z = -0.9
            if index == 248:
                speed.angular.z = -0.5
"""
"""
        if ranges[162] < 0.15:
                speed.angular.z = 0.5
        elif ranges[194] < 0.15:
                speed.angular.z = 0.9
        elif ranges[216] < 0.15:
                speed.angular.z = -0.9
        elif ranges[248] < 0.15:
                speed.angular.z = -0.5
        else:
                speed.angular.z = 0.0
"""

def pos_controller():
        global speed

        #speed.linear.x = 0.1
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
