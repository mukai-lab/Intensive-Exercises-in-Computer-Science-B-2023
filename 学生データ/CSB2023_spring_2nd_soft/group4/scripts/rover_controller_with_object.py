#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pos_controller_by_color

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan

_odom_x, _odom_y, _odom_theta = 0.0, 0.0, 0.0
ranges = []
speed = Twist()

#distance_ahead = 10.0
#min_index = 0
#min_range = 10.0

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
    
range1 = 0.0
range2 = 0.0
range3 = 0.0
range4 = 0.0
def callback_laser(msg):
    global ranges
    #global radiuses
    global range1 
    global range2
    global range3
    global range4
    #global distance_ahead
    #global min_index
    #global min_range
   # global radiuses
    ranges = msg.ranges
    #print('head  ' + str(ranges[205]))
    #print('left  ' + str(ranges[1]))
    #print('right ' + str(ranges[140]))
    #print('teil  ' + str(ranges[80]))
    #for i in range(200,210):
    #    print('ranges at ' + str(i) + ' ' + str(ranges[i]))
    #distance_ahead = ranges[205]
    #min_range = 20.0
    #side_min_range = 10.0
    #ahead_min_range = 15.0
    #for i in range(270):
    #    if (ranges[i] != 0 and ranges[i] < min_range):
    #        min_range = ranges[i]
    #        min_index = i
    
    #radiuses = [ranges[197], ranges[201], ranges[209], ranges[213]]
    range1 = ranges[197]
    range2 = ranges[201]
    range3 = ranges[209]
    range4 = ranges[213]
        
    #for i in range(4):
    #    if min_range > ahead_distances[i]:
    #        min_range = ahead_distances[i]
    #        min_index = i
    
def step_function(x):
    if x < 0.25:
        return 1.0
    else:
        return 0.0
    
def square(x):
    return 1 - x**2 


def pos_controller():
    global speed
    global dir_type
    
   
    global ranges
    global range1
    global range2
    global range3
    global range4
        
    #print('min index is' + str(min_index))
    #print('min range is' + str(min_range))
    #speed.linear.x = (range1 + range2 + range3 + range4) / 30
    speed.linear.x = 0
    #speed.angular.z = 0.0
    
    
    #speed.linear.z = ahead_distances[0] * -1 * np.log10(ahead_distances[0]) + ahead_distances[1] * -1 * np.log10(ahead_distances[1]) + ahead_distances[2] * np.log10(ahead_distances[2]) + ahead_distances[3] * np.log10(ahead_distances[3])
 #   s = (-1.0 * np.log(1+range1*5) + -1.0 * np.log(1+range2*10) + 1.0 * np.log(1+range3*10) + 1.0 * np.log(1+range4*5))
    #s = square(range1) + square(range2) + square(range3) + square(range4)
    #s = np.log(range1+0.5) + np.log(range2+0.5) + np.log(range3+0.5) + np.log(range4+0.5)
    #print(s.dtype)
    #print(s)
    speed.angular.z = 0
  
    #print('step(range1)'+ str(step_function(range1)))
    #print('step(range2)'+ str(step_function(range2)))
    #print('step(range3)'+ str(step_function(range3)))
    #print('step(range4)'+ 
    
    dir_type
    print("dir_type")
    print(dir_type)
    print("color")
    print(pos_controller_by_color.color_value)
    if pos_controller_by_color.color_value == 0 and dir_type == 0: 
        dir_type = 1
        print(pos_controller_by_color.color_value)
        print('goal, 0')
        
    if pos_controller_by_color.color_value == 1 and dir_type == 1: 
        dir_type = 2
        print(pos_controller_by_color.color_value)
        print('goal, 1')
    if pos_controller_by_color.color_value == 2 and dir_type == 2:
        print('goal, 2')
    
    
    
    
    #if min_index == 0 and min_range < 0.25:
    #    speed.angular.z = 1.0
    #if min_index == 1 and min_range < 0.26:
    #    speed.angular.z = 1.75
    #if min_index == 2 and min_range < 0.26:
    #    speed.angular.z = -1.75
    #if min_index == 3 and min_range < 0.25:
    #    speed.angular.z = -1.0
            
    
    #if distance_ahead < 0.29 and distance_ahead > 0.28:
    #    speed.linear.x = 0.0
    #elif distance_ahead <= 0.28:
    #    speed.linear.x = -0.05
    #elif distance_ahead >= 0.29:
    #    speed.linear.x = 0.05

      

def rover_controller():
        global speed
        global dir_type
        
        dir_type = 0
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
