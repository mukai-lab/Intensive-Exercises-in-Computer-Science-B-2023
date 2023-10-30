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
step = 0
wall_right_length = 0
wall_left_length = 0
ms = 0

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
        global speed
        global step
        global wall_right_length
        global wall_left_length
        global ms
        ranges = msg.ranges
        min_length = 343
        min_id = 189
        length_list = [ranges[154], ranges[175], ranges[196], ranges[217]]
        
                        
        #speed.angular.z = (length_list[0] * (-0.45) + length_list[1] * (-0.3) + length_list[2] * 0.3 + length_list[3] * 0.45) * 14
        #speed.linear.x = (length_list[0] * (2) + length_list[1] * (4) + length_list[2] * 4 + length_list[3] * 2) * 0.01
        #if(speed.angular.z > 0.349):
        #    speed.angular.z = 0.349
        #if(speed.linear.x > 0.149):
        #    speed.linear.x = 0.149        
        #if(speed.angular.z < -0.349):
        #    speed.angular.z = -0.349
        #print(ranges[140])
        print(step)
        if(step == 0):
            wall_right_length = ranges[140]
            wall_left_length = ranges[238]
            print('wall_right_length')
            print(wall_right_length)
            print('wall_left_length')
            print(wall_left_length)
            step = 1
        if(step == 1):
            speed.linear.x = 0.07
            speed.angular.z = 0.0
            if(ranges[140] <= wall_right_length - 0.07):
                if(ranges[140] != 0.0):
                    speed.angular.z = 0.349
                    speed.linear.x = 0.047
                    step = 2
            if(ranges[238] <= wall_left_length - 0.07):
                if(ranges[238] != 0.0):
                    speed.angular.z = -0.349
                    speed.linear.x = 0.047
                    step = 2
        if(step == 2):
            ms += 1
            print(ms)
            if(ms >= 59):
                speed.angular.z = 0.0
                speed.linear.x = -0.05
                step = 3
        if(step == 3):
            print(ranges[63])
            if(ranges[63] <= 0.11):
                if(ranges[63] != 0):
                    speed.linear.x = 0.0
                    step = 4


def pos_controller():
        global speed
        #speed.linear.x = 0.07


      

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
