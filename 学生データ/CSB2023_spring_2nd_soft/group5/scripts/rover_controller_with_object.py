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
        global speed
        ranges = msg.ranges
        w = [
            [0.10,0.15,0.15,0.10],
            [8,10,10,8]
        ]
        r1=ranges[210]
        r2=ranges[200]
        r3=ranges[190]
        r4=ranges[180]
        speed.linear.x = -0.20+r1*w[0][0] + r2*w[0][1] + r3*w[0][2] + r4*w[0][3]
        speed.angular.z = r1*(1/w[1][0]) + r2*(1/w[1][1]) + r3*(1/w[1][2]) + r4*(1/w[1][3])
        '''if(fr<0.2 or r<0.2):
            speed.angular.z=0.3
            speed.linear.x=0
        elif(fl<0.2 or l<0.2):
            speed.angular.z=-0.3
            speed.linear.x=0
        else:
            speed.linear.x=0.1
            speed.angular.z=0'''
        
        




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
