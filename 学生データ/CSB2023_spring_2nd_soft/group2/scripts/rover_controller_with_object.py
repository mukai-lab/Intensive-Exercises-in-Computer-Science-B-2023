#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
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
       # speed.linear.x = 0.05
        ranges2 = [ranges[185],ranges[195],ranges[215],ranges[225]]
        w1=-0.05
        w2=-0.08
        w3=01
        w4=0.08
        w5=0.03
        w6=0.015
        w7=0.015
        w8=0.032
        r1,r2,r3,r4=math.log(ranges2[0]+0.0001),math.log(ranges2[1]+0.0001),math.log(ranges2[2]+0.0001),math.log(ranges2[3]+0.0001)
        speed.angular.z=(w1*r1)+(w2*r2)+(w3*r3)+(w4*r3)
        speed.linear.x=w5*ranges2[0]+w6*ranges2[1]+w7*ranges2[2]+w8*ranges2[3]
'''   

        if(min(ranges2) <= 0.2):
                if(ranges2.index(min(ranges2))==1):
                        speed.angular.z = 2.0
                elif(ranges2.index(min(ranges2))==2):
                        speed.angular.z = -2.0
                elif(ranges2.index(min(ranges2))==0):
                        speed.angular.z = 1.0
                elif(ranges2.index(min(ranges2))==3):
                        speed.angular.z = -1.0
        else:
                speed.angular.z = 0.0
'''




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
