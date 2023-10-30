#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
import numpy as np

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
        global ranges,speed,r1,r2,r3,r4
        ranges = msg.ranges
        speed.linear.x = 0
        speed.angular.z = 0
        r1 = ranges[230]
        r2 = ranges[215]
        
        r3 = ranges[185]
        r4 = ranges[170]

        speed.linear.x=min([(r1*0.2+r2*0.3+r3*0.3+r4*0.2)/10, 0.1])
        a=min([(r1*0.2+r2*0.3-r3*0.3-r4*0.2)*10, 0.349])
        
        if a < -0.349:
           speed.angular.z = -0.349
        else:
           speed.angular.z = a

        s=[x for x in ranges[180:205] if x != 0]
        t=[x for x in ranges[155:180] if x != 0]
        s.append(1)
        t.append(1)
        if min(s) < 0.23 and min(t) < 0.23:
           speed.linear.x = -0.05
           k = min(s) - min(t)
           if k < 0:
              speed.angular.z = -0.3
           else:
              speed.angular.z = 0.3
        print('Right',min(t))
        print('Left',min(s))        
        
        
def pos_controller():
        global speed
          #speed.linera.x = 0.1


      

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
