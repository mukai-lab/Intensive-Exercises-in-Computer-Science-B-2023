#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan

_odom_x, _odom_y, _odom_theta = 0.0, 0.0, 0.0
flag = 0
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
        global flag
        ranges = msg.ranges
        speed.linear.x = 0.05
        speed.angular.z = 0.00

     
        if(ranges[215] == 0 or ranges[215] >= 0.3):
                speed.linear.x = 0.05
                speed.angular.z = -0.27
        '''
                        for i in range(100):
                        count = count+1
                speed.linear.x = 0.00
                speed.angular.z = 0.00

        if (flag == 0 and ranges[205] > 0 and ranges[205] < 0.25):
                speed.linear.x = 0.01
                speed.angular.z = -0.6
                flag = 1
        if (flag == 1 and ranges[120] > 0 and ranges[120] < 0.25):
                print('aknsosa')
                speed.linear.x = 0
                speed.angular.z = 0
                
                
        if (ranges[155] > 0 and ranges[155] < 0.25):
                speed.linear.x = 0.02
                speed.angular.z = 0.34
        elif (ranges[175] > 0 and ranges[170] < 0.25):
                speed.linear.x = 0.02
                speed.angular.z =  0.34
        elif (ranges[185] > 0 and ranges[190] < 0.25):
                speed.linear.x = 0.02
                speed.angular.z = - 0.34
        elif (ranges[205] > 0 and ranges[205] < 0.25):
                speed.linear.x = 0.02
                speed.angular.z =  -0.34
      
      w = [[0.01, -2.0], [0.04, -1.0], [0.04, 1.0], [0.01, 2.0]]

        s = ranges[150]*w[0][0] + ranges[175]*w[1][0] + ranges[185]*w[2][0] + ranges[210]*w[3][0]
        if(s > 0.15):
                speed.linear.x = 0.15
        else:
                speed.linear.x = s
                
                
        an = ranges[150]*w[0][1] + ranges[175]*w[1][1] + ranges[185]*w[2][1] + ranges[210]*w[3][1]
        if(an > 0.349):
                speed.angular.z = 0.30
        else:
                speed.angular.z = an
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
