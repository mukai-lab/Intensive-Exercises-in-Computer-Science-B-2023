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
back = False
interval = 0

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
        global back
        global interval
        ranges = msg.ranges
        r1 = ranges[260]
        r2 = ranges[220]
        r3 = ranges[210]
        r4 = ranges[190]
        r5 = ranges[180]
        r6 = ranges[140]
        if(back):
                time=0
                while(time<1000000):
                        speed.linear.x = -0.03
                        speed.angular.z = 0
                        time = time+1
                time=0
                while(time<2000000):
                        speed.linear.x = 0
                        speed.angular.z = 0.349
                        time = time+1
                time=0
                back=False
                interval = 20
        elif(interval != 0):
                print(interval)
                speed.linear.x = 0
                speed.angular.z = 0
                interval = interval-1
        else:
                speed.linear.x = (r2*0.2+r3*0.3+r4*0.3+r5*0.2)/10
                speed.angular.z = (r1*0.05+r2*0.15+r3*0.3-r4*0.3-r5*0.15-r6*0.05)*10
                if(speed.linear.x > 0.1):
                        speed.linear.x = 0.1
                if(speed.angular.z > 0.349):
                        speed.angular.z = 0.349
                if(speed.angular.z < -0.349):
                        speed.angular.z = -0.349
                if((r2<0.15 or r3<0.30 or r4<0.30 or r5<0.15) and r2!=0 and r3!=0 and r4!=0 and r5!=0):
                        back = True
                                
#        s1=100
#        s2=100
#        s3=100
#        s4=100
#        if(r1 != 0):
#                s1 = r1
#        if(r2 != 0):
#                s2 = r2
#        if(r3 != 0):
#                s3 = r3
#        if(r4 != 0):
#                s4 = r4
#        min_right = min(r3, r4)
#        min_left = min(r1, r2)
#        if(min_left<0.20):
#                speed.linear.x = -0.03
#                speed.angular.z = -0.349
#        elif(min_right<0.20):
#                speed.linear.x = -0.03
#                speed.angular.z = 0.349

        
#        kadai5
#        if((ranges[230] < 0.3 or ranges[210] < 0.3) and ranges[240] != 0 and ranges[210] != 0):
#                speed.linear.x = 0.01
#                speed.angular.z = -0.349
#        if((ranges[170] < 0.3 or ranges[190] < 0.3) and ranges[160] != 0 and ranges[190] != 0):
#                speed.linear.x = 0.01
#                speed.angular.z = 0.349
#        if(speed.linear.x > 0.15):
#                speed.linear.x = 0.15
#        if(speed.angular.z > 0.349):
#                speed.angular.z = 0.349




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
