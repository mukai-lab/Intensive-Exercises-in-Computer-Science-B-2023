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

        #min = 65535
        #turn = 0
        #0:str 1:ll 2:lc 3:rc 4:rr
        #turn r : minus / turn l : plus
        speed.linear.x=0.1

        print(ranges[160],ranges[180],ranges[195],ranges[215])

        #        if((ranges[160]!=0.0)&(min > ranges[160])):
        #            min = ranges[160]
        #            turn = 2
        #        if((ranges[180]!=0.0)&(min > ranges[180])):
        #            min = ranges[180]
        #            turn = 1
        #        if((ranges[195]!=0.0)&(min > ranges[195])):
        #            min = ranges[195]
        #            turn = 4
        #        if((ranges[215]!=0.0)&(min > ranges[215])):
        #            min = ranges[215]
        #            turn = 3
        #
        #        if(min>0.35):
        #            turn=0
        #            speed.linear.x=0.15
        #        elif(min<0.15):
        #            speed.linear.x=0
        #        else:
        #            speed.linear.x=(min*0.3)
        #
        #        if(turn == 1):
        #            speed.angular.z= 2.5
        #            print("left")
        #        elif(turn == 2):
        #            speed.angular.z= 0.5
        #            print("l")
        #        elif(turn == 3):
        #            speed.angular.z= -0.5
        #            print("r")
        #        elif(turn == 4):
        #            speed.angular.z= -2.5
        #            print("right")
        #        else:                      
        #            speed.angular.z= 0
        #            print("miss")

        speed.linear.x=min((ranges[160]*(0.08)+ranges[180]*(0.06)+ranges[195]*(0.06)+ranges[215]*(0.08)-0.04),0.15)
        speed.angular.z=min(1.5,max(-1.5,(ranges[160]*(-4.5)+ranges[180]*(-3.6)+ranges[195]*(3.6)+ranges[215]*(4.5)) ))

        print("speed",speed.linear.x)
        print("turn",speed.angular.z)


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
