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

turnrad=0.0
is_turn=False
z=0

def callback_odom(msg):
        global _odom_x, _odom_y, _odom_theta 
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
        
        global turnrad
        global is_turn
        global t
        global _odom_theta

        minim = 65535
        mindex = -1

        for i in range(150,225):
            if((ranges[i]!=0)&(ranges[i]<minim)):
                minim=ranges[i]
                mindex=i

        if(is_turn):
            print("rotating:",_odom_theta-t,"odom:",_odom_theta)    
            if(abs(_odom_theta-t)>=abs(turnrad)-0.1):
                is_turn=False
                print("turn end")
        elif(minim<0.20):
            is_turn=True
            if(mindex>187.5):#左入射,turnrad...どのくらい回すか
                turnrad = (-125+(mindex-187.5)*2)*3.141592/125
            else:
                turnrad = (125+(mindex-187.5)*2)*3.141592/125
            t=_odom_theta
            print("turn start! now_z =",_odom_theta,"rad =",turnrad)
            
        if(is_turn):
            speed.linear.x=0
            if(turnrad>0):
                speed.angular.z=( 1)
            else:
                speed.angular.z=(-1)
        else:
            speed.linear.x=min(0.15,minim*0.3)
            speed.angular.z=0


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
