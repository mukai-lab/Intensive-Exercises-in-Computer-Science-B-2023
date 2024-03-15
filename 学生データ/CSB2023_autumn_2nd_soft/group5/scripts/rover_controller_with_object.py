#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan

_odom_x, _odom_y, _odom_theta,qz = 0.0, 0.0, 0.0,0.0
ranges = []
speed = Twist()
count=0

def callback_odom(msg):
        global _odom_x, _odom_y, _odon_theta, qz
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
        global count
        goal = [0.6,0.6]
        goal_z=math.atan(goal[1]/goal[0])/2
        speed.angular.z=0
        speed.linear.x=0.05
  

        if(count==0):
                print(goal_z)
                print(qz)
                if(qz>=goal_z+0.05):
                        speed.angular.z=-0.1
                        speed.linear.x=0.0     
                elif(qz<=goal_z-0.05):
                        speed.angular.z=0.1
                        speed.linear.x=0.0
                elif(qz<=goal_z+0.05 and qz>=goal_z-0.05):
                        speed.angular.z=0
                        speed.linear.x=0.1
                        count=1


        if(count==1):
                if(ranges[185]<=0.20):
                        speed.linear.x=0.02
                        speed.angular.z=0.349
                        print(185)
                elif(ranges[195]<=0.25):
                        speed.linear.x=0.0
                        speed.angular.z=0.349
                        print(195)
                elif(ranges[205]<=0.25):
                        speed.linear.x=0.0
                        speed.angular.z=-0.349
                        print(205)
                elif(ranges[215]<=0.20):
                        speed.linear.x=0.02
                        speed.angular.z=-0.349
                        print(215)
               
                elif(ranges[205]>=0.15 and ranges[185]>=0.15):
                        count=0
               
                if(_odom_x >= goal[0]-0.1 and _odom_x <= goal[0]+0.1):
                        if(_odom_y >= goal[1]-0.1 and _odom_y <= goal[1]+0.1):
                                speed.linear.x = 0.0
                                speed.angular.z = 0
                                

        '''
        p=ranges[185]*0.01 + ranges[195]*0.01 + ranges[205]*0.01 + ranges[215]*0.01
        if(p>0.15):
                speed.linear.x=0.15
        else:
                speed.linear.x=p
        print("P: " + str(speed.linear.x))

        
        a=0.3-ranges[185]
        b=0.3-ranges[195]
        c=0.3-ranges[205]
        d=0.3-ranges[215]

        if(a<0):
                a=0
        if(b<0):
                b=0
        if(c<0):
                c=0
        if(d<0):
                d=0

        q=a*1 + b*1 + c*1 + d*1
        
        if(q > 0.349):
                
                speed.angular.z=0.349
        elif(q<-0.349):
                speed.angular.z=-0.349
        else:
                speed.angular.z=q

        print("Q:" + str(speed.angular.z))
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
