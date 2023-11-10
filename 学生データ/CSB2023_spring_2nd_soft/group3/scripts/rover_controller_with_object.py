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
        '''min_val = 10000
        for i in ranges:
            if(160<ranges.index(i) and ranges.index(i)<200 and min_val>i and i>=0):
                min_val = i
                min_ind = ranges.index(i)
        if(min_val<=0.2):
            speed.linear.x=0
            if(min_ind<=180):
                speed.angular.z = 1
            else:
                speed.angular.z = -1
        else:
            speed.linear.x=0.1
            speed.angular.z=0'''
        r1=ranges[160]
        r2=ranges[170]
        r3=ranges[190]
        r4=ranges[200]
        #r5=ranges[200]
        '''x1=int(r1*5)
        x2=int(r2*5)
        x3=int(r3*5)
        x4=int(r4*5)
        x=bool(x1*x2*x3*x4)
        speed.linear.x = 0.1*x'''
        speed.linear.x = (r1/0.7 + r2/0.7 + r3/0.7 + r4/0.7)/4 * 0.05
        '''y1=bool(r1*10000)
        y2=bool(r2*10000)
        y3=bool(r3*10000)
        y4=bool(r4*10000)'''
        #speed.angular.z=x1*0.5+x2*0.5-x3*0.4-x4*0.5
        #speed.angular.z = (0.3/(r1+0.001))/2 + (0.3/(r2+0.001))/2 + (-0.3/(r3+0.001))/2 + (-0.3/(r4+0.001))/2
        speed.angular.z =  2.0 * ((0.8-r1) + (0.8-r2) + -(0.90-r3) + -(0.85-r4))
        #speed.angular.z = (-r1-r2+r3+r4) * (1-x)
        print('x:', speed.linear.x)
        print('z:', speed.angular.z)
        print('ranges[160]', ranges[160])
        print('ranges[175]', ranges[175])
        print('ranges[185]', ranges[185])
        print('ranges[200]', ranges[200])
        #print(str(ranges[175]) + str(175))
        #print(str(ranges[185]) + str(185))
        #print(str(ranges[200]) + str(200))



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
