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
        obj1,obj2,obj3,obj4 = ranges[150],ranges[170],ranges[190],ranges[210]
        #max = ranges[135]
        #idx = 135
        #for i in ranges(3):
        #        if(ranges[135 + 30*(i+1)] > max):
        #                max = ranges[135 + 30*(i+1)]
        #                idx = 135 + 30*(i+1)
        # speed.angular.z = -0.1 * (180 - idx)

        '''
        print(obj1)
        print(obj2)
        print(obj3)
        print(obj4)
        right = obj1 + obj2
        left = obj3 + obj4
        print(right)
        print(left)
        print('--------')
        if(right < 0.7 and left < 0.7):
                speed.linear.x = 0.0
                speed.angular.z = 0.5
        elif(right < 0.7):
                speed.linear.x = 0.03
                speed.angular.z = 0.3
        elif(left < 0.7):
                speed.linear.x = 0.03
                speed.angular.z = -0.3
        else:
                speed.linear.x = 0.05
                speed.angular.z  = 0.0   
       '''

        #linear
        w11 = -0.05
        w21 = 0.1
        w31 = 0.1
        w41 = -0.05
        
        #angular
        w12 = -3
        w22 = -1.5
        w32 = 1.5
        w42 = 3
        
        speed.linear.x = obj1*w11 + obj2*w21 + obj3*w31 + obj4*w41
        speed.angular.z = obj1*w12 + obj2*w22 + obj4*w32 + obj4*w42
        print(obj1)
        print(obj2)
        print(obj3)
        print(obj4)
        print(obj1*w11 + obj2*w21 + obj3*w31 + obj4*w41)
        print(obj1*w12 + obj2*w22 + obj4*w32 + obj4*w42)
        print('------')

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
