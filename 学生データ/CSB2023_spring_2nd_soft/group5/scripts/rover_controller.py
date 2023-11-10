#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan

odom_x, odom_y, odom_theta = 0.0, 0.0, 0.0
ranges = []
speed = Twist()

def callback_odom(msg):
    global odom_x, odom_y, odom_theta 
    odom_x = msg.pose.pose.position.x  
    odom_y = msg.pose.pose.position.y  
    qx = msg.pose.pose.orientation.x
    qy = msg.pose.pose.orientation.y
    qz = msg.pose.pose.orientation.z
    qw = msg.pose.pose.orientation.w
    q = (qx, qy, qz, qw)
    e = euler_from_quaternion(q)
    odom_theta = e[2]

def callback_laser(msg): 
    global ranges
    global speed
    
    speed.langular.x = ranges[195]
    
'''    global
 speed
    ranges = msg.ranges
    m=ranges[1]
    mi=1
    if(m==0):
        mi=195
    for j in range(2,259):
        if(m>ranges[j] and ranges[j]!=0):
            m=ranges[j]
            mi=j

    if(mi>190 and mi<200):       
        speed.angular.z=0     
        if(m<=0.247):
            speed.linear.x=-0.01
        elif(m>=0.253 and m<=0.5):
            speed.linear.x=0.01
        else:
            speed.linear.x=0
    elif((mi>=1 and mi <=65) or (mi>=200 and mi<=259)):
        speed.angular.z=0.3
    else:
        speed.angular.z=-0.3
'''



def controller():
    global speed
    

#    print(odom_x)
#    if(odom_x>=0.103):
#        speed.linear.x=-0.001
#    elif(odom_x<=0.099):
#        speed.linear.x=0.05
#    else:
#        speed.linear.x=0

#    speed.linear.x = 0.05
#    if(odom_x >= 0.1):
#        speed.linear.x = 0






def rover_controller():
    global speed

    rospy.init_node('rover_controller', anonymous=True)

    pub = rospy.Publisher('rover_drive',Twist, queue_size=1)

    rate = rospy.Rate(5)
        
    odom_subscriber = rospy.Subscriber('odom', Odometry, callback_odom)
        
    lidar_subscriber = rospy.Subscriber('scan', LaserScan, callback_laser)

    while not rospy.is_shutdown():
        rate.sleep()
        controller()
        pub.publish(speed)

if __name__ == '__main__':
    rover_controller()
