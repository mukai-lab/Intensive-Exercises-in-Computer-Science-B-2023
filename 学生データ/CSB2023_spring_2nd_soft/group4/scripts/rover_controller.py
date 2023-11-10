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

distance_ahead = 10.0
min_index = 0
min_range = 10.0
def callback_laser(msg):
    global ranges
    global distance_ahead
    global min_index
    global min_range
    ranges = msg.ranges
    #print('head  ' + str(ranges[205]))
    #print('left  ' + str(ranges[1]))
    #print('right ' + str(ranges[140]))
    #print('teil  ' + str(ranges[80]))
    #for i in range(200,210):
    #    print('ranges at ' + str(i) + ' ' + str(ranges[i]))
    distance_ahead = ranges[205]
    min_range = 10.0
    for i in range(270):
        if (ranges[i] != 0 and ranges[i] < min_range):
            min_range = ranges[i]
            min_index = i
         
    
def controller():
    global speed
    global distance_ahead
    global min_index
    global min_range
    if distance_ahead < 0.30:
        speed.linear.x = 0.0
    else:
        speed.linear.x=1
    #print(distance_ahead)
    
   # if min_range > 0.5:
    #    speed.linear.x = 0
     #   speed.angular.z = 0
      #  return
        
   # print('min index is' + str(min_index))
   # print('min range is' + str(min_range))
    
   # speed.angular.z = 0.0
    #left side
   # if min_index > 210 or min_index < 80:
   #     speed.angular.z = 0.3
    #right_side
   # if min_index >= 81 and min_index < 200:
    #    speed.angular.z = -0.3
        
   # if distance_ahead < 0.29 and distance_ahead > 0.28:
    #    speed.linear.x = 0.0
   # elif distance_ahead <= 0.28:
    #    speed.linear.x = -0.05
   # elif distance_ahead >= 0.29:
    #    speed.linear.x = 0.05



 


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
