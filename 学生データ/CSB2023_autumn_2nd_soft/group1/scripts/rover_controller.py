#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
import math

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

    global ranges, speed
    #speed.linear.x = 0.1 
    #speed.angular.z = 0
"""
    ranges = msg.ranges
    fw = [x for x in ranges[170:210] if x != 0]
    ang = [x for x in ranges if x != 0]
    d = 1
    if(len(fw) != 0):
      d = min(fw)
      speed.linear.x = min(0.15, math.log(d+0.75))
      #print(ranges.index(d))
    if(len(ang) != 0):
      dist = ranges.index(min(ang))
      speed.angular.z = math.pi/180 * 360/len(ranges) * (dist - 190) 
    if(d < 0.25):
      speed.linear.x = max(-0.15, -1 * math.log((0.25-d)+1))
    if(d > 0.5):
      speed.linear.x = 0
      speed.angular.z = 0
    print(d)
"""
count = 0
prev = 0
def controller():
    global count
    global speed
    global prev
    d = 0.005
    count = count + 1
     
    print(str(count) + " odom_x: " + str(odom_x) + ", speed.linear.x: " + str(speed.linear.x))
    if(odom_x - prev < speed.linear.x/5 - d and count > 5):
      speed.linear.x = 0
      print("stop")
    prev = odom_x

def rover_controller():
    global speed

    rospy.init_node('rover_controller', anonymous=True)

    pub = rospy.Publisher('rover_drive',Twist, queue_size=1)

    rate = rospy.Rate(5)

    odom_subscriber = rospy.Subscriber('odom', Odometry, callback_odom)

    lidar_subscriber = rospy.Subscriber('scan', LaserScan, callback_laser)

    speed.linear.x = 0.1

    while not rospy.is_shutdown():
        rate.sleep()
        controller()
        pub.publish(speed)

if __name__ == '__main__':
    rover_controller()
