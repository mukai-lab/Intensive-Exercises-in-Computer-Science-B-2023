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
    global range
    ranges = msg.ranges
            
    min = 100
    index = 0
    ranges = msg.ranges
    #print(ranges[205])
    speed.linear.x = 0.045
    for i in range(156, 235, 26):
        if ranges[i] > 0 and min > ranges[i]:
            min = ranges[i]
            index = i
    print(speed.linear.x)
    if index == 156 and min < 0.25:
        speed.angular.z = 0.7
    elif index == 182 and min < 0.25:
        speed.angular.z = 1.1
    elif index == 208 and min < 0.25:
        speed.angular.z = -1.1
    elif index == 234 and min < 0.25:
        speed.angular.z = -0.7
    else:
        speed.angular.z = 0.0

    """
    min = 100
    ranges = msg.ranges
    for i in range(130, 260):
        if(ranges[i] > 0 and min > ranges[i]):
            min = ranges[i]
            index = i
    print(index)
    if index > 205:
        speed.angular.z = 0.3
    else:
        speed.angular.z = -0.3

        

    ranges = msg.ranges[205]
    print(ranges)
    if (ranges > 0.255 and ranges < 0.5):
        speed.linear.x = 0.045
    elif (ranges <= 0.255 and ranges >= 0.245):
        speed.linear.x = 0
    elif (ranges > 0 and ranges < 0.245):
        speed.linear.x = -0.045
    else:
        speed.linear.x = 0
        speed.angular.z = 0
        

    """
    """
    ranges = msg.ranges[205]
    print(ranges)
    if ranges > 0.25 + 0.0106:
        speed.linear.x = 0.04
        speed.angular.z = 0
    else:
        speed.linear.x = 0
        speed.angular.z = 0
        
    """
def controller():
    global speed

    #speed.linear.x = 0.1
    #speed.angular.z = 0.0
        
"""
    if odom_x < 0.1:
        speed.linear.x = 0.045
        speed.angular.z = 0
    else:
        speed.linear.x = 0
        speed.angular.z = 0

    print(odom_x)
"""





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
