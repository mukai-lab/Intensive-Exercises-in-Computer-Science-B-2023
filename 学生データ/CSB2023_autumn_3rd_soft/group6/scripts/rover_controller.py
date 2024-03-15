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
a = 1


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
    ranges = msg.ranges
    global a 
    #for i in range(170,190):
    #    print(i ," " , ranges[i], ",")
#    min = 1
#    min_index = 160
#    for i in range(160, 190):
#     if (ranges[i] != 0.0) and  (min > ranges[i]):
#            min = ranges[i]
#            min_index = i
#            print(min_index)
#            
#    print(ranges[min_index])
#    
#    if(ranges[min_index] != 0.0) and (ranges[min_index] > 0.25):
#        print("distances", ranges[min_index])
#        speed.linear.x = 0.05
#        speed.angular.z =  (min_index - 177) * 0.045
#        
#    elif(ranges[min_index] != 0.0) and (ranges[min_index] < 0.25):
#        print("distance: ",ranges[min_index])
#        speed.linear.x = -0.05
#        speed.angular.z = (min_index - 177) * 0.045
#    else:
#        if (a == 1):
#           print("stoppppppppppppppppppppppppppppppppppppppppppp!")
#            a = 0
#        speed.linear.x = 0
    
    #if(ranges[175] != 0.0) and (ranges[175] > 0.25):
    #    print("distance: ",ranges[175])
    #    speed.linear.x = 0.1
    #elif(ranges[175] != 0.0) and (ranges[175] < 0.25):
    #    print("distance: ",ranges[175])
    #    speed.linear.x = -0.1
    #else:
    #    if(a == 1):
    #        printw("stoppppppppppppppppppppppppppppppppppppppppppp!")
    #        a =0
    #    speed.linear.x = 0
    
    


def controller():
    global speed
    #if odom_x < 0.1:
    #    speed.linear.x =  0.1
    #else:
    #    speed.linear.x = 0.0
    speed.linear.x = 0.05


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
