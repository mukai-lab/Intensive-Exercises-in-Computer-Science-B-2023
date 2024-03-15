#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
import math
import numpy as np

_odom_x, _odom_y, _odom_theta = 0.0, 0.0, 0.0
ranges = []
speed = Twist()

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

def get_d(fw):
        if(len(fw)!=0):
            d=min(fw)
        else:
            d=0
        return d

def com_fw(fws, prev):
        a = sum([abs(fws[i]-prev[i]) for i in range(4)])
        return a


prev_fws=[0, 0, 0, 0]
count = 0
cs = 0
flag = 0
def callback_laser(msg):
        global ranges
        global prev_fws,count,flag,cs
        ranges = msg.ranges
        speed.linear.x = 0.1
        speed.angular.z = 0
        fw1 = [x for x in ranges[155:165] if x!= 0]
        fw2 = [x for x in ranges[175:185] if x!= 0]
        fw3 = [x for x in ranges[195:205] if x!= 0]
        fw4 = [x for x in ranges[215:225] if x!= 0]
        fws = [get_d(fw1), get_d(fw2), get_d(fw3), get_d(fw4)]
        fws_n0 = [x for x in fws if x!= 0]
        if(len(fws_n0)!=0):
            min_d = min(fws_n0)
            idx = fws.index(min_d)
            #print("idx: " + str(idx) + ", min_d: " + str(min_d))
            speed.linear.x = min(0.1, math.log(min_d+1)/3)
            if(idx == 0):
                speed.angular.z = 0.15
            elif(idx == 1):
                speed.angular.z = 0.349
            elif(idx == 2):
                speed.angular.z = -0.349
            elif(idx == 3):
                speed.angular.z = -0.15
            if(min_d < 0.2):
                speed.angular.z = 0.349
                if(idx >= 2):
                    speed.angular.z = speed.angular.z * -1
                speed.linear.x = 0
        else:
            print("forward")
        
        count = count + 1
        if(com_fw(fws, prev_fws) < 0.01 and count > 10 and speed.linear.x > 0.05 and flag==0):
            print("lidar stop")
            flag = 1
            cs = count
        if(flag == 1):
            if(count - cs < 15):
                speed.linear.x = -0.08
                speed.angular.z = np.sign(speed.angular.z) * 0.349
            else:
                flag = 0
        print(str(count) + " com" + str(com_fw(fws, prev_fws)) + ", speed: " + str(speed.linear.x)+", flag:" + str(flag) + ", cs:" + str(cs))
        prev_fws = fws
"""
        w = [[0.1, 0.4, 0.4, 0.1], [-0.3, -0.2, 0.2, 0.3]]
        nn_out = [0.0, 0.0]
        for i_w in range(len(w)):
                for i_f in range(len(fws)):
                        nn_out[i_w] += fws[i_f] * w[i_w][i_f]

        speed.linear.x = max(0, min(0.08, math.log(nn_out[0]+0.8)/3))
        speed.angular.z = np.sign(nn_out[1])*0.349*min(1, (1-abs(nn_out[1])/5))

        print("speed.linear.x: " + str(speed.linear.x) + ", speed.angular.z: " + str(speed.angular.z))
        print("nn_out: " + str(nn_out[0]) + ", " + str(nn_out[1]))
"""

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
