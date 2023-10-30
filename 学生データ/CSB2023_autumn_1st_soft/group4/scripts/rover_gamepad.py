#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#このプログラムは、ライトローバーをゲームパッドで動かすためのノードです。

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from sensor_msgs.msg import LaserScan

ranges = []
flag = 0

speed = Twist()

def callback_laser(msg):
        global speed
        global ranges
        global flag
        ranges = msg.ranges
        if ranges[203] > 0.0 and ranges[203] < 0.25 or ranges[236] > 0.0 and ranges[236] < 0.13 or ranges[170] > 0.0 and ranges[170] < 0.13:
                flag = 1
        elif ranges[68] > 0.0 and ranges[68] < 0.17 or ranges[101] > 0.0 and ranges[101] < 0.12 or ranges[35] > 0.0 and ranges[35] < 0.12:
                flag = -1 
        else:
                flag = 0
        print(ranges[203])


def call_back(data):
        global speed
        global flag

        #ゲームパッドの入力値からライトローバーの直進・旋回速度を算出
        #ゲームパッドによってボタン、アナログスティック等の配列要素は違うことに注意
        if flag == 1:
                if data.axes[1]*0.1 > 0.0:
                        speed.linear.x = 0.0
                        speed.angular.z = data.axes[2]*2.0
                else:
                        speed.linear.x = data.axes[1]*0.1
                        speed.angular.z = data.axes[2]*2.0
        elif flag == -1:
                if data.axes[1]*0.1 < 0.0:
                        speed.linear.x = 0.0
                        speed.angular.z = data.axes[2]*2.0
                else:
                        speed.linear.x = data.axes[1]*0.1
                        speed.angular.z = data.axes[2]*2.0
        else:
                speed.linear.x = data.axes[1]*0.1
                speed.angular.z = data.axes[2]*2.0
        

def rover_gamepad():
        global speed

        rospy.init_node('rover_gamepad', anonymous=True)

        pub = rospy.Publisher('rover_drive',Twist, queue_size=1)

        rate = rospy.Rate(20)

        
        rospy.Subscriber('joy',Joy,call_back)
        lidar_subscriber = rospy.Subscriber('scan', LaserScan, callback_laser)
        

        while not rospy.is_shutdown():
                pub.publish(speed)
                rate.sleep()

if __name__ == '__main__':
        rover_gamepad()
