#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
import random
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
        # speed.linear.x = 0.05
        # speed.angular.z = 0.0
        """
        if (ranges[235] <= 0.25 or ranges[218] <= 0.25):
                speed.linear.x = 0.0
                speed.angular.z = -0.5 
        
        elif (ranges[173] <= 0.25 or ranges[190] <= 0.25):
                speed.linear.x = 0.0
                speed.angular.z = 0.5
        elif ranges[204] <= 0.25:
                speed.linear.x = 0.0
                speed.angular.z = -0.5
        """
              
        # w は重み        
        w =[-0.01,-0.01,-0.01,-0.01,0.01,0.01,0.01,-0.01,0.01,0.01,0.01,0.01,0.01,-0.01,-0.01,-0.01,-0.25,-0.25,-0.25,-0.25,-0.25,-0.25,-0.25,-0.25,0.25,0.25,0.25,0.25,0.25,0.25,0.25,0.25]
        # r0 ~r1 はそれぞれのニューロンの値
        r0=ranges[204]
        r1=ranges[187]
        r2=ranges[170]
        r3=ranges[153]
        r4=ranges[136]
        r5=ranges[121]
        r6=ranges[104]
        r7=ranges[87]
        r8=ranges[70]
        r9=ranges[53]
        r10=ranges[36]
        r11=ranges[19]
        r12=ranges[2]
        r13=ranges[255]
        r14=ranges[238]
        r15=ranges[221]
        # L is Loss Fucntion
        # L = r0 + r1 + r2 + r3 + r4 + r5 + r6 + r7
        L = min(r0,r1,r2,r3,r4,r5,r6,r7,r8,r9,r10,r11,r12,r13,r14,r15)
        # lr is Learning Rate
        lr = 0.02
        print(L)
        """
        for i in range(32):
            flag = random.randint(0,1)
            if flag == 1:
                w[i] += lr
                speed.linear.x = r0*w[0] +r1*w[1] +r2*w[2] +r3*w[3] +r4*w[4] +r5*w[5] +r6*w[6] +r7*w[7] +r8*w[8] +r9*w[9] +r10*w[10] +r11*w[11] +r12*w[12] +r13*w[13] +r14*w[14] +r15*w[15]
                speed.angular.z = r0*w[16] +r1*w[17] +r2*w[18] +r3*w[19] +r4*w[20] +r5*w[21] +r6*w[22] +r7*w[23] +r8*w[24] +r9*w[25] +r10*w[26] +r11*w[27] +r12*w[28] +r13*w[29] +r14*w[30] +r15*w[31]
                tmp_L = min(r0,r1,r2,r3,r4,r5,r6,r7,r8,r9,r10,r11,r12,r13,r14,r15)
                print(tmp_L,L)
                if tmp_L < L:
                    w[i] -= lr
                else:
                    L = tmp_L
            else:
                w[i] -= lr
                speed.linear.x = r0*w[0] +r1*w[1] +r2*w[2] +r3*w[3] +r4*w[4] +r5*w[5] +r6*w[6] +r7*w[7] +r8*w[8] +r9*w[9] +r10*w[10] +r11*w[11] +r12*w[12] +r13*w[13] +r14*w[14] +r15*w[15]
                speed.angular.z = r0*w[16] +r1*w[17] +r2*w[18] +r3*w[19] +r4*w[20] +r5*w[21] +r6*w[22] +r7*w[23] +r8*w[24] +r9*w[25] +r10*w[26] +r11*w[27] +r12*w[28] +r13*w[29] +r14*w[30] +r15*w[31]
                tmp_L = min(r0,r1,r2,r3,r4,r5,r6,r7,r8,r9,r10,r11,r12,r13,r14,r15)
                print(tmp_L,L)
                if tmp_L < L:
                    w[i] += lr
                else:
                    L = tmp_L
           
        """
        i = random.randint(0,31)
        flag = random.randint(0,1)
        if flag == 1:
            w[i] += lr
            speed.linear.x = r0*w[0] +r1*w[1] +r2*w[2] +r3*w[3] +r4*w[4] +r5*w[5] +r6*w[6] +r7*w[7] +r8*w[8] +r9*w[9] +r10*w[10] +r11*w[11] +r12*w[12] +r13*w[13] +r14*w[14] +r15*w[15]
            speed.angular.z = r0*w[16] +r1*w[17] +r2*w[18] +r3*w[19] +r4*w[20] +r5*w[21] +r6*w[22] +r7*w[23] +r8*w[24] +r9*w[25] +r10*w[26] +r11*w[27] +r12*w[28] +r13*w[29] +r14*w[30] +r15*w[31]
            tmp_L = min(r0,r1,r2,r3,r4,r5,r6,r7,r8,r9,r10,r11,r12,r13,r14,r15)
            print(tmp_L,L)
            if tmp_L < L:
                w[i] -= lr
            else:
                L = tmp_L
        else:
            w[i] -= lr
            speed.linear.x = r0*w[0] +r1*w[1] +r2*w[2] +r3*w[3] +r4*w[4] +r5*w[5] +r6*w[6] +r7*w[7] +r8*w[8] +r9*w[9] +r10*w[10] +r11*w[11] +r12*w[12] +r13*w[13] +r14*w[14] +r15*w[15]
            speed.angular.z = r0*w[16] +r1*w[17] +r2*w[18] +r3*w[19] +r4*w[20] +r5*w[21] +r6*w[22] +r7*w[23] +r8*w[24] +r9*w[25] +r10*w[26] +r11*w[27] +r12*w[28] +r13*w[29] +r14*w[30] +r15*w[31]
            tmp_L = min(r0,r1,r2,r3,r4,r5,r6,r7,r8,r9,r10,r11,r12,r13,r14,r15)
            print(tmp_L,L)
            if tmp_L < L:
                w[i] += lr
            else:
                L = tmp_L
        """
        r0=ranges[204]
        r1=ranges[170]
        r2=ranges[136]
        r3=ranges[104]
        r4=ranges[70]
        r5=ranges[36]
        r6=ranges[2]
        r7=ranges[238]
        w = [0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,-0.5,-0.3,0.3,0.5,0.3,0.3,0.3,0.3]
        #if speed.linear.x <= 0.349:
        speed.linear.x = r0*w[0] +r1*w[1] +r2*w[2] +r3*w[3] +r4*w[4] +r5*w[5] +r6*w[6] +r7*w[7]
        #if speed.angular.z <= 0.1: 
        speed.angular.z = r0*w[8] +r1*w[9] +r2*w[10] +r3*w[11] +r4*w[12] +r5*w[13] +r6*w[14] +r7*w[15]
        print(speed.angular.z)
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
