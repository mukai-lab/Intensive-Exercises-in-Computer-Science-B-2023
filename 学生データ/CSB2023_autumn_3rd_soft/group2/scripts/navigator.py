#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import tf
from tf.transformations import euler_from_quaternion

import numpy as np

odom_x, odom_y, odom_theta = 0.0, 0.0, 0.0
mode = 0
current_odom_x = 0
speed = Twist()

class Navigator:
    goal = [[0.48, 0.3, 0.0, 0.0, 0.0, 1.0, 1.0]]
    goal_number = 0
    def __init__(self):
        rospy.init_node('set_goal', anonymous=True)
        self.goal_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)

    def setGoal(self, px, py, pz, ox, oy, oz, ow):
        rospy.sleep(1.0)

        now = rospy.Time.now()
        goal_point = PoseStamped()

        goal_point.pose.position.x = px
        goal_point.pose.position.y = py
        goal_point.pose.position.z = pz
        goal_point.pose.orientation.x = ox
        goal_point.pose.orientation.y = oy
        goal_point.pose.orientation.z = oz
        goal_point.pose.orientation.w = ow
        goal_point.header.stamp = now
        goal_point.header.frame_id = 'map'

        self.goal_pub.publish(goal_point)

    def callback_amcl(self, msg):
        global odom_x, odom_y, mode, current_odom_y, speed
        if mode == 0:
            if msg.pose.pose.position.x > self.goal[self.goal_number][0]-0.1 and msg.pose.pose.position.x < self.goal[self.goal_number][0]+0.1:
                if msg.pose.pose.position.y > self.goal[self.goal_number][1]-0.1 and msg.pose.pose.position.y < self.goal[self.goal_number][1]+0.1:
                    print("Goal")
                    print(odom_x)
                    mode = 1
                    current_odom_x = odom_x

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

def controller():
    global speed, mode
    if mode == 1:
        speed.linear.x = -0.1
        speed.angular.z = 0
        if odom_x <= current_odom_x - 0.3:
            speed.linear.x = 0
    print(speed.linear.x)
    
if __name__ == '__main__':
    try:
        navigator = Navigator()
        rate = rospy.Rate(5)
        pub = rospy.Publisher('rover_drive',Twist, queue_size=1)
        result_subscriber = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, navigator.callback_amcl)
        odom_subscriber = rospy.Subscriber('odom', Odometry, callback_odom)

        while not rospy.is_shutdown():
           print(mode)
           px = navigator.goal[navigator.goal_number][0]
           py = navigator.goal[navigator.goal_number][1]
           pz = navigator.goal[navigator.goal_number][2]
           ox = navigator.goal[navigator.goal_number][3]
           oy = navigator.goal[navigator.goal_number][4]
           oz = navigator.goal[navigator.goal_number][5]
           ow = navigator.goal[navigator.goal_number][6]
           if mode == 0:
               navigator.setGoal(px, py, pz, ox, oy, oz, ow)
           rate.sleep()
           controller()
           pub.publish(speed)

    except rospy.ROSInterruptException: pass

