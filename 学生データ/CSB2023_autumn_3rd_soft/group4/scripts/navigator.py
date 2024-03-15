#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
import sys
from nav_msgs.msg import Odometry
import tf
from tf.transformations import euler_from_quaternion

odom_x = 0.0
odom_y = 0.0
odom_theta = 0.0

class Navigator:
    goal = [[0.2, 0.2, 0.0, 1.0],[0.0, 0.0, 0.1, 1.0],[-0.2, -0.2, 0.0, 1.0],[0.2, -0.2, 0.0, 1.0]]
    goal_number = 0
    def __init__(self):
        rospy.init_node('set_goal', anonymous=True)
        self.goal_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)

    def setGoal(self, px, py, pz, ow):
        rospy.sleep(1.0)

        now = rospy.Time.now()
        goal_point = PoseStamped()

        goal_point.pose.position.x = px
        goal_point.pose.position.y = py
        goal_point.pose.position.z = pz
        goal_point.pose.orientation.w = ow
        goal_point.header.stamp = now
        goal_point.header.frame_id = 'map'

        self.goal_pub.publish(goal_point)

    def callback_amcl(self, msg):
        global odom_x, odom_y
        x = msg.pose.pose.position.x - self.goal[self.goal_number][0]
        y = msg.pose.pose.position.y - self.goal[self.goal_number][1]
        
        if abs(x) <= 0.1 and abs(y) <= 0.1 :
            print("GOAL")
            print(odom_x)
            print(msg.pose.pose.position.x)
            print(odom_y)
            print(msg.pose.pose.position.y)
            self.goal_number = self.goal_number + 1
            if(self.goal_number == 3):
                sys.exit(0)

    def callback_odom(self, msg):
        global odom_x, odom_y
        odom_x = msg.pose.pose.position.x 
        odom_y = msg.pose.pose.position.y 

if __name__ == '__main__':
    try:
        navigator = Navigator()
        rate = rospy.Rate(5)
        result_subscriber = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, navigator.callback_amcl)
        odom_subscriber = rospy.Subscriber('odom', Odometry,navigator.callback_odom)

        while not rospy.is_shutdown():
           print(navigator.goal_number)
           px = navigator.goal[navigator.goal_number][0]
           py = navigator.goal[navigator.goal_number][1]
           pz = navigator.goal[navigator.goal_number][2]
           ow = navigator.goal[navigator.goal_number][3]
           navigator.setGoal(px, py, pz, ow)
           rate.sleep()

    except rospy.ROSInterruptException: pass

