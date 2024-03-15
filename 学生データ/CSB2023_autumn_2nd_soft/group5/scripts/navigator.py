#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import tf
from tf.transformations import euler_from_quaternion

odom_x, odom_y, odom_theta = 0.0, 0.0, 0.0

class Navigator:
    goal = [[0.3, 0.3, 0.0, 1.0],[-0.1,-0.1,0,1.0]]
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
        goal_point.pose.orientation.w = 1.0
        goal_point.header.stamp = now
        goal_point.header.frame_id = 'map'

        self.goal_pub.publish(goal_point)

    def callback_amcl(self, msg):
        msg_pp = msg.pose.pose.position
        if(abs(self.goal[self.goal_number][0] - msg_pp.x) < 0.1 and abs(self.goal[self.goal_number][1] - msg_pp.y) < 0.1):
            print("succes")
            print("goal x:"+str(self.goal[self.goal_number][0])+"  goal y:"+str(self.goal[self.goal_number][1]))
            print("my_pos x:"+str(msg_pp.x)+"  my_pos y:"+str(msg_pp.y))
            print("odom_x:"+str(odom_x)+"  odom_y:"+str(odom_y))
            if(self.goal_number<1):
                self.goal_number += 1
                
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
        
        

if __name__ == '__main__':
    try:
        navigator = Navigator()
        rate = rospy.Rate(5)
        result_subscriber = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, navigator.callback_amcl)
        odom_subscriber = rospy.Subscriber('odom', Odometry,callback_odom)

        while not rospy.is_shutdown():
            print(navigator.goal_number)
            px = navigator.goal[navigator.goal_number][0]
            py = navigator.goal[navigator.goal_number][1]
            pz = navigator.goal[navigator.goal_number][2]
            ow = navigator.goal[navigator.goal_number][3]
            navigator.setGoal(px, py, pz, ow)
            rate.sleep()

    except rospy.ROSInterruptException: pass

    
