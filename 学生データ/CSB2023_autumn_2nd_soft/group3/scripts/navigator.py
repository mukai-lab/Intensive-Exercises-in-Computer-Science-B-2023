#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import tf
from tf.transformations import euler_from_quaternion

odom_x, odom_y, odom_theta = 0.0, 0.0, 0.0

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
    print("odo", odom_x, odom_y)

class Navigator:
    goal = [[0.2, 0.2, 0.0, 1.0],[-0.2, 0.2, 0.0, 1.0]]
    goal_number = 0
    def __init__(self):
        rospy.init_node('set_goal', anonymous=True)
        self.goal_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)

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
        xx = self.goal[self.goal_number][0]- msg.pose.pose.position.x
        yy = self.goal[self.goal_number][1] - msg.pose.pose.position.y
        rev_xx = msg.pose.pose.position.x - self.goal[self.goal_number][0]
        rev_yy = msg.pose.pose.position.y - self.goal[self.goal_number][1]
        print(self.goal[self.goal_number][0], msg.pose.pose.position.x, self.goal[self.goal_number][0], msg.pose.pose.position.x)
        if (((xx  < 0.1 and xx > 0.0) and  (yy < 0.1 and  yy > 0.0)) or ((rev_xx < 0.1 and rev_xx > 0.0) and (rev_yy < 0.1 and rev_yy > 0.0))):
            if (self.goal_number == 0):
                print("goal_1")
                self.goal_number += 1
            elif (self.goal_number == 1):
                print("goal_2")
                
                


if __name__ == '__main__':
    try:
        navigator = Navigator()
        rate = rospy.Rate(5)
        result_subscriber = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, navigator.callback_amcl)
        odom_subscriber = rospy.Subscriber('odom', Odometry, callback_odom)
        #rover_controller()

        
        while not rospy.is_shutdown():
           print(navigator.goal_number)
           px = navigator.goal[navigator.goal_number][0]
           py = navigator.goal[navigator.goal_number][1]
           pz = navigator.goal[navigator.goal_number][2]
           ow = navigator.goal[navigator.goal_number][3]
           navigator.setGoal(px, py, pz, ow)
           rate.sleep()

    except rospy.ROSInterruptException: pass

