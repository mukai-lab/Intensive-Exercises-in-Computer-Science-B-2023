#! /usr/bin/env python3
   
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import rospy
from geometry_msgs.msg import Twist
import tf
from tf.transformations import euler_from_quaternion


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

class Navigator:
    goal = [[0.0, 0.0, 0.0, 1.0],[0.25, 0.25,0.0,1.0],[0.25,-0.25,0.0,1.0],[-0.25,-0.25,0.0,1.0]]
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

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        goal_x = self.goal[self.goal_number][0]
        goal_y = self.goal[self.goal_number][1]
        
        flag1 = False
        flag2 = False

        d = 0.1
        if x <= goal_x + d and x >= goal_x - d:
            flag1 = True
        if y <= goal_y + d and y >= goal_y - d:
            flag2 = True

        if flag1 and flag2:
            if self.goal_number == 0:
                self.goal_number = 1
                print(0,"x:",x,"y:",y)
                print(0,"odom_x:",_odom_x,"odom_y:",_odom_y)
            elif self.goal_number == 1:
                self.goal_number = 2
                print(1,"x:",x,"y:",y)
                print(1,"odom_x:",_odom_x,"odom_y:",_odom_y)
            elif self.goal_number == 2:
                self.goal_number = 3
                print(2,"x:",x,"y:",y)
                print(2,"odom_x:",_odom_x,"odom_y:",_odom_y)
            elif self.goal_number == 3:
                print(3,"x:",x,"y:",y)
                print(3,"odom_x:",_odom_x,"odom_y:",_odom_y)
                print("Goal")
                

            
                
if __name__ == '__main__':
    try:
        navigator = Navigator()
        rate = rospy.Rate(5)
        result_subscriber = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, navigator.callback_amcl)
        odom_subscriber = rospy.Subscriber('odom', Odometry, callback_odom)
        while not rospy.is_shutdown():
           print(navigator.goal_number)
           px = navigator.goal[navigator.goal_number][0]
           py = navigator.goal[navigator.goal_number][1]
           pz = navigator.goal[navigator.goal_number][2]
           ow = navigator.goal[navigator.goal_number][3]
           navigator.setGoal(px, py, pz, ow)
           rate.sleep()
 
    except rospy.ROSInterruptException: pass

