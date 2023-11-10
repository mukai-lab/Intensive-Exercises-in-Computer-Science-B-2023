#! /usr/bin/env python3
   
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
#from nav_msgs.msg import Odometry

#odom_x=0

class Navigator:
    goal = [[0.20, -0.20, 0.0, 1.0],[-0.10,0.10,0.0,1.0],[0.10,-0.10,0.0,1.0]]
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
        #setGoal(self, 0.25, 0.25, 0.25, 1.0)
        #print('x: '+str(msg.pose.porise.position.x)+' y: '+str(msg.pose.pose.position.y))
        if (msg.pose.pose.position.x-0.10 <= self.goal[self.goal_number][0] and self.goal[self.goal_number][0] < msg.pose.pose.position.x+0.10) and (msg.pose.pose.position.y-0.10 <= self.goal[self.goal_number][1] and self.goal[self.goal_number][1] < msg.pose.pose.position.y+0.10):
                print('goal!')
                print('x: '+str(msg.pose.pose.position.x))
               
                if self.goal_number==0:
                   self.goal_number = 1
                   #print('x: '+str(msg.pose.pose.position.x))
                elif self.goal_number==1:
                   self.goal_number = 2
                  # print('x: '+str(msg.pose.pose.position.x))
        
	       


 
if __name__ == '__main__':
    try:
        navigator = Navigator()
        rate = rospy.Rate(5)
        result_subscriber = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, navigator.callback_amcl)
        while not rospy.is_shutdown():
           print(navigator.goal_number)
           px = navigator.goal[navigator.goal_number][0]
           py = navigator.goal[navigator.goal_number][1]
           pz = navigator.goal[navigator.goal_number][2]
           ow = navigator.goal[navigator.goal_number][3]
           navigator.setGoal(px, py, pz, ow)
           rate.sleep()
 
    except rospy.ROSInterruptException: pass

