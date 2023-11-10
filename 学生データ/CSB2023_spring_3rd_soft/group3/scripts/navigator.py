#! /usr/bin/env python3
   
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

class Navigator:
    goal = [[0.2, -0.2, 0.0, 1.0],[-0.2,-0.2,0.0,1.0],[0.2,0.2,0.0,1.0],[-0.2,0.2,0.0,1.0]]
    is_goal = False
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
        diff_x = abs(x - goal_x)
        diff_y = abs(y - goal_y)
        if(diff_x < 0.1 and diff_y < 0.1 and self.is_goal == False):
            if(self.goal_number == 3):
                self.is_goal = True
            print(goal_x)
            print(goal_y)
            print(x)
            print(y)
            print(diff_x)
            print(diff_y)
            print('goal')
            #print(goal_number)
            if(self.goal_number <= 2):
                self.goal_number += 1
            


 
if __name__ == '__main__':
    try:
        navigator = Navigator()
        rate = rospy.Rate(5)
        result_subscriber = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, navigator.callback_amcl)
        while not rospy.is_shutdown():
           #print(navigator.goal_number)
           px = navigator.goal[navigator.goal_number][0]
           py = navigator.goal[navigator.goal_number][1]
           pz = navigator.goal[navigator.goal_number][2]
           ow = navigator.goal[navigator.goal_number][3]
           navigator.setGoal(px, py, pz, ow)
           rate.sleep()
 
    except rospy.ROSInterruptException: pass

