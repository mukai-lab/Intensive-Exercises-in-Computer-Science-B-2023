#! /usr/bin/env python3
   
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
#from lightrover_ros.msg import color_detector

class Navigator:
    goal = [[0.2, -0.2, 0.0, 1.0],[-0.2, 0.2, 0.0, 1.0],[-0.2, -0.2, 0.0, 1.0],[0.2, 0.2, 0.0, 1.0]]
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
        a = msg.pose.pose.position.x - self.goal[self.goal_number][0]
        b = msg.pose.pose.position.y - self.goal[self.goal_number][1]

        if a < 0.1 and a > -0.1 and b < 0.1 and b > -0.1:
            print("x:" + str(a) + ", y:" + str(b))
            #self.goal_number += 1
            #if self.goal_number == 5:
            print("Goal")
            #self.goal_number = self.goal_number % 4
            #if self.goal_number < 2:
             #   print("Goal")




  #  def a(data):
  #      navigator.goal_number = data.color_type
            
if __name__ == '__main__':
    try:
        navigator = Navigator()
        rate = rospy.Rate(5)
        result_subscriber = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, navigator.callback_amcl)
        while not rospy.is_shutdown():
            print(navigator.goal_number)
           # rospy.Subscriber('color_detector',  color_detector, a)
            px = navigator.goal[navigator.goal_number][0]
            py = navigator.goal[navigator.goal_number][1]
            pz = navigator.goal[navigator.goal_number][2]
            ow = navigator.goal[navigator.goal_number][3]
            navigator.setGoal(px, py, pz, ow)
            rate.sleep()
            
    except rospy.ROSInterruptException: pass

