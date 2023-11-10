#! /usr/bin/env python3
   
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

class Navigator:
    goal = [[0.15, -0.15, 0.0, 1.0],[-0.15,0.15,0.0,1.0],[0.15,0.15,0.0,1,0],[-0.15,-0.15,0.0,1.0]]
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
        xn=msg.pose.pose.position.x
        yn=msg.pose.pose.position.y
        xg=self.goal[self.goal_number][0]
        yg=self.goal[self.goal_number][1]
        end=0
        if(xn<=xg+0.10 and xn>=xg-0.10 and yn<=yg+0.10 and yn>=yg-0.10):
            if(self.goal_number==3 and end==1):
                print("Goal")
                print("xn:")
                print(xn)
                print("yn:")
                print(yn)
                print("xg:")
                print(xg)
                print("yg:")
                print(yg)
                print("xn-xg:")
                print(xn - xg)
                print("yn-yg:")
                print(yn - yg)
                end=1
            else:
                self.goal_number+=1
                print("xn:")
                print(xn)
                print("yn:")
                print(yn)
                print("xg:")
                print(xg)
                print("yg:")
                print(yg)
                print("xn-xg:")
                print(xn - xg)
                print("yn-yg:")
                print(yn - yg)
 
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

