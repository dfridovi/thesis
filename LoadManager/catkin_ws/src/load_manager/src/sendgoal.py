import roslib
import rospy

import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import *
from geometry_msgs.msg import *

rospy.init_node('talker', anonymous=True)

move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
goal = MoveBaseGoal()
goal.target_pose.header.frame_id = 'map'
goal.target_pose.header.stamp = rospy.Time.now()

goal.target_pose.pose.position.x=1
goal.target_pose.pose.position.y=2
goal.target_pose.pose.position.z=0

goal.target_pose.pose.orientation.x=0
goal.target_pose.pose.orientation.y=0	
goal.target_pose.pose.orientation.z=.3
goal.target_pose.pose.orientation.w=.99

move_base.send_goal(goal)
y=move_base.wait_for_result(rospy.Duration(2)) 
if not y:
	#move_base.cancel_goal()
	rospy.loginfo("Timed out achieving goal")
