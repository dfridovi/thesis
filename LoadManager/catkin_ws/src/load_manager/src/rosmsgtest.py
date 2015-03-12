import move_base_msgs.msg
import geometry_msgs.msg
import nav_msgs.msg
import rospy

def move_base_subscriber_callback(data):
	print data.pose
	

if __name__ == "__main__":
	rospy.init_node("listener",anonymous = True)
	callback = move_base_subscriber_callback
	#rospy.Subscriber("/move_base/feedback", move_base_msgs.msg.MoveBaseActionFeedback,callback)
	rospy.Subscriber("/odom", nav_msgs.msg.Odometry,callback)
	rospy.spin()
