import rospy
import move_base_msgs.msg
import geometry_msgs.msg
import nav_msgs.msg
import time

def talker():
	pub = rospy.Publisher('/initialpose', geometry_msgs.msg.PoseWithCovarianceStamped, queue_size=10)
	rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(1) # 10hz
	
     	pose = geometry_msgs.msg.PoseWithCovarianceStamped()
	pose.header.seq=0
	pose.header.frame_id = "map"
	pose.pose.pose.position.x=1
	pose.pose.pose.position.y=1
	pose.pose.pose.position.z=0
	pose.pose.covariance=[0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
	pose.pose.pose.orientation.z=0.0267568523876
	pose.pose.pose.orientation.w=0.999641971333
     	rospy.loginfo(pose)
     	pub.publish(pose)

	pub2 = rospy.Publisher('/move_base/goal', move_base_msgs.msg.MoveBaseActionGoal,queue_size=10)
	goal = move_base_msgs.msg.MoveBaseActionGoal()	
	goal.header.stamp.secs = int(time.time())
	goal.header.seq=0
	goal.header.stamp.nsecs = 1
	goal.header.frame_id="map"
	goal.goal.target_pose.header.stamp.secs=int(time.time())
	goal.goal.target_pose.header.stamp.nsecs=2
	goal.goal.target_pose.header.frame_id="map"
	goal.goal.target_pose.pose.position.x=1
	goal.goal.target_pose.pose.position.y=2
	goal.goal.target_pose.pose.orientation.z=0.33132694162
	goal.goal.target_pose.pose.orientation.w=0.943516008214
	rospy.loginfo(goal)
     	pub2.publish(goal)

	rospy.spin()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
