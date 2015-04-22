import move_base_msgs.msg
import geometry_msgs.msg
import nav_msgs.msg
import sensor_msgs.msg as sm
import rospy

msecsdata = []
filtered = 0

def move_base_subscriber_callback(data):
	global filtered
	global msecsdata
	now = rospy.get_rostime()
	#print now.nsecs
	#print now.secs
	#print data.header.stamp.nsecs
	#print data.header.stamp.secs
	print now.nsecs-data.header.stamp.nsecs
	print now.secs-data.header.stamp.secs
	#print (now.nsecs-data.header.stamp.nsecs)/10e9
	#print (now.nsecs-data.header.stamp.nsecs)/10e9 + (now.secs-data.header.stamp.secs)
	msecs = ((now.nsecs-data.header.stamp.nsecs)/10e9 + (now.secs-data.header.stamp.secs))*1000
	msecsdata.append(msecs)
	filtered = .99*filtered + .01 * msecs
	print "unfiltered: " + str(msecs) + "filtered: " + str(filtered)

if __name__ == "__main__":

	try:
		rospy.init_node("listener", anonymous = True, disable_signals=True)
		callback = move_base_subscriber_callback
		#rospy.Subscriber("/move_base/feedback", move_base_msgs.msg.MoveBaseActionFeedback,callback)
		#rospy.Subscriber("/odom", nav_msgs.msg.Odometry,callback)
		rospy.Subscriber("/camera/depth/image_raw", sm.Image,callback)
		#rospy.Subscriber("/scan", sm.LaserScan, callback)
		#rospy.Subscriber("/camera/depth/image_raw/compressed", sm.CompressedImage, callback)
		while True:
			pass

	except KeyboardInterrupt:
		import cPickle as pickle
		import numpy as np
		global msecsdata		
		file = open("depthrawmsecsdata.pkl", 'wb')
		pickle.dump(np.asarray(msecsdata, dtype=np.float), file)
		file.close()
		sys.exit(0)
