#!/usr/bin/env python
import roslib
import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String)
    rospy.init_node('talker')

    while not rospy.is_shutdown():
        msg = "Hello ROS!"
        rospy.loginfo(msg)
        pub.publish(String(msg))
        rospy.sleep(1.0)


if __name__ == '__main__':
    talker()
