#!/usr/bin/env python

"""
Broadcast CPU utilization as a ROS topic.
"""

import rospy
import psutil
from std_msgs.msg import String
import os, sys

def cpu_util():

    # set publisher, node name, and publishing rate
    IP_ADDR = os.environ["ROS_HOSTNAME"].replace(".", "_")
    pub = rospy.Publisher("cpu_util/" + IP_ADDR, String, queue_size=10)
    rospy.init_node('activity_monitor', anonymous=True)
    rate = rospy.Rate(10) # 10 hz

    # continuously publish
    while not rospy.is_shutdown():
        data = str(psutil.cpu_percent())
        rospy.loginfo(data)
        pub.publish(data)
        rate.sleep()

if __name__ == '__main__':
    try:
        cpu_util()
    except rospy.ROSInterruptException:
        pass
