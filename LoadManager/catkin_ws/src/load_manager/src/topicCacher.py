"""
Cache a set of ROS topics and, if required, post data to each topic.

Note: this is currently only intended to transfer data from the current
position /move_base/feedback to /initialpose, and from /move_base/goal
to itself.
"""

import rospy
import functools
from std_msgs.msg import *
from move_base_msgs import *

class TopicCacher:
    
    def __init__(self, _topics):
        """
        Setup subscriber on specific topics. Cache values in
        the appropriate data structures.
        """

        self.data = {}

        # set up data structure to handle all this information
        for topic in _topics.keys():
            self.data[topic] = {}
            self.data[topic]["dtype"] = _topics[topic]["dtype"]
            self.data[topic]["data"] = None
            self.data[topic]["dest"] = _topics[topic]["dest"]

        # create a publisher for every "dest" entry
        for topic in self.data.keys():
            dest = self.data[topic]["dest"]
            if dest is not None:
                self.data[topic]["pub"] = rospy.Publisher(dest, 
                                                          self.data[dest]["dtype"], 
                                                          queue_size=10)

        # register a subscriber for every topic
        for topic in self.data.keys():
            rospy.Subscriber(topic, self.data[topic]["dtype"], 
                             functools.partial(listener, topic=topic))


    def listener(self, msg, topic):
        """ Listen to the topic. With each callback, cache the data. """
        
        self.data[topic]["data"] = msg.data

##### CHANGE THIS TO HANDLE NEW ARCHITECTURE #####

    def publish(self, data=None):
        """ 
        Publish the input data, if it is given. If not, 
        publish the most recently cached data from this topic.
        """

        rospy.loginfo(data)
        self.pub.publish(data)
