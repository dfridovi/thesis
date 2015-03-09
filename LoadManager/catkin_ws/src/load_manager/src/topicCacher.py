"""
Cache a ROS topic and, if required, post to that topic.
"""

import rospy
from std_msgs.msg import *
from move_base_msgs import *

class TopicCacher:
    
    def __init__(self, _topic, _dtype):
        """
        Setup subscriber on specific topic. Cache values in
        the appropriate data structure. When required, publish
        a value to that topic.
        """

        self.topic = _topic
        self.dtype = _dtype
        self.data = None
        self.pub = rospy.Publisher(self.topic, self.dtype, 
                                   queue_size=10)

        rospy.Subscriber(self.topic, self.dtype, self.listener)


    def listener(self, msg):
        """ Listen to the topic. With each callback, cache the data. """
        
        self.data = msg.data

    def publish(self, data=None):
        """ 
        Publish the input data, if it is given. If not, 
        publish the most recently cached data from this topic.
        """

        rospy.loginfo(data)
        self.pub.publish(data)
