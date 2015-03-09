"""
Cache a ROS topic and, if required, post to that topic.
"""

import rospy
from std_msgs.msg import String

class TopicCacher:
    
    def __init__(self, _topic):
        """
        Setup subscriber on specific topic. Cache values in
        the appropriate data structure. When required, publish
        a value to that topic.
        """

        self.topic = _topic
        self.data = None

        
