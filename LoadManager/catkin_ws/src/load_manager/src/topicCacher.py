"""
Cache position data.
"""

import rospy
import functools
import move_base_msgs.msg as mb
import tf.transformations as trans

class PositionTracker:
    
    def __init__(self):
        """
        Set up subscriber and data structure.
        """

        self.data = {"x" : 0.0,
                     "y" : 0.0,
                     "a" : 0.0}
        self.topic = "/move_base/feedback"
        self.dtype = mb.MoveBaseActionFeedback

        # register a subscriber
        rospy.Subscriber(self.topic, self.dtype, listener)

    def listener(self, msg):
        """ Listen to the topic. With each callback, cache the data. """
        
        self.data["x"] = msg.data.feedback.base_position.pose.position.x
        self.data["y"] = msg.data.feedback.base_position.pose.position.y
        
        quat = [msg.data.feedback.base_position.pose.orientation.x,
                msg.data.feedback.base_position.pose.orientation.y,
                msg.data.feedback.base_position.pose.orientation.z,
                msg.data.feedback.base_position.pose.orientation.w]
        rpy = trans.euler_from_quaternion(quat)

        self.data["a"] = rpy[2]

    def position(self):
        """ Return current position. """

        return self.data
