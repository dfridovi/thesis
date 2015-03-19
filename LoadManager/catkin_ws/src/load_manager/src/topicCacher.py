"""
Cache position data.
"""

import rospy
import functools
import move_base_msgs.msg as mb
import tf.transformations as trans

class GoalTracker:

	def listener(self,msg):
		""" Listen to the topic. With each callback, cache the data. """
		self.data = msg
		print "printing self.data:"
		print self.data
	
	def __init__(self):
		"""
        Set up subscriber and data structure
        """
		self.data = None
		self.topic = "move_base/goal"
		self.dtype = mb.MoveBaseActionGoal
		rospy.Subscriber(self.topic, self.dtype, self.listener)

	def goal(self):
		return self.data
		
		
class PositionTracker:

    def listener(self, msg):
        """ Listen to the topic. With each callback, cache the data. """
        
        self.data["x"] = msg.feedback.base_position.pose.position.x
        self.data["y"] = msg.feedback.base_position.pose.position.y
        
        quat = [msg.feedback.base_position.pose.orientation.x,
                msg.feedback.base_position.pose.orientation.y,
                msg.feedback.base_position.pose.orientation.z,
                msg.feedback.base_position.pose.orientation.w]
        rpy = trans.euler_from_quaternion(quat)
        

        self.data["a"] = rpy[2]

    def __init__(self):
        """
        Set up subscriber and data structure.
        """

        self.data = {"x" : 0.0,
                     "y" : 0.0,
                     "a" : 0.0,}
        self.topic = "/move_base/feedback"
        self.dtype = mb.MoveBaseActionFeedback

        # register a subscriber
        rospy.Subscriber(self.topic, self.dtype, self.listener)

    

    def position(self):
        """ Return current position. """

        return self.data
