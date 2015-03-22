"""
Capture manual control signals from the /hey topic
"""

import rospy
from std_msgs.msg import String
import functools
import move_base_msgs.msg as mb
import tf.transformations as trans

class ManualSignal:

	def listener(self, msg):
		print "message received:" 
		self.data = msg.data
		print self.data
    
	def __init__(self):
		"""
		Set up subscriber and data structure.
		"""
		self.data = None

        # register a subscriber
		rospy.Subscriber("/hey", String, self.listener)
        #pub = rospy.Publisher('/hey', std_msgs.msg._String, queue_size=10)


 	def message(self):
		return self.data
   
	def clear(self):
		self.data = None
    
    
