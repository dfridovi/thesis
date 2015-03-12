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

    def publish(self):
        """ 
        For every topic, publish its most recently cached data to
        the specified destination.

        Here, we hard-code in a datatype conversion between 
        MoveBaseActionFeedback and PoseWithCovarianceStamped. 
        """

        for topic in self.data.keys():
            dest = self.data[topic]["dest"]

            if dest is not None:
                dtype_in = self.data[topic]["dtype"]
                dtype_out = self.data[dest]["dtype"]

                data_in = self.data[topic]["data"]
                if data_in is None:
                    raise Exception("No data to publish.")

                # handle identical input/output dtypes
                if dtype_in == dtype_out:
                    self.data["pub"].publish(data_in)

                # check if the types match the description above
                elif ((dtype_in == MoveBaseActionFeedback) and 
                      (dtype_out == PoseWithCovarianceStamped)):
                    
                    # -------------- FILL THIS IN -------------- #

                # if neither of the above, there's a problem
                else:
                    raise Exception("Unrecognized input/output combination.")

        
