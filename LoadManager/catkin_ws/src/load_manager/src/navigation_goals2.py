#!/usr/bin/env python

import roslib; #roslib.load_manifest('robot_red')
import rospy
import actionlib
import pdb
import time

#move_base_msgs
from move_base_msgs.msg import *

def simple_move():

    rospy.init_node('simple_move')
#pdb.set_trace()
    #Simple Action Client
#     sac = actionlib.SimpleActionClient('move_base', MoveBaseAction )
    sac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    #create goal
    goal = MoveBaseGoal()

    #use self?
    #set goal
    goal.target_pose.pose.position.x = 3.0
    goal.target_pose.pose.position.y = 0.0
    goal.target_pose.pose.orientation.w = 1.0
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()

    #start listner
    print "wait started"
    sac.wait_for_server()
    print "wait finished"

    #send goal
    sac.send_goal(goal)
#    pdb.set_trace()

    #finish
    try:
        sac.wait_for_result(rospy.Duration(100030))
        sac.cancel_goal()
#       print sac.get_result()
    except KeyboardInterrupt:
        print "Keyboard Interrupt"

if __name__ == '__main__':
    simple_move()
    
