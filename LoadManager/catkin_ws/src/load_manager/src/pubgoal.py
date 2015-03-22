#!/usr/bin/env python

import roslib;
import rospy
import actionlib
import pdb
import time

from move_base_msgs.msg import *

def simple_move(inputgoal):
    #pdb.set_trace()
    print "creating sac"
    sac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    #create goal
    goal = MoveBaseGoal()

    #use self?
    #set goal
    goal.target_pose.pose.position.x = inputgoal.goal.target_pose.pose.position.x 
    goal.target_pose.pose.position.y = inputgoal.goal.target_pose.pose.position.y
    goal.target_pose.pose.position.z = inputgoal.goal.target_pose.pose.position.z

    goal.target_pose.pose.orientation.x = inputgoal.goal.target_pose.pose.orientation.x  
    goal.target_pose.pose.orientation.y = inputgoal.goal.target_pose.pose.orientation.y
    goal.target_pose.pose.orientation.z = inputgoal.goal.target_pose.pose.orientation.z
    goal.target_pose.pose.orientation.w = inputgoal.goal.target_pose.pose.orientation.w
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()

    #start listener
    print "waiting for server started"
    sac.wait_for_server()
    print "wait finished"

    #send goal
    sac.send_goal(goal)

    #finish
    try:
        sac.wait_for_result(rospy.Duration(100030))
        sac.cancel_goal()
        print sac.get_result()
    except KeyboardInterrupt:
        print "Keyboard Interrupt"
