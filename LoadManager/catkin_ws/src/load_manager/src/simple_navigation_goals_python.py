#! /usr/bin/env python

import roslib
roslib.load_manifest('my_pkg_name')
import rospy
from actionlib import SimpleActionClient
import rospy

from chores.msg import DoDishesAction, DoDishesGoal

from move_base_msgs import msg

if __name__ == '__main__':
    rospy.init_node('simple_navigation_goals')
    # create a client
    client = SimpleActionClient('simple_navigation_goals', msg.MoveBaseAction)
    #Wait for the server to come up
    client.wait_for_server()
    
    ac = SimpleActionClient()
    
    #Create a goal to send
    goal = msg.MoveBaseActionGoal()

    goal.goal.target_pose.header.frame_id = "base_link";
    goal.goal.target_pose.header.stamp = rospy.get_rostime()

    goal.goal.target_pose.pose.position.x = 1.0
    goal.goal.target_pose.pose.orientation.w = 1.0
    
    # Send the goal 
    rospy.loginfo("Sending Pythonic Goal")
    client.send_goal(goal)
    
    # Wait around a bit
    client.wait_for_result() 

    # Print out the results
    res = client.getState()
    
    print "Has res (%s) succeeded? %s" % (res, actionlib.SimpleGoalState.DONE)

    
