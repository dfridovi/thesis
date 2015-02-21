#!/usr/bin/env python

"""
Load manager package.
"""

import psutil, subprocess
import rospy
from std_msgs.msg import String
import time, sys, os, functools

from filterCPU import FilterCPU

# set up shells
(RPI_USR, RPI_IP) = ("pi", "10.8.244.74")
SQUIRREL = "squirrel"
ASDF = "asdf"

# machine IP addresses
IP_SQUIRREL = "10_9_160_238"
IP_ASDF = "10_8_190_94"
MACHINES = [IP_SQUIRREL] #, IP_ASDF]

# empty dictionary to store load data
load_data = {}

def openSSH(usr, ip):
   
    SSH = ["ssh", "-t", "-t", usr + "@" + ip]
    ssh = psutil.Popen(SSH, stdin=subprocess.PIPE)
    return ssh

def navigation():

    ROSCORE = "roscore"
    MIN_LAUNCH = "roslaunch turtlebot_bringup minimal.launch"
    SENSE_LAUNCH = "roslaunch turtlebot_bringup 3dsensor_edited.launch"
    AMCL_LAUNCH = "roslaunch turtlebot_navigation amcl_demo_edited.launch map_file:=/tmp/ee_lab_big.yaml"

    roscore = openSSH(SQUIRREL, SQUIRREL)
    roscore.stdin.write(ROSCORE + "\n")
    time.sleep(5)
    
    minlaunch = openSSH(ASDF, ASDF)
    minlaunch.stdin.write(MIN_LAUNCH + "\n")
    time.sleep(5)
    
    senselaunch = openSSH(ASDF, ASDF)
    senselaunch.stdin.write(SENSE_LAUNCH + "\n")
    time.sleep(5)

    amcllaunch = openSSH(SQUIRREL, SQUIRREL)
    amcllaunch.stdin.write(AMCL_LAUNCH + "\n")

def mapping():

    PRINTENV = "printenv"
    ROSCORE = "roscore"
    MIN_LAUNCH = "roslaunch turtlebot_bringup minimal.launch"
    MAP_LAUNCH = "roslaunch turtlebot_navigation gmapping_demo.launch"

    # "roslaunch turtlebot_rviz_launchers view_navigation.launch"

    roscore = openSSH(SQUIRREL, SQUIRREL)
    roscore.stdin.write(ROSCORE + "\n")
    time.sleep(5)
    
    minlaunch = openSSH(ASDF, ASDF)
    minlaunch.stdin.write(MIN_LAUNCH + "\n")
    time.sleep(5)
    
    maplaunch = openSSH(ASDF, ASDF)
    maplaunch.stdin.write(MAP_LAUNCH + "\n")

def genericCallback(data, machine):
    load_data[machine].update(float(data.data))
    rospy.loginfo("Logged data from " + machine + ": " + str((load_data[machine].output(), float(data.data))))

# main script
if __name__ == "__main__":
    try:
        rospy.init_node("load_manager", anonymous=True)

        for machine in MACHINES:

            # initialize to empty filter
            load_data[machine] = FilterCPU(_tap=0.99)
            
            # generate a callback function
            callback = functools.partial(genericCallback, machine=machine)

            # subscribe
            rospy.Subscriber("cpu_util/" + machine, String, callback) 

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
