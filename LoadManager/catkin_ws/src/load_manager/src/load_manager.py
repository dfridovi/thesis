#!/usr/bin/env python

"""
Load manager package.
"""

import psutil, subprocess
import rospy
from std_msgs.msg import String
from filterCPU import FilterCPU
from collections import deque

import time, sys, os, functools

# set up shells
(RPI_USR, RPI_IP) = ("pi", "10.8.244.74")
SQUIRREL = "squirrel"
ASDF = "asdf"
SHELL_MACHINES = [SQUIRREL] #, ASDF]

# machine IP addresses
IP_SQUIRREL = "10_9_160_238"
IP_ASDF = "10_8_190_94"
IP_MACHINES = [IP_SQUIRREL] #, IP_ASDF]

# commands
ROSCORE = "roscore\n"
MIN_LAUNCH = "roslaunch turtlebot_bringup minimal.launch\n"
SENSE_LAUNCH = "roslaunch turtlebot_bringup 3dsensor_edited.launch\n"
AMCL_LAUNCH = "roslaunch turtlebot_navigation amcl_demo_edited.launch map_file:=/tmp/ee_lab_big.yaml\n"
MAPPING_LAUNCH = "roslaunch turtlebot_navigation gmapping_demo.launch\n"
ACTIVITY_LAUNCH = "rosrun activity_monitor cpu_utilization.py"

# other parameters
FILTER_TAP = 0.99
WAIT_TIME = 5

# store load data, commands, and launched processes
load_data = {}
command_queue = deque()
process_queue = deque()

def init()
    ssh = executeCommand({"usr" = SQUIRREL, "ip" = SQUIRREL,
                          "command" = ROSCORE})
    process_queue.append(ssh)

def openSSH(usr, ip):
   
    SSH = ["ssh", "-t", "-t", usr + "@" + ip]
    ssh = psutil.Popen(SSH, stdin=subprocess.PIPE)
    return ssh

def executeCommand(command):
    ssh = openSSH(command["usr"], command["ip"])
    ssh.stdin.write(command["command"])
    time.sleep(WAIT_TIME)
    return ssh

def monitorCPUs():
    """
    Launch cpu monitoring node on all machines.
    """

    for machine in SHELL_MACHINES:
        ssh = executeCommand({"usr" = machine, "ip" = machine
                              "command" = ACTIVITY_LAUNCH})
        process_queue.append(ssh)


def launchTasks():
    

def navigationSetup():
    
    command_queue.append({"usr" = ASDF, "ip" = ASDF,
                          "command" = MIN_LAUNCH})
    command_queue.append({"usr" = ASDF, "ip" = ASDF,
                          "command" = SENSE_LAUNCH})
    command_queue.append({"usr" = SQUIRREL, "ip" = SQUIRREL,
                          "command" = AMCL_LAUNCH})
    
def mappingSetup():

    command_queue.append({"usr" = ASDF, "ip" = ASDF,
                          "command" = MIN_LAUNCH})
    command_queue.append({"usr" = ASDF, "ip" = ASDF,
                          "command" = MAPPING_LAUNCH})

def genericCallback(data, machine):
    load_data[machine].update(float(data.data))
    rospy.loginfo("Logged data from " + machine + ": " + 
                  str((load_data[machine].output(), float(data.data))))

# main script
if __name__ == "__main__":
    try:

        # launch roscore
        init()

        # set up CPU monitoring
        rospy.init_node("load_manager", anonymous=True)
        monitorCPUs()

        for machine in IP_MACHINES:

            # initialize to empty filter
            load_data[machine] = FilterCPU(_tap=0.99)
            
            # generate a callback function
            callback = functools.partial(genericCallback, machine=machine)

            # subscribe
            rospy.Subscriber("cpu_util/" + machine, String, callback) 

        # set up and launch all tasks
        navigationSetup()
        launchTasks()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
