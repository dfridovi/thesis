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
UPDATE_INTERVAL = 1
CPU_LO = 20.0
CPU_HI = 50.0

# store load data, commands, and launched processes
load_data = {}
command_queue = deque()
process_queue = deque()

def init()
    ssh = executeCommand({"usr" = SQUIRREL, "ip" = SQUIRREL,
                          "command" = ROSCORE})
    process_queue.append("command" = ROSCORE,
                         "usr" = SQUIRREL, 
                         "ip" = SQUIRREL,
                         "process" = ssh})

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
        process_queue.append({"command" = ACTIVITY_LAUNCH,
                              "usr" = machine,
                              "ip" = machine,
                              "process" = ssh})

def isIdle(machine):
    """
    Bang-bang control loop to avoid hysteresis. Set high and low
    thresholds of CPU activity and change load_data's "isIdle"
    parameter accordingly.
    """
    
    cpu = load_data[machine]["activity"].output()
    idle = load_data[machine]["isIdle"]
 
    if ((idle and (cpu > CPU_HI)) or ((not idle) and (cpu < CPU_LO))):
        return not idle
    else:
        return idle
        

def launchTasks():
    """
    Set up a polling loop. On each tic:
    1) Check process_queue and see if there are any processes
       running on non-idle machines. Kill those processes and add
       them back to the command_queue.
    2) Check command_queue and launch any pending commands.
    """

    while True:
        
        # check process_queue
        for task in process_queue:
            process = task["process"]
            

        time.sleep(UPDATE_INTERVAL)

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

def genericCPUCallback(data, machine):
    load_data[machine]["activity"].update(float(data.data))
    load_data[machine]["isIdle"] = isIdle(machine)
    rospy.loginfo((machine + " idle? " + str(load_data[machine]["isIdle"]) + 
                   ": " + str((load_data[machine].output(), float(data.data)))))

def onTerminateCallback(process):
    print("Process {} terminated".format(process))

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
            load_data[machine] = ({"activity" = FilterCPU(_tap=0.99),
                                   "isIdle" = False})
            
            # generate a callback function
            callback = functools.partial(genericCPUCallback, machine=machine)

            # subscribe
            rospy.Subscriber("cpu_util/" + machine, String, callback) 

        # set up and launch all tasks
        navigationSetup()
        launchTasks()

    except rospy.ROSInterruptException:
        pass

    except KeyboardInterrupt:
        print "Terminating all processes cleanly."
        process_list = []
        
        # gently terminate all processes on the process_queue
        for task in process_queue:
            process = task["process"]
            process_list.append(process)
            process.terminate()
        dead, alive = psutil.wait_procs(process_list, timeout=5, callback=onTerminateCallback)

        # forceably kill all remaining processes
        for process in alive:
            process.kill()
