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

# set up machines
SQUIRREL = {"usr" = "squirrel",
            "ip"  = "squirrel",
            "id"  = "10_9_160_238"}
ASDF = {"usr" = "asdf",
        "ip"  = "asdf",
        "id"  = "10_8_190_94"}
MACHINES = [SQUIRREL, ASDF]

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

def init():
    """ Launch roscore on SQUIRREL. """

    ssh = executeCommand({"machine" = SQUIRREL,
                          "command" = ROSCORE})
    process_queue.append("command" = ROSCORE,
                         "machine" = SQUIRREL,
                         "process" = ssh,
                         "isMovable" = False})

def openSSH(usr, ip):
    """ Open an SSH connection to the specified machine. """

    SSH = ["ssh", "-t", "-t", usr + "@" + ip]
    ssh = psutil.Popen(SSH, stdin=subprocess.PIPE)
    return ssh

def executeCommand(command):
    """ Execute a command on the specified machine. """

    ssh = openSSH(command["machine"]["usr"], command["machine"]["ip"])
    ssh.stdin.write(command["command"])
    time.sleep(WAIT_TIME)
    return ssh

def monitorCPUs():
    """ Launch CPU monitoring node on all machines. """

    for machine in MACHINES:
        ssh = executeCommand({"machine" = machine,
                              "command" = ACTIVITY_LAUNCH})
        process_queue.append({"command" = ACTIVITY_LAUNCH,
                              "machine" = machine,
                              "process" = ssh,
                              "isMovable" = False})

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
        
def findIdleMachine():
    """
    Return (the most) idle machine.

    As implemented, this method simply returns the machine in
    MACHINES that is currently idle, with the lowest CPU
    utilization.
    """

    lowest_cpu = float("inf")
    most_idle = None
    for machine in MACHINES:
        
        # check if idle
        if load_manager[machine]["isIdle"]:
            
            # check if lowest CPU utilization so far
            activity = load_manager[machine]["activity"].output()
            if activity < lowest_cpu:
                lowest_cpu = activity
                most_idle = machine

    # return None if no idle machines
    if not most_idle:
        return None

    else:
        return most_idle

def launchTasks():
    """
    Set up a polling loop. On each tic:
    1) Check process_queue and see if there are any processes
       running on non-idle machines. Kill those processes and add
       them back to the command_queue.
    2) Check command_queue and launch any pending commands.
    IDEA:
    3) Kill all marked processes.
    """

    while True:
        
        # check process_queue
        for task in process_queue:
            
            # check if movable
            if task["isMovable"]:
                
                # check if machine is not idle anymore and remove
                machine = task["machine"]
                if not load_data[machine]["isIdle"]:

                    # find (the most) idle machine
                    idle_machine = findIdleMachine()

                    # only move the process if there is an idle machine on the network
                    if not idle_machine:
                        command_queue.append({"command" = task["command"],
                                              "machine" = idle_machine})

                        # terminate and remove
                        task["process"].terminate() # don't bother waiting
                        process_queue.remove(task)

        # now check command_queue
        while len(command_queue) > 0:
            command = command_queue.popleft()            
            executeCommand(command)
            
        # tic
        time.sleep(UPDATE_INTERVAL)

def navigationSetup():
    """
    Add the requisite commands for autonomous navigation
    to the command queue.
    """
    
    command_queue.append({"machine" = ASDF,
                          "command" = MIN_LAUNCH})
    command_queue.append({"machine" = ASDF,
                          "command" = SENSE_LAUNCH})
    command_queue.append({"machine" = SQUIRREL,
                          "command" = AMCL_LAUNCH})
    
def mappingSetup():
    """
    Add the requisite commands for mapping to the command queue.

    Note that this will not start keyboard_teleop, since that
    requires additional user interaction (which is outside the scope
    of this module). That command may be launched on ASDF as follows:
    $ roslaunch turtlebot_teleop keyboard_teleop.launch

    Similarly, this does not start RViz. You may do so manually:
    $ roslaunch turtlebot_rviz_launchers view_navigation.launch
    """

    command_queue.append({"machine" = ASDF,
                          "command" = MIN_LAUNCH})
    command_queue.append({"machine" = ASDF,
                          "command" = MAPPING_LAUNCH})

def genericCPUCallback(data, machine):
    """
    Generic callback function for handling CPU utilization data.

    This function is extendable to future implementations of the 
    activity_monitor package, which may include data other than 
    simple CPU utilization percentage.
    """

    load_data[machine]["activity"].update(float(data.data))
    load_data[machine]["isIdle"] = isIdle(machine)
    rospy.loginfo((machine["id"] + " idle? " + str(load_data[machine]["isIdle"]) + 
                   ": " + str((load_data[machine].output(), float(data.data)))))

def onTerminateCallback(process):
    """ Callback function for process termination. """

    print("Process {} terminated".format(process))

# main script
if __name__ == "__main__":

    try:

        # launch roscore
        init()

        # set up CPU monitoring
        rospy.init_node("load_manager", anonymous=True)
        monitorCPUs()

        for machine in MACHINES:

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
