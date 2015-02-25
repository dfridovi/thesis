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
SQUIRREL_ID = "10_9_160_238"
ASDF_ID = "10_8_190_94"

SQUIRREL = {"usr" : "squirrel", 
            "ip"  : "squirrel", 
            "id"  : SQUIRREL_ID}
ASDF = {"usr" : "asdf", 
        "ip"  : "asdf", 
        "id"  : ASDF_ID}

MACHINES = {SQUIRREL_ID : SQUIRREL} #, 
#            ASDF_ID : ASDF}

# commands
ROSCORE = "roscore\n"
MIN_LAUNCH = "roslaunch turtlebot_bringup minimal.launch\n"
SENSE_LAUNCH = "roslaunch turtlebot_bringup 3dsensor_edited.launch\n"
AMCL_LAUNCH = "roslaunch turtlebot_navigation amcl_demo_edited.launch map_file:=/tmp/ee_lab_big.yaml\n"
MAPPING_LAUNCH = "roslaunch turtlebot_navigation gmapping_demo.launch\n"
ACTIVITY_LAUNCH = "rosrun activity_monitor cpu_utilization.py\n"

# other parameters
FILTER_TAP = 0.99
WAIT_TIME = 5
UPDATE_INTERVAL = 1
CPU_LO = 20.0
CPU_HI = 50.0
CATCH_NODES = True

# store load data, commands, and launched processes
load_data = {}
command_queue = deque()
process_queue = deque()

def init():
    """ Launch roscore on SQUIRREL. """

    ssh = executeCommand({"machine" : SQUIRREL,
                          "command" : ROSCORE,
                          "catchOut" : False})
    process_queue.append({"command" : ROSCORE,
                          "machine" : SQUIRREL,
                          "process" : ssh,
                          "isMovable" : False})

def openSSH(usr, ip, catch_output=False):
    """ Open an SSH connection to the specified machine. """

    SSH = ["ssh", "-t", "-t", usr + "@" + ip]
    if catch_output:
        ssh = psutil.Popen(SSH, stdin=subprocess.PIPE,
                           stdout=subprocess.PIPE)
    else:
        ssh = psutil.Popen(SSH, stdin=subprocess.PIPE)
    return ssh

def executeCommand(command):
    """ Execute a command on the specified machine. """

    ssh = openSSH(command["machine"]["usr"], 
                  command["machine"]["ip"],
                  command["catchOut"])
    ssh.stdin.write(command["command"])
    time.sleep(WAIT_TIME)
    return ssh

def monitorCPUs():
    """ Launch CPU monitoring node on all machines. """

    for machine_id in MACHINES.keys():
        ssh = executeCommand({"machine"  : MACHINES[machine_id],
                              "command"  : ACTIVITY_LAUNCH,
                              "catchOut" : True})
        process_queue.append({"command" : ACTIVITY_LAUNCH,
                              "machine" : MACHINES[machine_id],
                              "process" : ssh,
                              "isMovable" : False})

def isIdle(machine_id):
    """
    Bang-bang control loop to avoid hysteresis. Set high and low
    thresholds of CPU activity and change load_data's "isIdle"
    parameter accordingly.
    """
    
    cpu = load_data[machine_id]["activity"].output()
    idle = load_data[machine_id]["isIdle"]
 
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
    for machine_id in MACHINES.keys():
        
        # check if idle
        if load_manager[machine_id]["isIdle"]:
            
            # check if lowest CPU utilization so far
            activity = load_manager[machine_id]["activity"].output()
            if activity < lowest_cpu:
                lowest_cpu = activity
                most_idle = MACHINES[machine_id]

    # return None if no idle machines
    if not most_idle:
        return None
    else:
        return most_idle

def launchTasks():
    """
    Set up a polling loop. On each tic:
    1) Check process_queue and see if there are any processes
       running on non-idle machines. Mark those processes and add
       them back to the command_queue.
    2) Check command_queue and launch any pending commands.
    3) Kill all marked processes.
    """

    while True:

        # keep a list of processes to be killed
        marked_processes = []

        # check process_queue
        for task in process_queue:
            
            # check if movable
            if task["isMovable"]:
                
                # check if machine is not idle anymore and remove
                machine = task["machine"]
                if not load_data[machine["id"]]["isIdle"]:

                    # find (the most) idle machine
                    idle_machine = findIdleMachine()

                    # only move the process if there is an idle machine on the network
                    if not idle_machine:
                        command_queue.append({"command" : task["command"],
                                              "machine" : idle_machine})

                        # mark and remove
                        marked_processes.append(task)
                        process_queue.remove(task)

        # now check command_queue
        while len(command_queue) > 0:
            command = command_queue.popleft()            
            executeCommand(command)

        # now kill all marked processes
        for task in marked_processes:
            task["process"].terminate() # don't bother waiting
                        
            
        # tic
        time.sleep(UPDATE_INTERVAL)

def navigationSetup():
    """
    Add the requisite commands for autonomous navigation
    to the command queue.
    """
    
    command_queue.append({"machine" : ASDF,
                          "command" : MIN_LAUNCH,
                          "catchOut" : CATCH_NODES})
    command_queue.append({"machine" : ASDF,
                          "command" : SENSE_LAUNCH,
                          "catchOut" : CATCH_NODES})
    command_queue.append({"machine" : SQUIRREL,
                          "command" : AMCL_LAUNCH,
                          "catchOut" : CATCH_NODES})
    
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

    command_queue.append({"machine" : ASDF,
                          "command" : MIN_LAUNCH,
                          "catchOut" : CATCH_NODES})
    command_queue.append({"machine" : ASDF,
                          "command" : MAPPING_LAUNCH,
                          "catchOut" : CATCH_NODES})

def genericCPUCallback(data, machine_id):
    """
    Generic callback function for handling CPU utilization data.

    This function is extendable to future implementations of the 
    activity_monitor package, which may include data other than 
    simple CPU utilization percentage.
    """

    load_data[machine_id]["activity"].update(float(data.data))
    load_data[machine_id]["isIdle"] = isIdle(machine_id)
    rospy.loginfo("CPU activity for " + machine_id + ": " + 
                  str((load_data[machine_id]["activity"].output(), float(data.data))))

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

        # set up callbacks
        callbacks = {}
        for machine_id in MACHINES.keys():
            callbacks[machine_id] = functools.partial(genericCPUCallback, 
                                                      machine_id=machine_id)

        # set ROS subscribers
        for machine_id in MACHINES.keys():
            print machine_id

            # initialize to empty filter
            load_data[machine_id] = {"activity" : FilterCPU(_tap=0.99),
                                     "isIdle" : False}
            
            # subscribe
            rospy.Subscriber("cpu_util/" + machine_id, String, callbacks[machine_id]) 

        # set up and launch all tasks
#        navigationSetup()
        launchTasks()

    except rospy.ROSInterruptException:
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
            
        sys.exit()

    except KeyboardInterrupt:
        print "KeyboardInterrupt detected."
        sys.exit()
