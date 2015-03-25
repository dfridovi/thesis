#!/usr/bin/env python

"""
Load manager package.
"""

import time, sys, os, functools
import psutil, subprocess
import rospy
from std_msgs.msg import *
import geometry_msgs
import pubgoal
from move_base_msgs import *
from collections import deque

from filterCPU import FilterCPU
from dataCollector import DataCollector
from topicCacher import PositionTracker, GoalTracker
from manualsignals import ManualSignal


# set up machines
SQUIRREL_ID = "10_9_160_238"
ASDF_ID = "10_8_190_94"
UBUNTU_ID = "10_9_39_144"

SQUIRREL = {"usr" : "squirrel", 
            "ip"  : "squirrel", 
            "id"  : SQUIRREL_ID,
            "ssh" : None}
ASDF = {"usr" : "asdf", 
        "ip"  : "asdf", 
        "id"  : ASDF_ID,
        "ssh" : None}
UBUNTU = {"usr" : "ubuntu",
          "ip"  : "ubuntu",
          "id"  : UBUNTU_ID,
          "ssh" : None}

MACHINES = {SQUIRREL_ID : SQUIRREL, 
            ASDF_ID : ASDF} 
            
# commands
ROSCORE = "roscore\n"
MIN_LAUNCH = "roslaunch turtlebot_bringup minimal.launch\n"
SENSE_LAUNCH = "roslaunch turtlebot_bringup 3dsensor_edited.launch\n"
AMCL_LAUNCH = "roslaunch turtlebot_navigation amcl_demo_edited.launch map_file:=/tmp/ee_small.yaml\n"
MAPPING_LAUNCH = "roslaunch turtlebot_navigation gmapping_demo.launch\n"
ACTIVITY_LAUNCH = "rosrun activity_monitor cpu_utilization.py\n"
INIT_POSITION = "rosrun load_manager setPosition.py"


# other parameters
FILTER_TAP = 0.995
FILTER_INIT = 0.0
WAIT_TIME = 5
UPDATE_INTERVAL = 1
CPU_LO = 25.0
CPU_HI = 90.0
CATCH_NODES = False

# store load data, system history, commands, and launched processes
load_data = {}

position = None
goal = None
signal = None

history = DataCollector()
PATH = "/home/ubuntu/thesis/LoadManager/data/"
HIST_FILE = PATH + "history_file_" + str(time.time()) + ".pkl"

command_queue = deque()
process_queue = deque()

# system time
start_time = time.time()
SWITCH_TIME = 90000.0

def init():
    """ Create SSH shells and launch roscore on SQUIRREL."""

    for machine_id in MACHINES.keys():
        machine = MACHINES[machine_id]
        
        if machine["ssh"] is None:
            machine["ssh"] = openSSH(machine["usr"], machine["ip"])


    print "Launching roscore..."
    executeCommand({"machine" : SQUIRREL,
                    "command" : ROSCORE,
                    "catchOut" : CATCH_NODES,
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

def executeCommand(command, delay=WAIT_TIME):
    """ Execute a command on the specified machine. """

    oldssh = command["machine"]["ssh"]
    oldssh.stdin.write(command["command"])
    
    # log to history
    history.updateProcess(command["machine"]["id"], command["command"])

    # add to process_queue
    command["process"] = oldssh
    process_queue.append(command)

    # get new ssh shell for this machine
    newssh = openSSH(command["machine"]["usr"], 
                     command["machine"]["ip"],
                     command["catchOut"])
    command["machine"]["ssh"] = newssh

    # sleep
    time.sleep(delay)

def monitorCPUs():
    """ Launch CPU monitoring node on all machines. """
    
    for machine_id in MACHINES.keys():
        print "Launching CPU monitor code for machine: " + machine_id
        executeCommand({"machine"  : MACHINES[machine_id],
                        "command"  : ACTIVITY_LAUNCH,
                        "catchOut" : True, 
                        "isMovable" : False},
                       delay=0.0)


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
        if load_data[machine_id]["isIdle"]:
            
            # check if lowest CPU utilization so far
            activity = load_data[machine_id]["activity"].output()
            if activity < lowest_cpu:
                lowest_cpu = activity
                most_idle = MACHINES[machine_id]

    # return None if no idle machines
    return most_idle

def launchTasks():
    """
    Set up a polling loop. On each tic:
    1) Check process_queue and see if there are any processes
       running on non-idle machines. Mark those processes and add
       them back to the command_queue.
    2) Kill all marked processes.
    3) Check command_queue and launch any pending commands, making
       sure to edit the amcl launch file if needed.
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
                if not load_data[machine["id"]]["isIdle"] or signal.message() == "switch":
                    
                    # find (the most) idle machine
                    idle_machine = findIdleMachine()
                    signal.clear()
                    # only move the process if there is an idle machine on the network
                    if idle_machine is not None:

                        # mark and remove later
                        marked_processes.append(task)
                            
                        # change info
                        new_task = task.copy()
                        new_task["machine"] = idle_machine
                        new_task["process"] = None
                        command_queue.append(new_task)
                            
        # now kill all marked processes
        for task in marked_processes:
            print ("********************** Killing process on " + 
                   task["machine"]["id"] + ": " + task["command"])
            process_queue.remove(task)
            task["process"].terminate() # don't bother waiting

        # now check command_queue and launch
        while len(command_queue) > 0:
            command = command_queue.popleft()
            
		# adjust amcl launch file if needed
            print goal.goal()
            if command["command"] == AMCL_LAUNCH:
                initPosition(command["machine"])
                
            print ("********************** Launching process on " + 
                      command["machine"]["id"] + ": " + command["command"])     
			
            # execute
            executeCommand(command)

            if command["command"] == AMCL_LAUNCH and goal.goal() != None:
                pubgoal.simple_move(goal.goal()) 
            
        # print state of all processes
        printState()

        print "Robot At:"
        print position.position()

        # tic
        print "--------------------------------------------------------------------------"
        time.sleep(UPDATE_INTERVAL)

def initPosition(machine):
    """ 
    SSH into machine and alter amcl_demo_edited.launch file to include 
    current position as default.
    """
     
    print "Setting initial position on " + machine["id"] + " :"
    print position.position()
    x = position.position()["x"]
    y = position.position()["y"]
    a = position.position()["a"]

    print "executing command " + INIT_POSITION + " %s %s %s" % (x, y, a)
    executeCommand({"machine"  : machine,
                    "command"  : INIT_POSITION + " %s %s %s" % (x, y, a)+"\n",
                    "catchOut" : CATCH_NODES, 
                    "isMovable" : False},
                   delay=0.5)

def initPositionTopic():
    """ 
	Publish to initialPose the current position
    """
     
    print "Publising to Initial Pose"
    print position.position()
    x = position.position()["x"]
    y = position.position()["y"]
    a = position.position()["a"]


    rate = rospy.Rate(1) # 10hz
	    
    pose = geometry_msgs.msg.PoseWithCovarianceStamped()
    pose.header.frame_id = 'map'
    pose.pose.pose.position.x=-1
    pose.pose.pose.position.y=0
    pose.pose.pose.position.z=0
    pose.pose.covariance=[0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
    pose.pose.pose.orientation.z=0.0267568523876
    pose.pose.pose.orientation.w=0.999641971333


def printState():
    for task in process_queue:
        print "Machine: " + task["machine"]["id"]
        print "CPU %: " + str(load_data[task["machine"]["id"]]["activity"].output())
        print "isIdle: " + str(load_data[task["machine"]["id"]]["isIdle"])
        print "isMovable: " + str(task["isMovable"])
        print "Command: " + task["command"]

def navigationSetup():
    
    #Add the requisite commands for autonomous navigation
    #to the command queue.

    global position
    position = PositionTracker()
    global goal
    goal = GoalTracker()
    global signal	
    signal = ManualSignal()
    
    print "setting up navigation with initial pose"
    print position.position()

    command_queue.append({"machine" : ASDF,
                          "command" : MIN_LAUNCH,
                          "catchOut" : CATCH_NODES,
                          "isMovable" : False})
    command_queue.append({"machine" : ASDF,
                          "command" : SENSE_LAUNCH,
                          "catchOut" : CATCH_NODES,
                          "isMovable" : False})

    command_queue.append({"machine" : ASDF, #SQUIRREL,
                          "command" : AMCL_LAUNCH,
                          "catchOut" : CATCH_NODES,
                          "isMovable" : True})
    
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
                          "catchOut" : CATCH_NODES,
                          "isMovable" : False})
    command_queue.append({"machine" : ASDF,
                          "command" : MAPPING_LAUNCH,
                          "catchOut" : CATCH_NODES,
                          "isMovable" : True})

def genericCPUCallback(data, machine_id):
    """
    Generic callback function for handling CPU utilization data.

    This function is extendable to future implementations of the 
    activity_monitor package, which may include data other than 
    simple CPU utilization percentage.
    """

    load_data[machine_id]["activity"].update(float(data.data))
    load_data[machine_id]["isIdle"] = isIdle(machine_id)

    history.updateMachine(machine_id, 
                          load_data[machine_id]["activity"].output())
#    rospy.loginfo("CPU activity for " + machine_id + ": " + 
#                  str((load_data[machine_id]["activity"].output(), 
#                       float(data.data))))

def nice(command,niceness = 0):
    return "nice -n "+str(niceness)+" "+command 

def onTerminateCallback(process):
    """ Callback function for process termination. """

    print("Process {} terminated".format(process))

def killAll():
    """ Kill all running processes. """
    
    print "\nKeyboardInterrupt detected."
    print "Terminating all processes cleanly."
    process_list = []
    
    # gently terminate all processes on the process_queue
    for task in process_queue:
        process = task["process"]
        process_list.append(process)
        process.terminate()
    
    #Add roscore to the list of processes to terminate
    
    # wait, then forceably kill all remaining processes
    dead, alive = psutil.wait_procs(process_list, timeout=5, 
                                    callback=onTerminateCallback)
    for process in alive:
        process.kill()

    # save system history
    print "\nSaving system history to file {}".format(HIST_FILE)
    history.save(HIST_FILE)
        
# main script
if __name__ == "__main__":

    try:

        # launch roscore
        init()

        # set up CPU monitoring
        rospy.init_node("load_manager", anonymous=True, 
                        disable_signals=True)
        monitorCPUs()
        
        # set up callbacks
        callbacks = {}
        for machine_id in MACHINES.keys():
            callbacks[machine_id] = functools.partial(genericCPUCallback, 
                                                      machine_id=machine_id)
        # set ROS subscriptions
        for machine_id in MACHINES.keys():
                
            # initialize to empty filter
            load_data[machine_id] = {"activity" : FilterCPU(FILTER_INIT, 
                                                            FILTER_TAP),
                                     "isIdle" : False}
        
            # subscribe
            rospy.Subscriber("cpu_util/" + machine_id, String, 
                             callbacks[machine_id]) 
        
		
        # set up and launch all tasks
        navigationSetup()
        launchTasks()

    except KeyboardInterrupt:
      
        # terminate all processes gently
        killAll()
        sys.exit()

