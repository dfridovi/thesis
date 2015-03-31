"""
Load manager class. Implements the same functionality as the load_manager.py
script, except wrapped into a class, so that it can be used with the LoadManagerUI.
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

from PyQt4 import QtCore

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

class LoadManager(QtCore.QObject):

    cpu_signal = QtCore.pyqtSignal((str), (float))
    process_signal = QtCore.pyqtSignal((str), (str))
    idle_signal = QtCore.pyqtSignal((str), (bool))

    def __init__(self, ui):
      
        print "hi there"

        super(LoadManager, self).__init__()
        #QtCore.QObject.__init__()

        # store the UI
        self.ui = ui

        # store load data, system history, commands, and launched processes
        self.load_data = {}

        self.position = None
        self.goal = None
        self.signal = None

        PATH = "/home/ubuntu/thesis/LoadManager/data/"
        HIST_FILE = PATH + "history_file_" + str(time.time()) + ".pkl"
        self.history = DataCollector(HIST_FILE)
        
        self.command_queue = deque()
        self.process_queue = deque()

        # system time
        self.start_time = time.time()

        # set up signals and slots with GUI
        self.cpu_signal.connect(self.ui.updateCPU)
        self.process_signal.connect(self.ui.updateProcesses)
        self.idle_signal.connect(self.ui.updateIdleness)

        # launch the load manager
        print "hi"
        self.main()
 
    def init(self):
        """ Create SSH shells and launch roscore on SQUIRREL."""

        for machine_id in MACHINES.keys():
            machine = MACHINES[machine_id]
            
            if machine["ssh"] is None:
                machine["ssh"] = self.openSSH(machine["usr"], machine["ip"])


        print "Launching roscore..."
        self.executeCommand({"machine" : SQUIRREL,
                             "command" : ROSCORE,
                             "catchOut" : CATCH_NODES,
                             "isMovable" : False})

    def openSSH(self, usr, ip, catch_output=False):
        """ Open an SSH connection to the specified machine. """

        SSH = ["ssh", "-t", "-t", usr + "@" + ip]
        if catch_output:
            ssh = psutil.Popen(SSH, stdin=subprocess.PIPE,
                               stdout=subprocess.PIPE)
        else:
            ssh = psutil.Popen(SSH, stdin=subprocess.PIPE)
        return ssh

    def executeCommand(self, command, delay=WAIT_TIME):
        """ Execute a command on the specified machine. """

        oldssh = command["machine"]["ssh"]
        oldssh.stdin.write(command["command"])
        
        # log to history
        self.history.updateProcess(command["machine"]["id"], command["command"])

        # add to process_queue
        command["process"] = oldssh
        self.process_queue.append(command)

        # get new ssh shell for this machine
        newssh = self.openSSH(command["machine"]["usr"], 
                              command["machine"]["ip"],
                              command["catchOut"])
        command["machine"]["ssh"] = newssh

        # sleep
        time.sleep(delay)

    def monitorCPUs(self):
        """ Launch CPU monitoring node on all machines. """
        
        for machine_id in MACHINES.keys():
            print "Launching CPU monitor code for machine: " + machine_id
            self.executeCommand({"machine"  : MACHINES[machine_id],
                                "command"  : ACTIVITY_LAUNCH,
                                "catchOut" : True, 
                                "isMovable" : False},
                                delay=0.0)


    def isIdle(self, machine_id):
        """
        Bang-bang control loop to avoid hysteresis. Set high and low
        thresholds of CPU activity and change load_data's "isIdle"
        parameter accordingly.
        """
        
        cpu = self.load_data[machine_id]["activity"].output()
        idle = self.load_data[machine_id]["isIdle"]
     
        if ((idle and (cpu > CPU_HI)) or ((not idle) and (cpu < CPU_LO))):
            self.idle_signal.emit(machine_id, not idle)
            return not idle
        else:
            self.idle_signal.emit(machine_id, idle)
            return idle
            
    def findIdleMachine(self):
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
            if self.load_data[machine_id]["isIdle"]:
                
                # check if lowest CPU utilization so far
                activity = self.load_data[machine_id]["activity"].output()
                if activity < lowest_cpu:
                    lowest_cpu = activity
                    most_idle = MACHINES[machine_id]

        # return None if no idle machines
        return most_idle

    def launchTasks(self):
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
            for task in self.process_queue:
                    
                # check if movable
                if task["isMovable"]:
                    
                    # check if machine is not idle anymore and remove
                    machine = task["machine"]
                    if not self.load_data[machine["id"]]["isIdle"] or self.signal.message() == "switch":
                        
                        # find (the most) idle machine
                        idle_machine = self.findIdleMachine()
                        self.signal.clear()
                        # only move the process if there is an idle machine on the network
                        if idle_machine is not None:

                            # mark and remove later
                            marked_processes.append(task)
                                
                            # change info
                            new_task = task.copy()
                            new_task["machine"] = idle_machine
                            new_task["process"] = None
                            self.command_queue.append(new_task)
                                
            # now kill all marked processes
            for task in marked_processes:
                print ("********************** Killing process on " + 
                       task["machine"]["id"] + ": " + task["command"])
                self.process_signal.emit(task["machine"]["id"], 
                                        "Killing: " + task["command"] + "\n")
                self.process_queue.remove(task)
                task["process"].terminate() # don't bother waiting

            # now check command_queue and launch
            while len(self.command_queue) > 0:
                command = self.command_queue.popleft()
                
    		    # adjust amcl launch file if needed
                print self.goal.goal()
                if command["command"] == AMCL_LAUNCH:
                    self.initPosition(command["machine"])
                    
                print ("********************** Launching process on " + 
                        command["machine"]["id"] + ": " + command["command"])     
                self.process_signal.emit(command["machine"]["id"], 
                                         "Launching: " + command["command"] + "\n")

                # execute
                self.executeCommand(command)

                if command["command"] == AMCL_LAUNCH and self.goal.goal() != None:
                    self.pubgoal.simple_move(goal.goal()) 
                
            # print state of all processes
            self.printState()

            print "Robot At:"
            print self.position.position()

            # tic
            print "--------------------------------------------------------------------------"
            time.sleep(UPDATE_INTERVAL)

    def initPosition(self, machine):
        """ 
        SSH into machine and alter amcl_demo_edited.launch file to include 
        current position as default.
        """
         
        print "Setting initial position on " + machine["id"] + " :"
        print self.position.position()
        x = self.position.position()["x"]
        y = self.position.position()["y"]
        a = self.position.position()["a"]

        print "executing command " + INIT_POSITION + " %s %s %s" % (x, y, a)
        self.executeCommand({"machine"  : machine,
                            "command"  : INIT_POSITION + " %s %s %s" % (x, y, a)+"\n",
                            "catchOut" : CATCH_NODES, 
                            "isMovable" : False},
                            delay=0.5)

    def initPositionTopic(self):
        """ 
    	Publish to initialPose the current position
        """
         
        print "Publising to Initial Pose"
        print self.position.position()
        x = self.position.position()["x"]
        y = self.position.position()["y"]
        a = self.position.position()["a"]


        rate = rospy.Rate(1) # 10hz
    	    
        pose = geometry_msgs.msg.PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'
        pose.pose.pose.position.x=-1
        pose.pose.pose.position.y=0
        pose.pose.pose.position.z=0
        pose.pose.covariance=[0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
        pose.pose.pose.orientation.z=0.0267568523876
        pose.pose.pose.orientation.w=0.999641971333


    def printState(self):
        for task in self.process_queue:
            print "Machine: " + task["machine"]["id"]
            print "CPU %: " + str(self.load_data[task["machine"]["id"]]["activity"].output())
            print "isIdle: " + str(self.load_data[task["machine"]["id"]]["isIdle"])
            print "isMovable: " + str(task["isMovable"])
            print "Command: " + task["command"]

    def navigationSetup(self):
        
        #Add the requisite commands for autonomous navigation
        #to the command queue.

        self.position = PositionTracker()
        self.goal = GoalTracker()
        self.signal = ManualSignal()
        
        print "setting up navigation with initial pose"
        print self.position.position()

        self.command_queue.append({"machine" : ASDF,
                              "command" : MIN_LAUNCH,
                              "catchOut" : CATCH_NODES,
                              "isMovable" : False})
        self.command_queue.append({"machine" : ASDF,
                              "command" : SENSE_LAUNCH,
                              "catchOut" : CATCH_NODES,
                              "isMovable" : False})

        self.command_queue.append({"machine" : ASDF, #SQUIRREL,
                                   "command" : AMCL_LAUNCH,
                                   "catchOut" : CATCH_NODES,
                                   "isMovable" : True})
        
    def mappingSetup(self):
        """
        Add the requisite commands for mapping to the command queue.

        Note that this will not start keyboard_teleop, since that
        requires additional user interaction (which is outside the scope
        of this module). That command may be launched on ASDF as follows:
        $ roslaunch turtlebot_teleop keyboard_teleop.launch

        Similarly, this does not start RViz. You may do so manually:
        $ roslaunch turtlebot_rviz_launchers view_navigation.launch
        """
    		
        self.command_queue.append({"machine" : ASDF,
                                   "command" : MIN_LAUNCH,
                                   "catchOut" : CATCH_NODES,
                                   "isMovable" : False})
        self.command_queue.append({"machine" : ASDF,
                                   "command" : MAPPING_LAUNCH,
                                   "catchOut" : CATCH_NODES,
                                   "isMovable" : True})

    def genericCPUCallback(self, data, machine_id):
        """
        Generic callback function for handling CPU utilization data.

        This function is extendable to future implementations of the 
        activity_monitor package, which may include data other than 
        simple CPU utilization percentage.
        """

        self.load_data[machine_id]["activity"].update(float(data.data))
        self.load_data[machine_id]["isIdle"] = self.isIdle(machine_id)

        self.history.updateMachine(machine_id, 
                              raw_cpu=float(data.data),
                              filtered_cpu=self.load_data[machine_id]["activity"].output())
        self.cpu_signal.emit(machine_id, 
                             self.load_data[machine_id]["activity"].output())

    #    rospy.loginfo("CPU activity for " + machine_id + ": " + 
    #                  str((load_data[machine_id]["activity"].output(), 
    #                       float(data.data))))

    def nice(self, command, niceness=0):
        return "nice -n " + str(niceness) + " " + command 

    def onTerminateCallback(self, process):
        """ Callback function for process termination. """

        print("Process {} terminated".format(process))

    def killAll(self):
        """ Kill all running processes. """
        
        print "\nKeyboardInterrupt detected."
        print "Terminating all processes cleanly."
        process_list = []
        
        # gently terminate all processes on the process_queue
        for task in self.process_queue:
            process = task["process"]
            process_list.append(process)
            process.terminate()
        
        #Add roscore to the list of processes to terminate
        
        # wait, then forceably kill all remaining processes
        dead, alive = psutil.wait_procs(process_list, timeout=5, 
                                        callback=self.onTerminateCallback)
        for process in alive:
            process.kill()

        # save system history
        print "\nSaving system history to file."
        self.history.save()
        
    # main script
    def main(self):

        try:

            # launch roscore
            self.init()

            # set up CPU monitoring
            rospy.init_node("load_manager", anonymous=True, 
                            disable_signals=True)
            self.monitorCPUs()
            
            # set up callbacks
            callbacks = {}
            for machine_id in MACHINES.keys():
                callbacks[machine_id] = functools.partial(self.genericCPUCallback, 
                                                          machine_id=machine_id)
            # set ROS subscriptions
            for machine_id in MACHINES.keys():
                    
                # initialize to empty filter
                self.load_data[machine_id] = {"activity" : FilterCPU(FILTER_INIT, FILTER_TAP),
                                              "isIdle" : False} 
            
                # subscribe
                rospy.Subscriber("cpu_util/" + machine_id, String, 
                                 callbacks[machine_id]) 
            
    		
            # set up and launch all tasks
            self.navigationSetup()
            self.launchTasks()

        except KeyboardInterrupt:
          
            # terminate all processes gently
            self.killAll()
            sys.exit()

from ldmgrGUI import LoadManagerUI
