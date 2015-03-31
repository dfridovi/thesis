#!/usr/bin/env python

"""
GUI version of load_manager.
"""


import sys
from PyQt4 import QtGui

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

import threading

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

class LoadManagerUI(QtGui.QWidget):
    
    def __init__(self):
        super(LoadManagerUI, self).__init__()
        self.initUI()
                
    def initUI(self):      

        FRAME_H = 650
        FRAME_W = 1000
        SQUARE_SIDE = 100
        TEXT_W = 400
        CPU_H = 50
        PROCESS_H = 300

        self.red = QtGui.QColor(255, 0, 0)
        self.green = QtGui.QColor(0, 255, 0)

        # set up color coded squares for idleness
        self.square1lbl = QtGui.QLabel('Idle?', self)
        self.square1lbl.move(FRAME_W/4 - SQUARE_SIDE/2 - 40, 
                             FRAME_H - (55 + SQUARE_SIDE/2))
        self.square1 = QtGui.QFrame(self)
        self.square1.setGeometry(FRAME_W/4 - SQUARE_SIDE/2, 
                                 FRAME_H - (50+SQUARE_SIDE), 
                                 SQUARE_SIDE, SQUARE_SIDE)
        self.square1.setStyleSheet("QWidget { background-color: %s }" %  
                                   self.green.name())

        self.square2lbl = QtGui.QLabel('Idle?', self)
        self.square2lbl.move(FRAME_W*3/4 - SQUARE_SIDE/2 - 40, 
                             FRAME_H - (55 + SQUARE_SIDE/2))
        self.square2 = QtGui.QFrame(self)
        self.square2.setGeometry(FRAME_W*3/4 - SQUARE_SIDE/2, 
                                 FRAME_H - (50+SQUARE_SIDE), 
                                 SQUARE_SIDE, SQUARE_SIDE)
        self.square2.setStyleSheet("QWidget { background-color: %s }" %  
                                   self.green.name())
        
        # set up text boxes for CPU monitoring
        self.cpu1lbl = QtGui.QLabel('CPU Percentage', self)
        self.cpu1lbl.move(FRAME_W/4 - TEXT_W/2, 
                          FRAME_H - (120 + CPU_H + SQUARE_SIDE))
        self.cpu1 = QtGui.QTextEdit(self)
        self.cpu1.setReadOnly(True)
        self.cpu1.setLineWrapMode(QtGui.QTextEdit.NoWrap);
        self.cpu1.setGeometry(FRAME_W/4 - TEXT_W/2, 
                              FRAME_H - (100 + CPU_H + SQUARE_SIDE), 
                              TEXT_W, CPU_H)

        self.cpu2lbl = QtGui.QLabel('CPU Percentage', self)
        self.cpu2lbl.move(FRAME_W*3/4 - TEXT_W/2, 
                          FRAME_H - (120 + CPU_H + SQUARE_SIDE))
        self.cpu2 = QtGui.QTextEdit(self)
        self.cpu2.setReadOnly(True)
        self.cpu2.setLineWrapMode(QtGui.QTextEdit.NoWrap);
        self.cpu2.setGeometry(FRAME_W*3/4 - TEXT_W/2, 
                              FRAME_H - (100 + CPU_H + SQUARE_SIDE), 
                              TEXT_W, CPU_H)

        self.cpu1sb = self.cpu1.verticalScrollBar()
        self.cpu2sb = self.cpu2.verticalScrollBar()

        # set up text boxes for process monitoring
        self.process1lbl = QtGui.QLabel('Active Processes', self)
        self.process1lbl.move(FRAME_W/4 - TEXT_W/2, 
                              FRAME_H - (170 + PROCESS_H + CPU_H + SQUARE_SIDE))
        self.process1 = QtGui.QTextEdit(self)
        self.process1.setReadOnly(True)
        self.process1.setLineWrapMode(QtGui.QTextEdit.NoWrap);
        self.process1.setGeometry(FRAME_W/4 - TEXT_W/2, 
                                  FRAME_H - (150 + PROCESS_H + CPU_H + SQUARE_SIDE), 
                                  TEXT_W, PROCESS_H)

        self.process2lbl = QtGui.QLabel('Active Processes', self)
        self.process2lbl.move(FRAME_W*3/4 - TEXT_W/2, 
                              FRAME_H - (170 + PROCESS_H + CPU_H + SQUARE_SIDE))
        self.process2 = QtGui.QTextEdit(self)
        self.process2.setReadOnly(True)
        self.process2.setLineWrapMode(QtGui.QTextEdit.NoWrap);
        self.process2.setGeometry(FRAME_W*3/4 - TEXT_W/2, 
                                  FRAME_H - (150 + PROCESS_H + CPU_H + SQUARE_SIDE), 
                                  TEXT_W, PROCESS_H)

        self.process1sb = self.process1.verticalScrollBar()
        self.process2sb = self.process2.verticalScrollBar()

        # set machine names
        self.machine1lbl = QtGui.QLabel('SQUIRREL', self)
        self.machine1lbl.move(FRAME_W/4 - 40, 
                              FRAME_H - (190 + PROCESS_H + CPU_H + SQUARE_SIDE))

        self.machine2lbl = QtGui.QLabel('ASDF', self)
        self.machine2lbl.move(FRAME_W*3/4 - 20, 
                              FRAME_H - (190 + PROCESS_H + CPU_H + SQUARE_SIDE))

        # set up start button
        self.start = QtGui.QPushButton('Start!', self)
        self.start.clicked.connect(self.launchtest)
        self.start.move(FRAME_W/2 - 40,
                        FRAME_H - (190 + PROCESS_H + CPU_H + SQUARE_SIDE))

        # set up main frame
        self.setGeometry(300, 300, FRAME_W, FRAME_H)
        self.setWindowTitle('Load Manager')
        self.show()

    def junk(self, mach, val):
        self.updateIdleness(mach, val)
        time.sleep(1)

    def launchtest(self):
        self.junk(SQUIRREL_ID, False)
        print "1"
        self.junk(ASDF_ID, False)
        print "2"

    def launch(self):
        """ Launch load manager. """

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

        # launch the load manager
        try:

            # launch roscore
            self.initLoadManager()

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

    def initLoadManager(self):
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
            #self.updateIdleness(machine_id, not idle)
            return not idle
        else:
            #self.updateIdleness(machine_id, idle)
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
                #self.updateProcesses(task["machine"]["id"], "Killing: " + task["command"] + "\n")
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
                #self.updateProcesses(command["machine"]["id"], "Launching: " + command["command"] + "\n")          
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
        self.updateCPU(machine_id, str(self.load_data[machine_id]["activity"].output()) + "\n")

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


    def updateIdleness(self, machine, isIdle):
        """ Change square colors to match machine idleness. """

        if (machine == SQUIRREL_ID) and isIdle:
            self.square1.setStyleSheet("QWidget { background-color: %s }" %  
                                       self.green.name())                
        if (machine == SQUIRREL_ID) and not isIdle:
            self.square1.setStyleSheet("QWidget { background-color: %s }" %  
                                       self.red.name())                
        if (machine == ASDF_ID) and isIdle:
            self.square2.setStyleSheet("QWidget { background-color: %s }" %  
                                       self.green.name())                
        if (machine == ASDF_ID) and not isIdle:
            self.square2.setStyleSheet("QWidget { background-color: %s }" %  
                                       self.red.name())                

    def updateCPU(self, machine, cpu):
        """ Change text of cpu box. """

        if machine == SQUIRREL_ID:
            self.cpu1.moveCursor(QtGui.QTextCursor.End)
            self.cpu1.insertPlainText(str(cpu))
            self.cpu1sb.setValue(self.cpu1sb.maximum())
        if machine == ASDF_ID:
            self.cpu2.moveCursor(QtGui.QTextCursor.End)
            self.cpu2.insertPlainText(str(cpu))
            self.cpu2sb.setValue(self.cpu1sb.maximum())
    
    def updateProcesses(self, machine, process):
        """ Change text of processes box. """

        if machine == SQUIRREL_ID:
            self.process1.moveCursor(QtGui.QTextCursor.End)
            self.process1.insertPlainText(process)
            self.process1sb.setValue(self.process1sb.maximum())
        if machine == ASDF_ID:
            self.process2.moveCursor(QtGui.QTextCursor.End)
            self.process2.insertPlainText(process)
            self.process2sb.setValue(self.process2sb.maximum())
            
def main():
    app = QtGui.QApplication(sys.argv)
    ex = LoadManagerUI()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()    
