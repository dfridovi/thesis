#!/usr/bin/env python

"""
GUI version of load_manager.
"""


import sys
from PyQt4 import QtGui, QtCore

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
from loadManager import LoadManager

# set up machines
SQUIRREL_ID = "10_9_160_238"
ASDF_ID = "10_8_190_94"

class LoadManagerThread(QtCore.QThread):

    def __init__(self, ui):
        super(LoadManagerThread, self).__init__()
        self.ui = ui

    def run(self):
        self.ldmgr = LoadManager(self.ui)

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
        self.start.clicked.connect(self.run)
        self.start.move(FRAME_W/2 - 40,
                        FRAME_H - (190 + PROCESS_H + CPU_H + SQUARE_SIDE))

        # set up quit button
#        self.kill = QtGui.QPushButton('Exit', self)
#        self.kill.clicked.connect(self.killAll)
#        self.kill.move(FRAME_W/2 - 40,
#                       FRAME_H - (140 + PROCESS_H + CPU_H + SQUARE_SIDE))

        # set up main frame
        self.setGeometry(300, 300, FRAME_W, FRAME_H)
        self.setWindowTitle('Load Manager')
        self.show()

    def junk(self, mach, val):
        self.updateIdleness(mach, val)
        print "hi"
        time.sleep(1)
        self.junk(mach, not val)

    def run(self):
#        self.app = QtCore.QCoreApplication([])
        self.thread = LoadManagerThread(self)
#        self.thread.finished.connect(self.app.exit)
        self.thread.start()

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
            self.cpu1.insertPlainText(str(cpu) + "\n")
            self.cpu1sb.setValue(self.cpu1sb.maximum())
        if machine == ASDF_ID:
            self.cpu2.moveCursor(QtGui.QTextCursor.End)
            self.cpu2.insertPlainText(str(cpu) + "\n")
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
