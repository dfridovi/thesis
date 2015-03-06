"""
A class to collect computer activity data and, when required, save it to disk.
"""

import time

class DataCollector:
    
    def __init__(self):
        """ Initialize to empty. """

        self.activity = {}
        self.processTimes = {}

    def updateMachine(self, machine, cpu):
        """ Keep track of machine activity with time stamps. """

        stamp = time.time()
        cpu = float(cpu)
        if machine in self.activity.keys():
            self.activity[machine]["activity"].append(cpu)
            self.activity[machine]["time"].append(stamp)
        else:
            self.activity[machine] = {"activity" : [cpu],
                                      "time" : [stamp]}

    def updateProcess(self, machine, process):
        """ Record start time of a process as well as its host machine. """

        stamp = time.time()
        
