"""
A class to collect computer activity data and, when required, save it to disk.
"""

import time
import numpy as np
import cPickle as pickle

class DataCollector:
    
    def __init__(self):
        """ Initialize to empty. """

        self.initTime = time.time()
        self.activity = {}

    def updateMachine(self, machine, cpu):
        """ Keep track of machine activity with time stamps. """

        stamp = time.time() - self.initTime
        cpu = float(cpu)
        if machine in self.activity.keys():
            self.activity[machine]["activity"].append(cpu)
            self.activity[machine]["time"].append(stamp)
        else:
            self.activity[machine] = {"activity" : [cpu],
                                      "time" : [stamp]}

    def updateProcess(self, machine, process):
        """ Record start time of a process as well as its host machine. """

        stamp = time.time() - self.initTime
        if machine in self.activity.keys():
            if "processes" in self.activity[machine].keys():
                self.activity[machine]["processes"][process] = stamp
            else:
                self.activity[machine]["processes"] = {process : stamp}
        else:
            self.activity[machine] = {"activity" = [],
                                      "time" = [],
                                      "processes" = {process : stamp}}

    def save(self, filename):
        """ 
        Convert to numpy arrays and pickle to the specified file. 
        """

        # make a clone to preserve the original in case it's still needed
        clone = self.activity.copy()

        for machine in clone.keys():
            data = clone[machine]
            data["activity"] = np.array(data["activity"], dtype=np.float)
            data["time"] = np.array(data["time"], dtype=np.float)

        out = open(filename, "wb")
        pickle.dump(clone, out)
        out.close()
