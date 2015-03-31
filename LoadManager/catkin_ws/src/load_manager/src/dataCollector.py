"""
A class to collect computer activity data and, when required, save it to disk.
"""

import time
import numpy as np
import cPickle as pickle

class DataCollector:
    
    def __init__(self, filename):
        """ Initialize to empty. """

        self.initTime = time.time()
        self.activity = {}
        self.filename = filename

    def updateMachine(self, machine, raw_cpu, filtered_cpu):
        """ Keep track of machine activity with time stamps. """

        stamp = time.time() - self.initTime
        raw_cpu = float(raw_cpu)
        filtered_cpu = float(filtered_cpu)
        if machine in self.activity.keys():
            self.activity[machine]["filtered activity"].append(filtered_cpu)
            self.activity[machine]["raw activity"].append(raw_cpu)
            self.activity[machine]["time"].append(stamp)
        else:
            self.activity[machine] = {"filtered activity" : [filtered_cpu],
                                      "raw activity" : [raw_cpu],
                                      "time" : [stamp]}

    def updateProcess(self, machine, process):
        """ Record start time of a process as well as its host machine. """

        stamp = time.time() - self.initTime
        if machine in self.activity.keys():
            if (("processes" in self.activity[machine].keys()) and 
                (process in self.activity[machine]["processes"].keys())):
                self.activity[machine]["processes"][process].append(stamp)
            else:
                self.activity[machine]["processes"] = {process : [stamp]}
        else:
            self.activity[machine] = {"filtered activity" : [],
                                      "raw activity" : [],
                                      "time" : [],
                                      "processes" : {process : [stamp]}}

    def save(self):
        """ 
        Convert to numpy arrays and pickle to the specified file. 
        """

        # make a clone to preserve the original in case it's still needed
        clone = {}

        for machine in self.activity.keys():
            data = self.activity[machine].copy()
            data["filtered activity"] = np.array(data["filtered activity"], dtype=np.float)
            data["raw activity"] = np.array(data["raw activity"], dtype=np.float)
            data["time"] = np.array(data["time"], dtype=np.float)
            clone[machine] = data

        out = open(self.filename, "wb")
        pickle.dump(clone, out)
        out.close()
