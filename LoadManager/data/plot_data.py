"""
Read in activity data and plot.
"""

import numpy as np
import matplotlib.pyplot as plt
import cPickle as pickle

DATA_FILE = "./history_data_3-7-15.pkl"
file = open(DATA_FILE, "rb")
data = pickle.load(file)
file.close()

num_machines = len(data.keys())
for machine, row in zip(data.keys(), range(1, num_machines + 1)):
    
    # plot the cpu utilization data
    plt.subplot(num_machines, 1, row)
    time = data[machine]["time"]
    activity = data[machine]["activity"]

    plt.plot(time, activity)
    plt.title("Activity for Machine: %s" % machine)
    plt.xlabel("Time since program initialization (seconds)")
    plt.ylabel("CPU utilization percentage")

    # insert vertical lines for each process start
#    for process in data[machine]["processes"].keys():
#        stamp = data[machine]["processes"][process]
#        plt.axvline(stamp, color="r")

plt.show()
