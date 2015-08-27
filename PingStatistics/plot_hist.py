import cPickle as pickle
import sys
import matplotlib.pyplot as plt
import numpy as np

# plots histogram from pkl file

file1 = open(sys.argv[1], 'rb')
file2 = open(sys.argv[2], 'rb')

data1 = pickle.load(file1)
data2 = pickle.load(file2)

file1.close()
file2.close()

thresh1 = np.percentile(data1, 95.0)
thresh2 = np.percentile(data2, 95.0)
print thresh1, thresh2

n, bins, patches = plt.hist(data1[data1 <= thresh1], 20, facecolor='g', alpha=0.75)
n, bins, patches = plt.hist(data2[data2 <= thresh2], 20, facecolor='r', alpha=0.75)
plt.tick_params(labelsize=14)
plt.xlabel("Ping Times (ms)", fontsize=16)
plt.ylabel("Normalized Frequency", fontsize=16)
plt.title("Comparison of Ping Times", fontsize=18)
plt.legend(["Local Network", "Google"])
plt.grid(True)
plt.show()
