import cPickle as pickle
import sys
import matplotlib.pyplot as plt
import numpy as np

# plots histogram from pkl file

file = open(sys.argv[1], 'rb')
data = pickle.load(file)
file.close()

thresh = np.percentile(data, 94.0)
print thresh

n, bins, patches = plt.hist(data[data <= thresh], 20, facecolor='g', alpha=0.75)
plt.xlabel("Message Age Distribution (ms)")
plt.ylabel('Frequency')
plt.title('Message Age of /camera/depth/image_raw/compressed')
plt.grid(True)
plt.show()
