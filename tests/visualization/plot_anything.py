import matplotlib.pyplot as plt
import numpy             as np
import csv
from numpy import genfromtxt

data = genfromtxt('../example-data/wemding-2018-07-31/logging-v02/t3_darknet.log', delimiter=',')

fig, ax = plt.subplots(figsize=(16,16))

# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_title("X/Y Map")

acc_data = []

for frame in data:
    dim      = frame[13]
    acc_data = np.append(acc_data, dim)

plt.plot(acc_data)
plt.show()