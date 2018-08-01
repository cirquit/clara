import matplotlib.pyplot as plt
import numpy             as np
import csv
from numpy import genfromtxt

data = genfromtxt('../example-data/wemding-2018-08-01/trackdrive-v02/t3_darknet.log', delimiter=',')

fig, ax = plt.subplots(figsize=(16,16))

# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_title("X/Y Map")

acc_data = []
acc_data_2 = []

counter = 0
for frame in data:
    dim      = frame[2]
    dim_2    = frame[3]
    if counter % 10:
        acc_data = np.append(acc_data, dim)
        acc_data_2 = np.append(acc_data_2, dim_2)
    counter += 1

ax.scatter(acc_data, acc_data_2, s = 2.5)
# plt.plot(acc_data)
# plt.plot(acc_data_2)
plt.show()