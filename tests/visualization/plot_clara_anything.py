import matplotlib.pyplot as plt
import numpy             as np
import csv
from numpy import genfromtxt

data = genfromtxt('../../build/something.csv', delimiter=',')
# data_darknet = genfromtxt('../example-data/wemding-2018-08-02/trackdrive-v16/t3_darknet.log', delimiter=',')

fig, ax = plt.subplots(figsize=(16,16))

# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_title("X/Y Map")

acc_data = []
acc_data_2 = []
#acc_data_3 = []
#acc_data_4 = []

counter = 0
for frame in data:
    dim      = frame[0]
    dim_2    = frame[1]
    #dim_3    = frame[6]
    #dim_4    = frame[4]
    #if counter % 10:
    acc_data = np.append(acc_data, dim)
    acc_data_2 = np.append(acc_data_2, dim_2)
    #acc_data_3 = np.append(acc_data_3, dim_3)
    #acc_data_4 = np.append(acc_data_4, dim_4)
    #counter += 1

#acc_data_dark = []
#
#for frame_dark in data_darknet:
#    dim      = frame_dark[14]
#    #if counter % 10:
#    acc_data_dark = np.append(acc_data_dark, dim)




#ax.scatter(acc_data, acc_data_2, s = 2.5)
plt.plot(acc_data_2, label = 'velocity from acceleration')
plt.plot(acc_data, label = 'velocity')
#plt.plot(acc_data_3, label = 'yaw_rate should')
#plt.plot(acc_data_4, label = 'vx')
#plt.plot(acc_data_dark, label = 'yaw_rate is')
plt.grid()
plt.legend()
plt.show()