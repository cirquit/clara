# import matplotlib.pyplot as plt
# import numpy             as np
# import csv


# def read_csv(path):
#     '''
#     '''

#     x_list = np.array([])
#     y_list = np.array([])
#     counter  = 0
#     with open(path, 'r') as csvfile:
#         reader = csv.reader(csvfile, skipinitialspace=False, delimiter=',')
#         for row in reader:
#             counter += 1
#             x = row[0]
#             y = row[1]
#             x_list = np.append(x_list, x)
#             y_list = np.append(y_list, y)
#     return x_list, y_list


# normal_vx_list, normal_vy_list = read_csv('../../build/normal-velocity.csv')
# #steer_x_list, steer_y_list = read_csv('../../build/steering-yaw-pos.csv')
# #kafi_x_list, kafi_y_list = read_csv('../../build/kafi-yaw-pos.csv')
# x = np.arange(0,len(normal_vx_list))

# # kafi_x_list, kafi_y_list = read_csv('../../build/kafi-yaw-pos.csv')

# fig, ax = plt.subplots(figsize=(16,16))

# #ax.set_xlabel('time')
# #ax.set_ylabel('m/s')
# #ax.set_title("X/Y Map")

# plt.xlim(-1000, 11000, 0.5)
# plt.ylim(-3.5, 10.0, 0.5)

# cut = 10000

# x_new = x[:cut]
# y_new = normal_vx_list[:cut]

# plt.plot(x_new, y_new, label = 'normal velocity')
# #plt.scatter(x[:cut], normal_vx_list[:cut], s=5, color='blue', alpha=0.5, label='normal-velocity')
# #ax.scatter(steer_x_list[:cut], steer_y_list[:cut], s = 2.5, color = 'red', label = 'steering calc. yaw')
# #ax.scatter(kafi_x_list[:cut], kafi_y_list[:cut], s = 2.5, color = 'green', label = 'kafi calc. yaw')

# #ax.scatter(normal_x_list[:1], normal_y_list[:1], s = 200, label = 'start', color = '#FF5500', marker='x')
# # ax.scatter(test_x_list[:cut], test_y_list[:cut], s = 2.5, label = 'regular yaw')

# #ax.set_xticks(np.arange(-20, 80, 5))
# #ax.set_yticks(np.arange(-60, 30, 5))
# #ax.grid()
# plt.legend()
# plt.show()

import numpy as np
from numpy import ma
import matplotlib.pyplot as plt
import csv

def read_csv(path):
    '''
    '''

    x_list = np.array([])
    y_list = np.array([])
    counter  = 0
    with open(path, 'r') as csvfile:
        reader = csv.reader(csvfile, skipinitialspace=False, delimiter=',')
        for row in reader:
            counter += 1
            x = float(row[0])
            y = float(row[1])
            x_list = np.append(x_list, x)
            y_list = np.append(y_list, y)
    return x_list, y_list

normal_vx_list, normal_vy_list = read_csv('../../build/normal-velocity.csv')
esitmated_vx_list, esitmated_vy_list = read_csv('../../build/estimated-velocity.csv')

fig, ax = plt.subplots(figsize=(16,16))

x = np.arange(1,10001)
y = normal_vx_list.copy()[:10000]

x2 = np.arange(1,10001)
y2 = esitmated_vx_list.copy()[:10000]

plt.step(x, y, label='normal velocity')
plt.step(x2,y2, label = 'estimated velocity')


plt.legend()

plt.xlim(-1000, 11000)
plt.ylim(-0.5, 10)

plt.show()