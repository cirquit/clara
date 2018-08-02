import matplotlib.pyplot as plt
import numpy             as np
import csv
# from numpy import genfromtxt

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
            x = row[0]
            y = row[1]
            x_list = np.append(x_list, x)
            y_list = np.append(y_list, y)
    return x_list, y_list


fig, ax = plt.subplots(figsize=(16,16))

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_title("Positional Map")

cut = 1000000


from numpy import genfromtxt

data = genfromtxt('../example-data/wemding-2018-08-02/trackdrive-v05/t3_darknet.log', delimiter=',')

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

ax.scatter(acc_data, acc_data_2, s = 2.5, color = 'cyan', label = 'driven pos')


normal_x_list, normal_y_list = read_csv('../../build/normal-pos-yaw.csv')
ax.scatter(normal_x_list[:cut], normal_y_list[:cut], s = 2.5,                   label = 'regular yaw')

#steer_x_list, steer_y_list   = read_csv('../../build/steering-pos-yaw.csv')
#ax.scatter(steer_x_list[:cut],  steer_y_list[:cut],  s = 2.5, color = 'red',    label = 'steering calc. yaw')
# 
#acc_x_list, acc_y_list       = read_csv('../../build/acc-pos-yaw.csv')
#ax.scatter(acc_x_list[:cut],    acc_y_list[:cut],    s = 2.5, color = 'orange', label = 'acc. yaw')
# 
kafi_x_list, kafi_y_list     = read_csv('../../build/kafi-pos-yaw.csv')
ax.scatter(kafi_x_list[:cut],   kafi_y_list[:cut],   s = 2.5, color = 'green',  label = 'kafi calc. yaw')
# 
# st_x_list, st_y_list         = read_csv('../../build/st-yaw-pos.csv')
# ax.scatter(st_x_list[:cut],     st_y_list[:cut],     s = 2.5, color = 'violet', label = 'st. yaw')

#ax.set_xticks(np.arange(-20, 80, 5))
#ax.set_yticks(np.arange(-60, 30, 5))

ax.grid()
plt.legend()
plt.show()