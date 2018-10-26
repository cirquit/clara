import matplotlib.pyplot as plt
import numpy             as np
import csv
from numpy import genfromtxt
from matplotlib import cm

# set the prefix to the sample you want to evaluate
prefix = '../example-data/hockenheim/practice-trackdrive-v10'

traj_data         = genfromtxt(prefix + '/path.csv',         delimiter=',')
path_data         = genfromtxt(prefix + '/t3_tp.log',        delimiter=',')
blue_cones_data   = genfromtxt(prefix + '/blue_cones.csv',   delimiter=',')
yellow_cones_data = genfromtxt(prefix + '/yellow_cones.csv', delimiter=',')

fig, ax = plt.subplots(figsize=(16,16))

x_traj_pos = []
y_traj_pos = []

#
for frame in traj_data:
    x_pos    = frame[0]
    y_pos    = frame[1]
    
    x_traj_pos = np.append(x_traj_pos, x_pos)
    y_traj_pos = np.append(y_traj_pos, y_pos)

x_path_pos = []
y_path_pos = []
dist_error = []
angle_error = []
local_path_found = []
#
for frame in path_data:
    x_pos    = frame[0]
    y_pos    = frame[1]
    dist     = frame[2]
    angle    = frame[3]
    local_path = frame[8]
    x_path_pos = np.append(x_path_pos, x_pos)
    y_path_pos = np.append(y_path_pos, y_pos)
    dist_error = np.append(dist_error, dist)
    angle_error = np.append(angle_error, angle)
    local_path_found = np.append(local_path_found, local_path)


x_blue_cones_pos = []
y_blue_cones_pos = []

for frame in blue_cones_data:
    x_pos    = frame[0]
    y_pos    = frame[1]
    
    x_blue_cones_pos = np.append(x_blue_cones_pos, x_pos)
    y_blue_cones_pos = np.append(y_blue_cones_pos, y_pos)


x_yellow_cones_pos = []
y_yellow_cones_pos = []

for frame in yellow_cones_data:
    x_pos    = frame[0]
    y_pos    = frame[1]
    
    x_yellow_cones_pos = np.append(x_yellow_cones_pos, x_pos)
    y_yellow_cones_pos = np.append(y_yellow_cones_pos, y_pos)

#ax.scatter( x_blue_cones_pos, y_blue_cones_pos, s = 20, color = '#277ece', label = 'blue_cones' );
#ax.scatter( x_yellow_cones_pos, y_yellow_cones_pos, s = 20, color = '#ffc100', label = 'yellow_cones' );

ax.scatter( x_traj_pos, y_traj_pos, s = 2.5, color = '#7ddc1f', label = 'planned trajectory' );

#counter = 0
#for x_path, y_path, path_found in zip(x_path_pos, y_path_pos, local_path_found):
#    #if counter %  
#    if (path_found == '1'):
#color = '#ff6347'
#    else:
#        color = '#ff1053'

local_path_found += 0.5
local_path_found = local_path_found / local_path_found.max()
#print(local_path_found[:200])
#ax.scatter( x_path_pos , y_path_pos, s = 2.5, color = cm.cool(local_path_found), label = 'driven path' );


#plt.scatter(np.arange(0, len(dist_error)), dist_error, s = 5, color = cm.cool(local_path_found), label = 'distance error')
# plt.scatter(np.arange(0, len(dist_error)), dist_error, color = cm.cool(local_path_found), label = 'angle error')

#plt.ylim(-2,2,0.5)

#plt.plot(acc_data_dark, label = 'yaw_rate is')
plt.legend()
plt.grid()
plt.show()