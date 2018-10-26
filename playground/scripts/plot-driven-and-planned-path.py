import matplotlib.pyplot as plt
import numpy             as np
import csv
from numpy import genfromtxt
from matplotlib import cm

def rotate_by(x_pos, y_pos, angle):

    new_x_pos = x_pos * np.cos(angle) - y_pos * np.sin(angle);
    new_y_pos = y_pos * np.sin(angle) + y_pos * np.cos(angle);

    return new_x_pos, new_y_pos



# set the prefix to the sample you want to evaluate
prefix = '../loggings-1819/jesenwang-26-10-2018/stint-03'

traj_data         = genfromtxt(prefix + '/path.csv',         delimiter=',')
path_data         = genfromtxt(prefix + '/t3_tp.log',        delimiter=',')
#blue_cones_data   = genfromtxt(prefix + '/blue_cones.csv',   delimiter=',')
#yellow_cones_data = genfromtxt(prefix + '/yellow_cones.csv', delimiter=',')

#blue_cones_gps_data   = genfromtxt(prefix + '/blue_cones_gps.csv',   delimiter=',')
#yellow_cones_gps_data = genfromtxt(prefix + '/yellow_cones_gps.csv', delimiter=',')


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
# #
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


# x_blue_cones_pos = []
# y_blue_cones_pos = []

# for frame in blue_cones_data:
#     x_pos    = frame[0]
#     y_pos    = frame[1]
    
#     x_blue_cones_pos = np.append(x_blue_cones_pos, x_pos)
#     y_blue_cones_pos = np.append(y_blue_cones_pos, y_pos)


# x_yellow_cones_pos = []
# y_yellow_cones_pos = []

# for frame in yellow_cones_data:
#     x_pos    = frame[0]
#     y_pos    = frame[1]
    
#     x_yellow_cones_pos = np.append(x_yellow_cones_pos, x_pos)
#     y_yellow_cones_pos = np.append(y_yellow_cones_pos, y_pos)


# x_blue_cones_gps_pos = []
# y_blue_cones_gps_pos = []

# for frame in blue_cones_gps_data:
#     x_pos    = frame[0]
#     y_pos    = frame[1]
    
#     angle = np.arctan2(-0.0311436783522, -1.5746489875)
#     x_pos, y_pos = rotate_by(x_pos, y_pos, angle)
# #
#     x_blue_cones_gps_pos = np.append(x_blue_cones_gps_pos, x_pos)
#     y_blue_cones_gps_pos = np.append(y_blue_cones_gps_pos, y_pos)


# x_yellow_cones_gps_pos = []
# y_yellow_cones_gps_pos = []

# for frame in yellow_cones_gps_data:
#     x_pos    = frame[0]
#     y_pos    = frame[1]
#     angle = np.arctan2(-0.0311436783522, -1.5746489875)
#     x_pos, y_pos = rotate_by(x_pos, y_pos, angle)

#     x_yellow_cones_gps_pos = np.append(x_yellow_cones_gps_pos, x_pos)
#     y_yellow_cones_gps_pos = np.append(y_yellow_cones_gps_pos, y_pos)

# ax.scatter( x_blue_cones_pos, y_blue_cones_pos, s = 20, color = '#000000', label = 'blue_cones' );
# ax.scatter( x_yellow_cones_pos, y_yellow_cones_pos, s = 20, color = '#FF0000', label = 'yellow_cones' );

# ax.scatter( x_blue_cones_gps_pos - cog_x, y_blue_cones_gps_pos - cog_y, s = 20, color = '#277ece', label = 'GPS blue_cones' );
# ax.scatter( x_yellow_cones_gps_pos - cog_x, y_yellow_cones_gps_pos - cog_y, s = 20, color = '#ffc100', label = 'GPS yellow_cones' );

#ax.scatter( 0, 0, s = 40, color='violet', label = 'cog')
#
#angle = np.arctan2(-0.0311436783522, -1.5746489875)
#nose_x, nose_y = rotate_by(-1.5746489875, -0.0311436783522, angle)
#ax.scatter( nose_x, nose_y, color='violet', label = 'nose')
ax.scatter( x_traj_pos, y_traj_pos, s = 2.5, color = '#7ddc1f', label = 'planned trajectory' );

local_path_found += 0.5
local_path_found = local_path_found / local_path_found.max()
#print(local_path_found[:200])
ax.scatter( x_path_pos , y_path_pos, s = 2.5, color = cm.cool(local_path_found), label = 'driven path' );

plt.legend()
plt.grid()
plt.show()