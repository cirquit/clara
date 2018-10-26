import matplotlib.pyplot as plt
import numpy             as np
import csv
from numpy import genfromtxt
from matplotlib import cm

<<<<<<< HEAD
def rotate_by(x_pos, y_pos, angle):

    new_x_pos = x_pos * np.cos(angle) - y_pos * np.sin(angle);
    new_y_pos = x_pos * np.sin(angle) + y_pos * np.cos(angle);

    return new_x_pos, new_y_pos

def from_deg(rad):
    return (rad * 180) / np.pi


# set the prefix to the sample you want to evaluate
prefix = '../loggings-1819/jesenwang-26-10-2018/stint-02/'

#traj_data         = genfromtxt(prefix + '/path.csv',         delimiter=',')
#path_data         = genfromtxt(prefix + '/t3_tp.log',        delimiter=',')
blue_cones_data   = genfromtxt(prefix + '/blue_cones.csv',   delimiter=',')
yellow_cones_data = genfromtxt(prefix + '/yellow_cones.csv', delimiter=',')

blue_cones_gps_data   = genfromtxt(prefix + '/blue_cones_gps.csv',   delimiter=',')
yellow_cones_gps_data = genfromtxt(prefix + '/yellow_cones_gps.csv', delimiter=',')


fig, ax = plt.subplots(figsize=(16,16))

# x_traj_pos = []
# y_traj_pos = []

# #
# for frame in traj_data:
#     x_pos    = frame[0]
#     y_pos    = frame[1]
    
#     x_traj_pos = np.append(x_traj_pos, x_pos)
#     y_traj_pos = np.append(y_traj_pos, y_pos)

# x_path_pos = []
# y_path_pos = []
# dist_error = []
# angle_error = []
# local_path_found = []
# #
# for frame in path_data:
#     x_pos    = frame[0]
#     y_pos    = frame[1]
#     dist     = frame[2]
#     angle    = frame[3]
#     local_path = frame[8]
#     x_path_pos = np.append(x_path_pos, x_pos)
#     y_path_pos = np.append(y_path_pos, y_pos)
#     dist_error = np.append(dist_error, dist)
#     angle_error = np.append(angle_error, angle)
#     local_path_found = np.append(local_path_found, local_path)
=======
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
>>>>>>> 0a54cf1e1fff4bb2869432a3b24ef48aee652c93


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

<<<<<<< HEAD

x_blue_cones_gps_pos = []
y_blue_cones_gps_pos = []

for frame in blue_cones_gps_data:
    x_pos    = frame[0]
    y_pos    = frame[1]
    
    #angle = np.arctan2(-0.0311436783522, -1.5746489875)
    #angle = np.arctan2(0, -1.5746489875)
    x_pos, y_pos = rotate_by(x_pos, y_pos, from_deg(270))
#
    x_blue_cones_gps_pos = np.append(x_blue_cones_gps_pos, x_pos)
    y_blue_cones_gps_pos = np.append(y_blue_cones_gps_pos, y_pos)


x_yellow_cones_gps_pos = []
y_yellow_cones_gps_pos = []

for frame in yellow_cones_gps_data:
    x_pos    = frame[0]
    y_pos    = frame[1]
    #angle = np.arctan2(-0.0311436783522, -1.5746489875)
    #x_pos, y_pos = rotate_by(x_pos, y_pos, angle)

    x_yellow_cones_gps_pos = np.append(x_yellow_cones_gps_pos, x_pos)
    y_yellow_cones_gps_pos = np.append(y_yellow_cones_gps_pos, y_pos)



#1.38935792947,
# cog_x = 1.38935792947   + 1.13112601847
# cog_y = -0.466946133412 + 1.69169445056
# #ax.scatter( cog_x, cog_y, s = 20, color = 'violet', label = 'COG' );

# ax.scatter( 0, 0, s = 20, color = 'violet', label = 'COG' );



ax.scatter( x_blue_cones_pos, y_blue_cones_pos, s = 20, color = '#000000', label = 'blue_cones' );
#ax.scatter( x_yellow_cones_pos, y_yellow_cones_pos, s = 20, color = '#FF0000', label = 'yellow_cones' );

ax.scatter( x_blue_cones_gps_pos, y_blue_cones_gps_pos, s = 20, color = '#277ece', label = 'GPS blue_cones' );
#ax.scatter( x_yellow_cones_gps_pos, y_yellow_cones_gps_pos, s = 20, color = '#ffc100', label = 'GPS yellow_cones' );

#ax.scatter( 0, 0, s = 40, color='violet', label = 'cog')
#
#angle = np.arctan2(-0.0311436783522, -1.5746489875)
#nose_x, nose_y = rotate_by(-1.5746489875, -0.0311436783522, angle)
#ax.scatter( nose_x, nose_y, color='violet', label = 'nose')
#ax.scatter( x_traj_pos, y_traj_pos, s = 2.5, color = '#7ddc1f', label = 'planned trajectory' );
=======
#ax.scatter( x_blue_cones_pos, y_blue_cones_pos, s = 20, color = '#277ece', label = 'blue_cones' );
#ax.scatter( x_yellow_cones_pos, y_yellow_cones_pos, s = 20, color = '#ffc100', label = 'yellow_cones' );

ax.scatter( x_traj_pos, y_traj_pos, s = 2.5, color = '#7ddc1f', label = 'planned trajectory' );
>>>>>>> 0a54cf1e1fff4bb2869432a3b24ef48aee652c93

#counter = 0
#for x_path, y_path, path_found in zip(x_path_pos, y_path_pos, local_path_found):
#    #if counter %  
#    if (path_found == '1'):
#color = '#ff6347'
#    else:
#        color = '#ff1053'

<<<<<<< HEAD
# local_path_found += 0.5
# local_path_found = local_path_found / local_path_found.max()
=======
local_path_found += 0.5
local_path_found = local_path_found / local_path_found.max()
>>>>>>> 0a54cf1e1fff4bb2869432a3b24ef48aee652c93
#print(local_path_found[:200])
#ax.scatter( x_path_pos , y_path_pos, s = 2.5, color = cm.cool(local_path_found), label = 'driven path' );


#plt.scatter(np.arange(0, len(dist_error)), dist_error, s = 5, color = cm.cool(local_path_found), label = 'distance error')
# plt.scatter(np.arange(0, len(dist_error)), dist_error, color = cm.cool(local_path_found), label = 'angle error')

#plt.ylim(-2,2,0.5)

#plt.plot(acc_data_dark, label = 'yaw_rate is')
plt.legend()
plt.grid()
plt.show()