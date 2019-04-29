import numpy as np
import matplotlib.pyplot as plt

# data = np.genfromtxt('/home/rewrite/remote-jetson/T2-PWd218/build/clara.log', delimiter=',', skip_header=10,
#                      skip_footer=10, names=['distance', 'angle'])

# fig = plt.figure(figsize=(13,10))

# ax1 = fig.add_subplot(111)
# ax1.set_title("Error logging")
# ax1.set_xlabel('time')

# ax1.plot(data['distance'], color='r', label='distance')
# ax1.plot(data['angle'], color='b', label='angle')
# ax1.legend()



path = '../loggings-1819/jesenwang-02-11-2018/trackdrive-v03/'

gps_data = np.genfromtxt(path + 'jesenwang_2018-11-02_line_v01.txt', delimiter=';')

pose_data = np.genfromtxt(path + 'pose.csv', delimiter=',')


fig, ax = plt.subplots(figsize=(16,16))



def rotate_by(x_pos, y_pos, angle):

    new_x_pos = x_pos * np.cos(angle) - y_pos * np.sin(angle);
    new_y_pos = x_pos * np.sin(angle) + y_pos * np.cos(angle);

    return new_x_pos, new_y_pos

#cog_x = -1.58254044387
#cog_y = -0.0374559834599
cog_x = gps_data[26][1] - gps_data[25][1]
cog_y = gps_data[26][2] - gps_data[25][2]

print(cog_x, cog_y)

gps_x_pos = []
gps_y_pos = []

for frame in gps_data:
    x_pos    = frame[1]
    y_pos    = frame[2]
    

    angle = np.arctan2(-cog_y, cog_x) - 0.075 #15
    #angle = np.arctan2(0, -1.5746489875)
    #x_pos, y_pos = rotate_by(x_pos, y_pos, from_deg(angle))
    x_pos, y_pos = rotate_by(x_pos, y_pos, angle)

    gps_x_pos = np.append(gps_x_pos, x_pos)
    gps_y_pos = np.append(gps_y_pos, y_pos)

path_x_pos = []
path_y_pos = []

for frame in pose_data:
    x_pos    = frame[0]
    y_pos    = frame[1]
    
    path_x_pos = np.append(path_x_pos, x_pos)
    path_y_pos = np.append(path_y_pos, y_pos)

# gps_x_pos = list(gps_x_pos)
# gps_y_pos = list(gps_y_pos)

# gps_x_pos.reverse()
# gps_y_pos.reverse()

gps_start = 25
gps_cut = 43

path_cut = 24000
#ax.scatter( gps_x_pos, gps_y_pos, s = 40, color = '#3b5b9d', label = 'gps path' );
ax.scatter( gps_x_pos[gps_start:gps_cut] - gps_x_pos[gps_start], gps_y_pos[gps_start:gps_cut]- gps_y_pos[gps_start], s = 40, color = '#3b5b9d', label = 'gps path' );
#ax.scatter( gps_x_pos[gps_start:gps_cut] - gps_x_pos[gps_start], gps_y_pos[gps_start:gps_cut] - gps_y_pos[gps_start], s = 40, color = '#3b5b9d', label = 'gps path' );
ax.scatter( path_x_pos[:path_cut], path_y_pos[:path_cut], s = 10, color = 'red', label = 'path' );

plt.legend()
plt.grid()
plt.show()







