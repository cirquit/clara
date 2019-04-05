import matplotlib.pyplot as plt
import numpy             as np
import csv

from matplotlib import rc
rc('font',**{'family':'sans-serif','sans-serif':['Helvetica']})
## for Palatino and other serif fonts use:
#rc('font',**{'family':'serif','serif':['Palatino']})
rc('text', usetex=True)


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

ax.set_xlabel(r'$\textbf{x}$ (meter)', fontsize=25)
ax.set_ylabel(r'$\textbf{y}$ (meter)', fontsize=25)

plt.tick_params(axis='both', which='major', labelsize=15)

#ax.set_yticks(np.arange(-60, 175, 25))
#ax.set_ylim(            -60, 175, 25)
#
#ax.set_xticks(np.arange(-120, 120, 25))
#ax.set_xlim(            -120, 120, 25)


#plt.title('Localization of 7 laps with different yaw rates', y=1.01)
#ax.set_title('Localization of 7 laps with different yaw rates', fontsize=17)
cut = 5000000


# from numpy import genfromtxt

# data = genfromtxt('../example-data/wemding-2018-08-02/trackdrive-v08/t3_darknet.log', delimiter=',')

# fig, ax = plt.subplots(figsize=(16,16))
# 
# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_title("X/Y Map")

# acc_data = []
# acc_data_2 = []

# counter = 0
# for frame in data:
#     dim      = frame[2]
#     dim_2    = frame[3]
#     if counter % 10:
#         acc_data = np.append(acc_data, dim)
#         acc_data_2 = np.append(acc_data_2, dim_2)
#     counter += 1

# ax.scatter(acc_data, acc_data_2, s = 2.5, color = 'cyan', label = 'driven pos')

prefix = "../../build/tests/"
size = 2.5

# 
acc_x_list, acc_y_list       = read_csv(prefix + 'acceleration-integrated-yaw-pos-2018-08-02-td-v17.csv')
ax.scatter(acc_x_list[:cut],    acc_y_list[:cut],    s = size, color='#60dfaf', label = 'BOSCH + Kistler')
# 
steer_x_list, steer_y_list   = read_csv(prefix + 'steering-integrated-yaw-pos-2018-08-02-td-v17.csv')
ax.scatter(steer_x_list[:cut],  steer_y_list[:cut],  s = size, color='#ce6f0e',    label = 'POSIROT + Kistler')
# 
st_x_list, st_y_list         = read_csv(prefix + 'vehicle-model-integrated-yaw-pos-2018-08-02-td-v17.csv')
ax.scatter(st_x_list[:cut],     st_y_list[:cut],     s = size, color='#eef019', label = 'Single-track model')
# 
normal_x_list, normal_y_list = read_csv(prefix + 'bosch-integrated-yaw-pos-2018-08-02-td-v17.csv')
ax.scatter(normal_x_list[:cut], normal_y_list[:cut], s = size, color='#2038b0',  label = 'BOSCH')
# 
kafi_x_list, kafi_y_list     = read_csv(prefix + 'kafi-yaw-pos-2018-08-02-td-v17.csv')
ax.scatter(kafi_x_list[:cut],   kafi_y_list[:cut],   s = size, color='#26b605', label = 'EKF')

#ax.set_xticks(np.arange(-110, 120, 25))
#ax.set_yticks(np.arange(-80, 180, 25))

#ax.set_xticks(np.arange(-15, 70, 1))
#ax.set_yticks(np.arange(-60, 10, 1))

plt.axis('scaled')
ax.grid(linestyle='dotted')
plt.legend()
ax.legend(markerscale=6)
plt.show()