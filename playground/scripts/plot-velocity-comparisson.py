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


data = np.genfromtxt('../../build/tests/real-velocities-2018-08-02-wemding-v17.csv', delimiter=',')

vx = []
vy = []
counter = 0
#
for frame in data:
    if (counter % 5 == 0):
        vx = np.append(vx, frame[0])
        vy = np.append(vy, frame[1])
    counter += 1

data2 = np.genfromtxt('../../build/tests/cone-velocities-2018-08-02-wemding-v17.csv', delimiter=',')

cone_vx = []
cone_vy = []
counter = 0
#
for frame in data2:
    if (counter % 5 == 0):
        cone_vx = np.append(cone_vx, frame[0])
        cone_vy = np.append(cone_vy, frame[1])
    counter += 1

data3 = np.genfromtxt('../../build/tests/acc-velocities-2018-08-02-wemding-v17.csv', delimiter=',')

acc_vx = []
acc_vy = []
counter = 0
#
for frame in data3:
    if (counter % 5 == 0):
        acc_vx = np.append(acc_vx, frame[0])
        acc_vy = np.append(acc_vy, frame[1])
    counter += 1




#normal_vx_list, normal_vy_list = read_csv('../../build/tests/real-velocities-2018-08-02-wemding-v17.csv')

#esitmated_vx_list, esitmated_vy_list = read_csv('../../build/tests/cone-velocities-2018-08-02-wemding-v17.csv')


fig, ax = plt.subplots(figsize=(16,16))

plt.tick_params(axis='both', which='major', labelsize=28)

#ax.set_xlabel(r'Timesteps', fontsize=17)
ax.set_xlabel(r'timesteps', fontsize=28)
ax.set_ylabel(r'$v_x$ in $\frac{m}{s}$', fontsize=28)
#ax.set_title("Different yaw rates", fontsize=17)

#ax.set_title("Comparisson of extracted longitudinal velocities", fontsize=)


# x = np.arange(1,10001)
# y = normal_vx_list.copy()[:10000]

# x2 = np.arange(1,10001)
# y2 = esitmated_vx_list.copy()[:10000]

cone_vx_mean, cone_vx_std               = np.mean(cone_vx), np.std(cone_vx)
cone_vx_txt = '$\mu$: ' + str(round(cone_vx_mean, 2)) + ', $\sigma^2$: ' + str(round(cone_vx_std, 2))

vx_mean, vx_std               = np.mean(vx), np.std(vx)
vx_txt = '$\mu$: ' + str(round(vx_mean, 2)) + ', $\sigma^2$: ' + str(round(vx_std, 2))

acc_vx_mean, acc_vx_std = np.mean(acc_vx), np.std(acc_vx)
acc_vx_txt = '$\mu$: ' + str(round(acc_vx_mean, 2)) + ', $\sigma^2$: ' + str(round(acc_vx_std, 2))


plt.plot(np.arange(0,len(vx)), cone_vx[:len(vx)], label = 'longitudinal visual odometry velocity') #, ' + cone_vx_txt, alpha=0.5)
plt.plot(np.arange(0,len(vx)), vx[:len(vx)], label='longitudinal Kistler velocity') #, ' + vx_txt)#, color='#00FF00')
#plt.plot(np.arange(0,len(vx)), acc_vx[:len(vx)], label='longitudinal BOSCH velocity, ' + acc_vx_txt)#, color='#00FF00')

plt.legend(prop={'size': 17})

# plt.xlim(-1000, 11000)
ax.set_yticks(np.arange(-1.0, 4, 0.5))
ax.set_ylim(-1.0, 4, 0.5)


plt.show()