import matplotlib.pyplot as plt
import numpy             as np
import csv
from numpy import genfromtxt

from matplotlib import rc
rc('font',**{'family':'sans-serif','sans-serif':['Helvetica']})
## for Palatino and other serif fonts use:
#rc('font',**{'family':'serif','serif':['Palatino']})
rc('text', usetex=True)


#data = genfromtxt('../../build/tests/custom-clock-rate-O1-2018-08-02-td-v17-with-O0-wo-logging.csv', delimiter=',')
#data = genfromtxt('../../build/tests/custom-clock-rate-O1-2018-08-02-td-v17-wo-udp.csv', delimiter=',')
data = genfromtxt('../../build/tests/clock-rate-2018-08-02-td-v07.csv', delimiter=',')
#data = genfromtxt('../../build/tests/custom-clock-rate-O1-2018-08-02-td-v17-wo-logging.csv', delimiter=',')

fig, ax = plt.subplots(figsize=(16,16))

#ax.set_xlabel(r'Timesteps', fontsize=17)
ax.set_xlabel(r'Clock rate (Hz)', fontsize=25)
ax.set_ylabel(r'Density', fontsize=25)
#ax.set_ylabel(r'Count', fontsize=25)
#ax.set_title("Different yaw rates", fontsize=17)

plt.tick_params(axis='both', which='major', labelsize=28)

# ax.set_title("Density plot of clock rates with O(1) from the Wemding 02.08.2018 trackdrive v07", fontsize=17)

#cut = 100000

normal        = []
steering      = []
acceleration  = []
kafi          = []
vehicle_model = []
#
for frame in data:
    normal        = np.append(normal,        frame)

# normal_mean, normal_std               = np.mean(normal), np.std(normal)
# print(normal_mean)
# print(normal_std)
# normal_txt = '$\mu$: ' + str(round(normal_mean, 2)) + ', $\sigma^2$: ' + str(round(normal_std, 2))

plt.tick_params(
    axis='y',          # changes apply to the x-axis
    which='both',      # both major and minor ticks are affected
    bottom=False,      # ticks along the bottom edge are off
    top=False,         # ticks along the top edge are off
    labelbottom=False) # labels along the bottom edge are off

# from_y = -2
# to_y   = 2
# y_stepsize = 0.5

# ax.set_yticks(np.arange(from_y, to_y, y_stepsize))
# ax.set_ylim(from_y, to_y, y_stepsize)

# from_y = -1
# to_y   = 1
# y_stepsize = 0.1

# ax.set_yticks(np.arange(from_y, to_y, y_stepsize))
# ax.set_ylim(from_y, to_y, y_stepsize)

# ax.set_xticks(np.arange(0, 60000, 10000))
# ax.set_xlim(0, 60000, 10000)

# ax.set_xticks(np.arange(0, 270000, 50000))
# ax.set_xlim(            0, 270000, 50000)


#ax.set_yticks(np.arange(0, 0.00015, 0.00003))
#ax.set_ylim(0, 0.00015, 0.00003)

# ax.set_yticks(np.arange(0, 200000, 25000))
# ax.set_ylim(            0, 200000, 25000)


#ax.boxplot(normal, vert=False)
# print(normal)
# hist, bins = np.histogram(np.array(normal), bins=np.linspace(1,20,1000), density=True)

import seaborn as sns

sns.distplot(normal[4000:], hist=True, kde=True, 
             bins=50, color = 'darkblue', 
             hist_kws={'edgecolor':'black'},
             kde_kws={'linewidth': 4})
             #label=normal_txt)

#plt.hist(normal, bins=500, range=[1, 1000], histtype='step')

#plt.plot(acceleration,  color='#60dfaf', label="acceleration yaw rate, " + acceleration_txt)
#plt.plot(steering,      color='#ce6f0e', label="steering yaw rate, " + steering_txt)
#plt.plot(vehicle_model, color='#eef019', label="single track vehicle model yaw rate, " + vehicle_model_txt)
#plt.plot(normal,        color='#2038b0', label="clock frequency, " + normal_txt)
#plt.plot(kafi,          color='#26b605', label="Kalman filter yaw rate, " + kafi_txt)

#kafi_vs_acc_mean, kafi_vs_acc_std  = np.mean(kafi - acceleration), np.std(kafi - acceleration)
#kafi_vs_acc_text = '$\mu$: ' + str(round(kafi_vs_acc_mean, 2)) + ', $\sigma^2$: ' + str(round(kafi_vs_acc_std, 2))
#plt.plot(kafi - acceleration , color='#333333')
#plt.title("Difference between Kalman filter and acceleration yaw rate, " + kafi_vs_acc_text, fontsize=17)

#kafi_vs_steer_mean, kafi_vs_steer_std  = np.mean(kafi - steering), np.std(kafi - steering)
#kafi_vs_steer_text = '$\mu$: ' + str(round(kafi_vs_steer_mean, 2)) + ', $\sigma^2$: ' + str(round(kafi_vs_steer_std, 2))
#plt.plot(kafi - steering     , color='#333333')
#plt.title("Difference between Kalman filter and steering yaw rate, " + kafi_vs_steer_text, fontsize=17)

#kafi_vs_vm_mean, kafi_vs_vm_std  = np.mean(kafi - vehicle_model), np.std(kafi - vehicle_model)
#kafi_vs_vm_text = '$\mu$: ' + str(round(kafi_vs_vm_mean, 2)) + ', $\sigma^2$: ' + str(round(kafi_vs_vm_std, 2))
#plt.plot(kafi - vehicle_model, color='#333333')
#plt.title("Difference between Kalman filter and single track vehicle model yaw rate, " + kafi_vs_vm_text, fontsize=17)

# kafi_vs_nm_mean, kafi_vs_nm_std  = np.mean(kafi - normal), np.std(kafi - normal)
# kafi_vs_nm_text = '$\mu$: ' + str(round(kafi_vs_nm_mean, 4)) + ', $\sigma^2$: ' + str(round(kafi_vs_nm_std, 4))
# plt.plot(kafi - normal       , color='#333333')
# plt.title("Difference between Kalman filter and bosch yaw rate, " + kafi_vs_nm_text, fontsize=17)


plt.legend(prop={'size': 12})
plt.grid(linestyle='dotted')
plt.show()
