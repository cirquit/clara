import matplotlib.pyplot as plt
import numpy             as np
import csv
from numpy import genfromtxt

from matplotlib import rc
rc('font',**{'family':'sans-serif','sans-serif':['Helvetica']})
## for Palatino and other serif fonts use:
#rc('font',**{'family':'serif','serif':['Palatino']})
rc('text', usetex=True)


data = genfromtxt('../../build/tests/yaw-rate-summary-2018-08-02-td-v17.csv', delimiter=',')

fig, ax = plt.subplots(figsize=(16,16))

ax.set_xlabel(r'timesteps', fontsize=28)
ax.set_ylabel(r'yaw rate $(\frac{rad}{s})$', fontsize=28)
#ax.set_title("Different yaw rates", fontsize=17)

#cut = 100000

normal        = []
steering      = []
acceleration  = []
kafi          = []
vehicle_model = []
#
for frame in data:
    normal        = np.append(normal,        frame[0])
    steering      = np.append(steering,      frame[1])
    acceleration  = np.append(acceleration,  frame[2])
    kafi          = np.append(kafi,          frame[3])
    vehicle_model = np.append(vehicle_model, frame[4])

normal_mean, normal_std               = np.mean(normal), np.std(normal)
normal_txt = '$\mu$: ' + str(round(normal_mean, 2)) + ', $\sigma^2$: ' + str(round(normal_std, 2))

steering_mean, steering_std           = np.mean(steering), np.std(steering)
steering_txt = '$\mu$: ' + str(round(steering_mean, 2)) + ', $\sigma^2$: ' + str(round(steering_std, 2))

acceleration_mean, acceleration_std   = np.mean(acceleration), np.std(acceleration)
acceleration_txt = '$\mu$: ' + str(round(acceleration_mean, 2)) + ', $\sigma^2$: ' + str(round(acceleration_std, 2))

kafi_mean, kafi_std                   = np.mean(kafi), np.std(kafi)
kafi_txt = '$\mu$: ' + str(round(kafi_mean, 2)) + ', $\sigma^2$: ' + str(round(kafi_std, 2))

vehicle_model_mean, vehicle_model_std = np.mean(vehicle_model), np.std(vehicle_model)
vehicle_model_txt = '$\mu$: ' + str(round(vehicle_model_mean, 2)) + ', $\sigma^2$: ' + str(round(vehicle_model_std, 2))

from_y = -2
to_y   = 2
y_stepsize = 0.5

# ax.set_yticks(np.arange(from_y, to_y, y_stepsize))
# ax.set_ylim(from_y, to_y, y_stepsize)

# from_y = -1
# to_y   = 1
# y_stepsize = 0.25

ax.set_yticks(np.arange(from_y, to_y, y_stepsize))
ax.set_ylim(from_y, to_y, y_stepsize)

plt.tick_params(axis='both', which='major', labelsize=28)


plt.plot(acceleration,  color='#60dfaf', label="BOSCH + Kistler"   ) # + ", " + acceleration_txt)
plt.plot(steering,      color='#ce6f0e', label="POSIROT + Kistler" ) # + ", " + steering_txt)
plt.plot(vehicle_model, color='#eef019', label="Single-track model") # + ", " + vehicle_model_txt)
plt.plot(normal,        color='#2038b0', label="BOSCH"             ) # + ", " + normal_txt)
plt.plot(kafi,          color='#26b605', label="EKF"               ) # + ", " + kafi_txt)

# kafi_vs_acc_mean, kafi_vs_acc_std  = np.mean(kafi - acceleration), np.std(kafi - acceleration)
# kafi_vs_acc_text = '$\mu$: ' + str(round(kafi_vs_acc_mean, 2)) + ', $\sigma^2$: ' + str(round(kafi_vs_acc_std, 2))
# plt.plot(kafi - acceleration , color='#333333')
#plt.title("Difference between EKF and BOSCH + Kistler yaw rate, " + kafi_vs_acc_text, fontsize=25)

# kafi_vs_steer_mean, kafi_vs_steer_std  = np.mean(kafi - steering), np.std(kafi - steering)
# kafi_vs_steer_text = '$\mu$: ' + str(round(kafi_vs_steer_mean, 2)) + ', $\sigma^2$: ' + str(round(kafi_vs_steer_std, 2))
# plt.plot(kafi - steering     , color='#333333')
# plt.title("Difference between EKF and POSIROT + Kistler yaw rate, " + kafi_vs_steer_text, fontsize=25)

# kafi_vs_vm_mean, kafi_vs_vm_std  = np.mean(kafi - vehicle_model), np.std(kafi - vehicle_model)
# kafi_vs_vm_text = '$\mu$: ' + str(round(kafi_vs_vm_mean, 2)) + ', $\sigma^2$: ' + str(round(kafi_vs_vm_std, 2))
# plt.plot(kafi - vehicle_model, color='#333333')
# plt.title("Difference between EKF and Single-track model yaw rate, " + kafi_vs_vm_text, fontsize=25)

# kafi_vs_nm_mean, kafi_vs_nm_std  = np.mean(kafi - normal), np.std(kafi - normal)
# kafi_vs_nm_text = '$\mu$: ' + str(round(kafi_vs_nm_mean, 4)) + ', $\sigma^2$: ' + str(round(kafi_vs_nm_std, 4))
# plt.plot(kafi - normal       , color='#333333')
# plt.title("Difference between EKF and BOSCH yaw rate, " + kafi_vs_nm_text, fontsize=25)


plt.legend(prop={'size': 12})
plt.grid(linestyle='dotted')
plt.show()