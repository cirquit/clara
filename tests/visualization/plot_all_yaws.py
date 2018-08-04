import matplotlib.pyplot as plt
import numpy             as np
import csv
from numpy import genfromtxt


data = genfromtxt('../../build/yaws.csv', delimiter=',')

fig, ax = plt.subplots(figsize=(16,16))

ax.set_xlabel('Time', fontsize=17)
ax.set_ylabel('Yaw rate', fontsize=17)
ax.set_title("Different yaw rates", fontsize=17)

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


#ax.set_yticks(np.arange(0, 0.03, 0.001))
ax.set_yticks(np.arange(-2, 2, 0.5))
ax.set_ylim(-2,2,0.5)

plt.plot(acceleration,  color='#2038b0', label="acceleration calc. yaw rate, " + acceleration_txt)
plt.plot(steering,      color='#ce6f0e', label="steering calc. yaw rate, " + steering_txt)
plt.plot(vehicle_model, color='#eef019', label="single track calc. yaw rate, " + vehicle_model_txt)
plt.plot(normal,        color='#60dfaf', label="regular yaw rate, " + normal_txt)
plt.plot(kafi,          color='#26b605', label="kafi calc. yaw rate, " + kafi_txt)
plt.legend(prop={'size': 12})
plt.show()