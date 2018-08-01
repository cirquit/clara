import matplotlib.pyplot as plt
import numpy             as np
import csv
from numpy import genfromtxt


data = genfromtxt('../../build/yaws.csv', delimiter=',')

fig, ax = plt.subplots(figsize=(16,16))

ax.set_xlabel('Time')
ax.set_ylabel('Yaw Rate')
ax.set_title("Different Yaw Rate")

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

#ax.set_yticks(np.arange(0, 0.03, 0.001))

plt.plot(normal,        label="normal")
plt.plot(steering,      label="steering")
plt.plot(acceleration,  label="acceleration")
plt.plot(kafi,          label="kafi")
plt.plot(vehicle_model, label="vehicle model")
plt.legend()
plt.show()