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

normal_vx_list, normal_vy_list = read_csv('../../build/normal-velocity.csv')
esitmated_vx_list, esitmated_vy_list = read_csv('../../build/estimated-velocity.csv')

fig, ax = plt.subplots(figsize=(16,16))

x = np.arange(1,10001)
y = normal_vx_list.copy()[:10000]

x2 = np.arange(1,10001)
y2 = esitmated_vx_list.copy()[:10000]

plt.step(x, y, label='normal velocity')
plt.step(x2,y2, label = 'estimated velocity')

plt.legend()

plt.xlim(-1000, 11000)
plt.ylim(-0.5, 10)

plt.show()