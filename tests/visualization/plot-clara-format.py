import numpy as np
import matplotlib.pyplot as plt

data = np.genfromtxt('/home/rewrite/remote-jetson/T2-PWd218/build/clara.log', delimiter=',', skip_header=10,
                     skip_footer=10, names=['distance', 'angle'])



fig = plt.figure(figsize=(13,10))

ax1 = fig.add_subplot(111)
ax1.set_title("Error logging")    
ax1.set_xlabel('time')

ax1.plot(data['distance'], color='r', label='distance')
ax1.plot(data['angle'], color='b', label='angle')
ax1.legend()
