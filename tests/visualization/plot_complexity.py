import matplotlib.pyplot as plt
import numpy             as np
import csv
#from numpy import genfromtxt

from operator import mul

def factorial(n):
    '''
    '''
    l = []
    acc = 1
    for i in n:
        acc = acc * i
        l = np.append(l, acc)
    return l

# 
fig, ax = plt.subplots(figsize=(16,16))
#
ax.set_xticks(np.arange(1, 100, 5))
ax.set_yticks(np.arange(1, 100, 5))
plt.xlim(0, 100)
plt.ylim(0, 100)
#
ax.set_xlabel('Complexity of the problem', fontsize=17)
ax.set_ylabel('Number of operations', fontsize=17)
ax.set_title("Time Complexity", fontsize=17)

x = np.arange(1, 100, 0.2)

constant_time  = [1] * len(x)
linear_time    = x
nlogn_time     = x * np.log(x)
squared_time   = x**2
#quadded_time   = x**3
factorial_time = factorial(x)

color_1 = '#bb133e88'
color_2 = '#00266488'
color_3 = '#99999988'
color_4 = '#4d9dbaAA'
color_5 = '#ffb60088'

lw = 5

plt.plot( x,      constant_time,       linewidth = lw, color = color_1, label = '$\mathcal{O}(1)$')
plt.plot( x,      linear_time,         linewidth = lw, color = color_2, label = '$\mathcal{O}(n)$')
plt.plot( x,      nlogn_time,          linewidth = lw, color = color_3, label = '$\mathcal{O}(n\cdot\log(n))$')
plt.plot( x[:48], squared_time[:48],   linewidth = lw, color = color_4, label = '$\mathcal{O}(n^2)$')
#plt.plot( x[:20], quadded_time[:20],   linewidth = lw, color = color_4, label = '$\mathcal{O}(n^3)$')
plt.plot( x[:15], factorial_time[:15], linewidth = lw, color = color_5, label = '$\mathcal{O}(n!)$')


font = {'family': 'sanserif',
        'color':  '#222222',
        'weight': 'normal',
        'size': 17,
        }

plt.text(50, 5, r'$\mathcal{O}(1)$', fontdict=font)
plt.text(70, 65, r'$\mathcal{O}(n)$', fontdict=font)
plt.text(11, 70, r'$\mathcal{O}(n^2)$', fontdict=font)
plt.text(30, 80, r'$\mathcal{O}(n\cdot\log(n))$', fontdict=font)
plt.text(0, 102, r'$\mathcal{O}(n!)$', fontdict=font)
# plt.text(10, 10, 'blub')

plt.grid()
# plt.legend(prop={'size': 15})

plt.show()

#acc_data_dark = []
#
#for frame_dark in data_darknet:
#    dim      = frame_dark[14]
#    #if counter % 10:
#    acc_data_dark = np.append(acc_data_dark, dim)

#plt.plot(x,  , label = 'O(1)', linewidth=lw)
#plt.plot(x, x * np.log(x), label = 'O(n*log(n))', linewidth=lw)
#plt.plot(x, x, label = 'O(n)', linewidth=lw)
#plt.plot(x, x**2, label = 'O(n^2)', linewidth=lw)
#plt.plot(x, factorial(x), label = 'O(!n)', linewidth=lw)

