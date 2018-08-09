import matplotlib.pyplot as plt
import numpy             as np
import matplotlib.mlab as mlab
import math 

loc, scale = 0., 1.
s = np.random.laplace(loc, scale, 1000)

# count, bins, ignored = plt.hist(s, 30, normed=True)
x = np.arange(-8., 8., .01)
pdf = np.exp(-abs(x-loc)/scale)/(2.*scale)
plt.plot(x, pdf, label='laplace distribution')


mu = 0
variance = 3

sigma = math.sqrt(variance)
x = np.linspace(mu - 3*sigma, mu + 3*sigma, 1000)


plt.title('Difference between Laplacian and Gaussian Distribution')

plt.plot(x,mlab.normpdf(x, mu, sigma), label='gaussian distribution')
plt.legend()
plt.show()