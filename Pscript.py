#!/usr/bin/python

import numpy as np
import matplotlib.pylab as plt
from odetrail1 import ode45
from odetrail1 import ode45_step
from odetrail1 import f


b = 0.25
c = 5.0    

N = 101

x0 = np.array([np.pi - 0.1, 0.0])
t = np.linspace(0, 10, N)

x = ode45(f, t, x0, b, c)

plt.plot(x)
plt.show()

