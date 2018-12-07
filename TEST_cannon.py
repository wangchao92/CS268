from scipy.integrate import odeint
import numpy as np
import matplotlib.pyplot as plt
import math
from cannonDynamics import *
from utils import *
from rk4_cannon import *

x0 = 0
y0 = 0

v0 = 9.0
th0 = (math.pi / 180) * 45

c = 0.4

dx0 = v0 * math.cos(th0)
dy0 = v0 * math.sin(th0)

if dy0 < 0:
	raise ValueError('Cannot point cannon through ground! sin(th0) > 0 Required.')

time = np.linspace(0, 10, 100)

z0 = np.array([x0, y0, dx0, dy0])

sol = rk4_cannon(time, z0, c)

idx = findGround(sol[:, 1])

tspan = np.linspace(0, time[idx], 100)
z = rk4_cannon(tspan, z0, c)

plt.figure()
plt.axis('equal')
groundX = np.linspace(-1, 9, 100)
groundY = np.linspace(0, 0, 100)
plt.plot(z[:, 0], z[:, 1])
plt.plot(groundX, groundY, color = 'tomato', linewidth = 4.0)
plt.show()