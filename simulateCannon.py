from cannonDynamics import *
from utils import *
from scipy.integrate import odeint
import numpy as np
from rk4_cannon import *

def simulateCannon(init, param):
	x0 = 0
	y0 = 0

	v0 = init['speed']
	th0 = init['angle']

	c = param['c']
	nGrid = param['nGrid']

	dx0 = v0 * math.cos(th0)
	dy0 = v0 * math.sin(th0)

	if dy0 < 0:
		raise ValueError('Cannot point cannon through ground! sin(th0) > 0 Required.')

	time = np.linspace(0, 10, 100)
	z0 = np.array([x0, y0, dx0, dy0]).reshape((4,1))

	sol = rk4_cannon(time, z0, c)

	idx = findGround(sol[1,:])

	traj = {}
	traj['t'] = np.linspace(0, time[idx], nGrid)
	z = rk4_cannon(traj['t'], z0, c)
	traj['x'] = z[0,:]
	traj['y'] = z[1,:]
	traj['dx'] = z[2,:]
	traj['dy'] = z[3,:]

	return traj