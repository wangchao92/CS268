from cannonDynamics import *
from utils import *
from scipy.integrate import odeint
import numpy as np

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
	z0 = np.array([x0, y0, dx0, dy0])

	sol = odeint(cannonDynamics, z0, time, args=(c,))

	idx = findGround(sol[:, 1])

	traj = {}
	traj['t'] = np.linspace(0, time[idx], nGrid)
	z = odeint(cannonDynamics, z0, traj['t'], args=(c,))
	traj['x'] = z[:, 0]
	traj['y'] = z[:, 1]
	traj['dx'] = z[:, 2]
	traj['dy'] = z[:, 3]

	return traj