from simulateCannon import *
from scipy.optimize import minimize
from cannonDynamics import *

def cannon_singleShooting(guess,target,param):
	init = {
		'speed' : guess['initSpeed'],
		'angle' : guess['initAngle']
	}
	
	P = {
		'c' : param['c'],
		'nGrid' : param['nGrid']
	}

	traj = simulateCannon(init, P)

	guess['dx0'] = traj['dx'][0]
	guess['dy0'] = traj['dy'][0]
	guess['T'] = traj['t'][-1]

	x0 = [guess['dx0'], guess['dy0']]
	tol = 0.01
	eq_cons = {'type': 'eq', 'fun': nonLinCst, 'args': (target, P, tol)}
	sol = minimize(objective, x0, method='SLSQP', constraints=[eq_cons], options={'disp': True})

	return sol

def nonLinCst(x, target, P, tol):
	x0 = 0.0
	y0 = 0.0
	dx0 = x[0]
	dy0 = x[1]

	nGrid = P['nGrid']
	t = np.linspace(0, 10, nGrid)
	z0 = np.array([x0, y0, dx0, dy0])

	sol = odeint(cannonDynamics, z0, t, args=(P['c'],))

	idx = findGround(sol[:, 1])

	t = np.linspace(0, t[idx], nGrid)
	z = odeint(cannonDynamics, z0, t, args=(P['c'],))

	dist = z[-1, 0] - target['x'] + z[-1, 1] - target['y']

	return 0.0 if abs(dist) < tol else dist

def objective(x):
	return x[0]**2 + x[1]**2