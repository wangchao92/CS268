from simulateCannon import *
from scipy.optimize import minimize
from cannonDynamics import *
from rk4_cannon import *

def cannon_singleShooting_cons(guess, target, param):
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

	x0 = [guess['dx0'], guess['dy0']]

	eq_cons = {'type': 'eq', 'fun': nonLinCst, 'args': (target, P)}
	sol = minimize(objective, x0, method='SLSQP', constraints=[eq_cons], options={'disp': True})

	return sol

def nonLinCst(x, target, P):
	x0 = 0.0
	y0 = 0.0
	dx0 = x[0]
	dy0 = x[1]

	nGrid = P['nGrid']
	t = np.linspace(0, 10, nGrid)
	z0 = np.array([x0, y0, dx0, dy0]).reshape((4,1))

	sol = rk4_cannon(t, z0, P['c'])

	idx = findGround(sol[1,:])

	t = np.linspace(0, t[idx], nGrid)
	z = rk4_cannon(t, z0, P['c'])

	dist = z[0, -1] - target['x'] + z[1, -1] - target['y']

	return dist

def objective(x):
	return x[0]**2 + x[1]**2

def cannon_singleShooting_uncons(guess, target, param):
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

	x0 = [guess['dx0'], guess['dy0']]

	sol = minimize(objective_uncons, x0, args = (target, P), method='Nelder-Mead', options={'disp': True})

	return sol

def objective_uncons(x, target, P):
	return x[0]**2 + x[1]**2 + abs(nonLinCst(x, target, P)) * 1000