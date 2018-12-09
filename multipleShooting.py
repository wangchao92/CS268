from simulateCannon import *
from scipy.optimize import minimize
from cannonDynamics import *
import numpy as np
from scipy.interpolate import interp1d
from rk4_cannon import *

def cannon_multipleShooting(guess, target, params):
    init = {
        'speed': guess['initSpeed'],
        'angle': guess['initAngle']
    }

    P = {
        'c': params['c'],
        'nGrid': params['nGrid'],
        'nSegment': params['nSegment'],
        'nSubStep': params['nSubStep']
    }

    traj = simulateCannon(init, P)

    guess['T'] = traj['t'][-1]
    guess['t'] = np.linspace(0, guess['T'], P['nSegment'] + 1)
    guess['t'] = np.delete(guess['t'], -1)
    
    f = interp1d(traj['t'], [traj['x'], traj['y'], traj['dx'], traj['dy']], kind='cubic')
    guess['z'] = f(guess['t'])

    nState = 4
    x0 = [guess['T']]
    x0.extend(guess['z'].transpose().reshape(-1).tolist())
    
    eq_cons = {'type': 'eq', 'fun': nonLinCst, 'args': (target, P)}
    sol = minimize(objective, x0, method='SLSQP', constraints=[eq_cons], options={'disp': True})

    return {'dx': sol.x[3], 'dy': sol.x[4], 'success': sol.success}

def nonLinCst(x, target, P):
    nState = 4
    nSegment = P['nSegment']
    tEnd = x[0]

    z0 = np.reshape(x[1:], (P['nSegment'], nState)).transpose()

    nSub = P['nSubStep']
    tSim = np.linspace(0, tEnd / nSegment, nSub + 1)
    z = rk4_cannon(tSim, z0, P['c'])

    BoundaryInit = [z[0,0,0], z[1,0,0]]
    BoundaryFinal = [z[0,-1,-1] - target['x'], z[1,-1,-1] - target['y']]

    zEnd = z[:,:-1,-1]
    zStart = z[:,1:,0]
    Defects = np.reshape(zStart - zEnd, (-1))

    Ceq = []
    Ceq.extend(BoundaryInit)
    Ceq.extend(BoundaryFinal)
    Ceq.extend(Defects.tolist())

    return Ceq


def objective(x):
    return x[3]**2 + x[4]**2

def objective_uncon(x, target, P):
    return x[3]**2 + x[4]**2 + sum(x**2 for x in nonLinCst(x, target, P)) * 1000

def cannon_multipleShooting_uncon(guess, target, params):
    init = {
        'speed': guess['initSpeed'],
        'angle': guess['initAngle']
    }

    P = {
        'c': params['c'],
        'nGrid': params['nGrid'],
        'nSegment': params['nSegment'],
        'nSubStep': params['nSubStep']
    }

    traj = simulateCannon(init, P)

    guess['T'] = traj['t'][-1]
    guess['t'] = np.linspace(0, guess['T'], P['nSegment'] + 1)
    guess['t'] = np.delete(guess['t'], -1)
    
    f = interp1d(traj['t'], [traj['x'], traj['y'], traj['dx'], traj['dy']], kind='cubic')
    guess['z'] = f(guess['t'])

    nState = 4
    x0 = [guess['T']]
    x0.extend(guess['z'].transpose().reshape(-1).tolist())
    
    eq_cons = {'type': 'eq', 'fun': nonLinCst, 'args': (target, P)}
    sol = minimize(objective_uncon, x0, args = (target, P), method='Nelder-Mead', options={'disp': True})

    return {'dx': sol.x[3], 'dy': sol.x[4], 'success': sol.success}