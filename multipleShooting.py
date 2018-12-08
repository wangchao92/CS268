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

    return sol

def nonLinCst(x, target, P):
    nState = 4
    nSegment = P['nSegment']
    tEnd = x[0]
    z0 = np.reshape(x[1:], (nState, P['nSegment']))

    nSub = P['nSubStep']
    tSim = np.linspace(0, tEnd / nSegment, nSub + 1)
    z = rk4_cannon(tSim, z0, P['c'])

    BoundaryInit = abs(z[0,0,0]) + abs(z[1,0,0])
    BoundaryFinal = abs(z[0,-1,-1] - target['x']) + abs(z[1,-1,-1] - target['y'])

    zEnd = z[:,:-1,-1]
    zStart = z[:,1:,0]
    Defects = np.reshape(zStart - zEnd, (nState * (nSegment - 1), 1))

    dist = sum(np.abs(Defects)) + BoundaryFinal + BoundaryInit
    
    return dist


def objective(x):
    return x[3]**2 + x[4]**2