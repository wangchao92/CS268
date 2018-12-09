from cannonDynamics import *

def rk4_cannon(t, z0, c):
	nTime = len(t)
	nState, nSim = z0.shape

	if nSim == 1:
		z = np.zeros((nState, nTime))

		z[:,:1] = z0
		for i in range(nTime - 1):
			dt = t[i + 1] - t[i]
			k1 = cannonDynamics(z[:,i:i+1], t[i], c)
			k2 = cannonDynamics(z[:,i:i+1] + 0.5 * dt * k1, t[i] + 0.5 * dt, c)
			k3 = cannonDynamics(z[:,i:i+1] + 0.5 * dt * k2, t[i] + 0.5 * dt, c)
			k4 = cannonDynamics(z[:,i:i+1] + dt * k3, t[i] + dt, c)
			z[:,i+1:i+2] = z[:,i:i+1] + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
	else:
		z = np.zeros((nState, nSim, nTime))
		z[:,:,0] = z0
		for i in range(nTime - 1):
			dt = t[i + 1] - t[i]
			k1 = cannonDynamics(z[:,:,i], t[i], c)
			k2 = cannonDynamics(z[:,:,i] + 0.5 * dt * k1, t[i] + 0.5 * dt, c)
			k3 = cannonDynamics(z[:,:,i] + 0.5 * dt * k2, t[i] + 0.5 * dt, c)
			k4 = cannonDynamics(z[:,:,i] + dt * k3, t[i] + dt, c)
			z[:,:,i + 1] = z[:,:,i] + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)

	return z