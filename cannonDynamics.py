import math
import numpy as np

def cannonDynamics(z, t, c):
	dz = np.zeros(z.shape)

	# First-order form (velocity)
	dz[0:2,:] = z[2:4,:]

	# Compute speed and drag force
	dx = z[2,:]
	dy = z[3,:]
	v = np.sqrt(dx*dx + dy*dy)
	fx = -c * dx * v
	fy = -c * dy * v - 1

	dz[2,:] = fx
	dz[3,:] = fy

	return dz