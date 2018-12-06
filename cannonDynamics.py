import math
import numpy as np

def cannonDynamics(z, t, c):
	dz = [0] * len(z)

	# First-order form (velocity)
	dz[0:2] = z[2:4]

	# Compute speed and drag force
	dx = z[2]
	dy = z[3]
	v = math.sqrt(dx**2 + dy**2)
	fx = -c * dx * v
	fy = -c * dy * v - 1

	dz[2] = fx
	dz[3] = fy

	return np.array(dz)