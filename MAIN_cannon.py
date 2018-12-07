import math
from singleShooting import *
from utils import *
from rk4_cannon import *
import matplotlib.pyplot as plt

def main():
	guess = {
		'initSpeed' : 9.0,
		'initAngle' : (math.pi / 180) * 45
	}

	target = {
		'x' : 7.0,
		'y' : 0.0
	}

	param = {
		'c' : 0.4,
		'nGrid' : 100
	}

	soln = {}
	soln['singleShooting_cons_SLSQP'] = cannon_singleShooting_cons(guess, target, param)
	soln['singleShooting_uncons_NM'] = cannon_singleShooting_uncons(guess, target, param)
	
	print '--------------------------------------'
	result_dx = []
	result_dy = []
	result_speed = []
	methods = []
	for key in soln:
		if soln[key].success:
			dx = soln[key].x[0]
			dy = soln[key].x[1]
			speed = math.sqrt(soln[key].x[0]**2 + soln[key].x[1]**2)
			print key + ':'
			print 'dx: %f, dy: %f, speed: %f\n' % (dx, dy, speed)
			result_dx.append(dx)
			result_dy.append(dy)
			result_speed.append(speed)
			methods.append(key)
	plotResult(methods, result_dx, result_dy, result_speed, param, target)

def plotResult(methods, result_dx, result_dy, result_speed, param, target):
	plt.figure()
	time = np.linspace(0, 10, 100)

	for i in range(len(result_dx)):
		dx0 = result_dx[i]
		dy0 = result_dy[i]
		speed = result_speed[i]
		x0 = 0.0
		y0 = 0.0
		z0 = np.array([x0, y0, dx0, dy0])

		sol = rk4_cannon(time, z0, param['c'])

		idx = findGround(sol[:, 1])

		tspan = np.linspace(0, time[idx], 100)
		z = rk4_cannon(tspan, z0, param['c'])

		plt.axis('equal')
		groundX = np.linspace(-1, 9, 100)
		groundY = np.linspace(0, 0, 100)
		plt.plot(z[:, 0], z[:, 1], label=methods[i]+':v=%.2f' % (result_speed[i]), linewidth=2)
	plt.plot(groundX, groundY, color='brown', linewidth=4)
	plt.plot((0), (0), 'o', color='black', markersize=10)
	plt.plot((target['x']), (target['y']), 'x', color='r', markersize=10, markeredgewidth=2, label='target')
	plt.legend(loc='upper right')
	plt.title('Single Shooting Method')
	plt.show()

if __name__ == '__main__':
	main()