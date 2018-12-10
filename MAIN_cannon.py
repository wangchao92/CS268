import math
from singleShooting import *
from multipleShooting import *
from utils import *
from rk4_cannon import *
import matplotlib.pyplot as plt
import random

def main():
	guess = {
		'initSpeed' : 9.0,
		'initAngle' : (math.pi / 180) * 45
	}

	target = {
		'x' : 6.0,
		'y' : 0.0
	}

	param = {
		'c' : 0.4,
		'nGrid' : 100,
		'nSegment' : 10,
		'nSubStep' : 5
	}

	# opt = 9.845
	# error = []
	# num_iter = []
	# num_fev = []
	# for i in range(10):
	# 	guess['initSpeed'] = random.uniform(7.0, 13.0)
	# 	print guess
	# 	sol = cannon_multipleShooting_uncon(guess, target, param)
	# 	error.append(abs(opt - math.sqrt(sol['dx']**2 + sol['dy']**2)))
	# 	num_iter.append(sol['iter'])
	# 	num_fev.append(sol['fev'])

	# print np.mean(error), np.std(error), np.mean(num_iter), np.std(num_iter), np.mean(num_fev), np.std(num_fev)
	soln = {}
	soln['singleShooting_cons_SLSQP'] = cannon_singleShooting_cons(guess, target, param)
	soln['singleShooting_uncons_NM'] = cannon_singleShooting_uncons(guess, target, param)
	soln['multipleShooting_cons_SLSQP'] = cannon_multipleShooting(guess, target, param)
	soln['multipleShooting_uncon_NM'] = cannon_multipleShooting_uncon(guess, target, param)
	
	print '--------------------------------------'
	result_dx = []
	result_dy = []
	result_speed = []
	methods = []
	for key in soln:
		if soln[key]['success']:
			dx = soln[key]['dx']
			dy = soln[key]['dy']
			speed = math.sqrt(dx**2 + dy**2)
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
	if len(result_dx) == 0:
		return
	for i in range(len(result_dx)):
		dx0 = result_dx[i]
		dy0 = result_dy[i]
		speed = result_speed[i]
		x0 = 0.0
		y0 = 0.0
		z0 = np.array([x0, y0, dx0, dy0]).reshape((4,1))

		sol = rk4_cannon(time, z0, param['c'])

		idx = findGround(sol[1, :])

		tspan = np.linspace(0, time[idx], 100)
		z = rk4_cannon(tspan, z0, param['c'])

		groundX = np.linspace(-1, 9, 100)
		groundY = np.linspace(0, 0, 100)
		plt.plot(z[0,:], z[1,:], label=methods[i]+':v=%.2f' % (result_speed[i]), linewidth=2)
	plt.xlim(-1, 7)
	plt.ylim(-1, 3)
	plt.gca().set_aspect('equal', adjustable='box')
	plt.plot(groundX, groundY, color='brown', linewidth=4)
	plt.plot((0), (0), 'o', color='black', markersize=10)
	plt.plot((target['x']), (target['y']), 'x', color='r', markersize=10, markeredgewidth=2, label='target')
	plt.legend(loc='upper right')
	plt.title('Optimization Results')
	plt.show()

if __name__ == '__main__':
	main()