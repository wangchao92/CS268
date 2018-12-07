import math
from singleShooting import *

def main():
	guess = {
		'initSpeed' : 5.0,
		'initAngle' : (math.pi / 180) * 45
	}

	target = {
		'x' : 6.0,
		'y' : 0.0
	}

	param = {
		'c' : 0.4,
		'nGrid' : 1000
	}

	soln = {}
	soln['singleShooting_cons_SLSQP'] = cannon_singleShooting_cons(guess, target, param)
	soln['singleShooting_uncons_NM'] = cannon_singleShooting_uncons(guess, target, param)
	
	print '--------------------------------------'
	for key in soln:
		if soln[key].success:
			print key + ':'
			print 'dx: %f, dy: %f, speed: %f' % (soln[key].x[0], soln[key].x[1], math.sqrt(soln[key].x[0]**2 + soln[key].x[1]**2))

if __name__ == '__main__':
	main()