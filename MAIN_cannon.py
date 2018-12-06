import math
from singleShooting import cannon_singleShooting

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
		'nGrid' : 100
	}

	soln = {}
	soln['singleShooting'] = cannon_singleShooting(guess, target, param)
	print soln.success

if __name__ == '__main__':
	main()