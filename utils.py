def findGround(arr):
	for i, n in enumerate(arr):
		if i > 0 and arr[i] < 0 and arr[i - 1] >0:
			return i if abs(arr[i]) < abs(arr[i - 1]) else i - 1
	return len(arr)