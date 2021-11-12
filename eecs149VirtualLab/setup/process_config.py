import math
import simplejson
import os

theta = 0 # deg
x_input = 0
y_input = 0 


def writeConfigFiles(input_arg):
	[x_input, y_input, theta, filename] = input_arg
	print([x_input, y_input, theta, filename])
	x = x_input
	y = y_input
	theta = theta + 90 # deg

	# dl: long diagonal, ds: short diagonal
	dl = math.sqrt(math.pow(3,2) + math.pow(0.58,2))
	ds = math.sqrt(math.pow(1.43,2) + math.pow(0.58,2)) 
	print("dl: ", dl)
	print("ds: ", ds)

	# tl : theta for long diagonal, ts : theta for short diagonal
	tl = math.degrees(math.atan2(0.58, 3))
	ts = math.degrees(math.atan2(0.58, 1.43))
	print("tl: ", tl)
	print("ts: ", ts)

	x1 = 0.58 * math.cos(math.radians(theta+90)) + x - 0.2 * math.cos(math.radians(theta))
	y1 = 0.58 * math.sin(math.radians(theta+90)) + y - 0.2 * math.sin(math.radians(theta))

	x2 = ds * math.cos(math.radians(theta+ts)) + x - 0.2 * math.cos(math.radians(theta))
	y2 = ds * math.sin(math.radians(theta+ts)) + y - 0.2 * math.sin(math.radians(theta))

	x3 = ds * math.cos(math.radians(theta-ts)) + x - 0.2 * math.cos(math.radians(theta))
	y3 = ds * math.sin(math.radians(theta-ts)) + y - 0.2 * math.sin(math.radians(theta))

	x4 = 0.58 * math.cos(math.radians(theta-90)) + x - 0.2 * math.cos(math.radians(theta))
	y4 = 0.58 * math.sin(math.radians(theta-90)) + y - 0.2 * math.sin(math.radians(theta))

	x5 = dl * math.cos(math.radians(theta+tl)) + x - 0.2 * math.cos(math.radians(theta))
	y5 = dl * math.sin(math.radians(theta+tl)) + y - 0.2 * math.sin(math.radians(theta))

	x6 = dl * math.cos(math.radians(theta-tl)) + x - 0.2 * math.cos(math.radians(theta))
	y6 = dl * math.sin(math.radians(theta-tl)) + y - 0.2 * math.sin(math.radians(theta))

	coords = [(x1,y1), (x2,y2), (x3,y3), (x4,y4), (x5,y5), (x6,y6)]

	filePath = os.path.join('./tests_and_configs', filename+'.config')

	with open(filePath, 'w') as file:
		file.write("ramp: [")
		for coord in coords:
			file.write(str(coord))
			if coord != coords[-1]:
				file.write(";")
		file.write("]")
	file.close()

	return coords


tests = [[-0.2, -0.2, -145, 'ramp_test3'], [0.2, 0.2, -145, 'ramp_test4'], [0.2, 0.2, -230, 'ramp_test5'], [0.2, -0.5, 0, 'ramp_test6'], [0.2, -0.2, 0, 'ramp_test6_extra']]

for test in tests:
	output = writeConfigFiles(test)
	print(output)