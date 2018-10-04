#!/usr/bin/env python
#Author Peng Wei
#Description: read and plot data from .csv file

import matplotlib.pyplot as plt
import matplotlib.mlab as mlab
import csv
import sys
import os

if __name__ == "__main__":
	if len(sys.argv) != 2:
		print '[Error 1]: Please provide the file path!'
		exit()
	elif os.path.isfile(sys.argv[1]) == False:
		print '[Error 2]: No such file!'
		exit()
	r = mlab.csv2rec(sys.argv[1])
	N = len(r)
	error_x, = plt.plot(r['time'], r['x'], 'r-')
	error_y, = plt.plot(r['time'], r['y'], 'b-')
	error_z, = plt.plot(r['time'], r['z'], 'g-')
	plt.plot([0.0, r['time'][-1]], [0.0, 0.0], 'k--') 
	plt.xlabel('Time (s)')
	plt.ylabel('Error (m)')
	plt.legend([error_x, error_y, error_z,], ['x','y','z'])
	#plt.axis([0, 6, 0, 20])
	plt.show()