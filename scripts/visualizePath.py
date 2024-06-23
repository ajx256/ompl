#!/usr/bin/env python3

from mpl_toolkits.mplot3d import Axes3D
import numpy
import matplotlib.pyplot as plt
import sys

if len(sys.argv) < 2:
    print("Missing path to file containing list of states")
    exit()

data = numpy.loadtxt(sys.argv[1])
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot(data[:,0],data[:,1],data[:,2],'.-')

ax.invert_zaxis()
plt.show()