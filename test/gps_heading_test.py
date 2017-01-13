#!/usr/bin/env python

"Test script to validate the GPS heading component."

import numpy as np
import matplotlib.pyplot as plt
import math

# Get the GPS data points

gps = "gps.txt"
gps_points = np.empty(shape=[0, 4])

with open(gps) as f:
    for line in f:
        point = np.array(line.strip().split(" "), dtype=float)
        gps_points = np.vstack((gps_points, point))

# Get the heading based on the GPS data points

heading = "heading.txt"
heading_points = np.empty(shape=[0, 5])

with open(heading) as f:
    for line in f:
        point = np.array(line.strip().split(" "), dtype=float)
        
        x, y, z, w = point[1:5]
        
        # Get the heading from the quaternion
        yaw = math.atan2(2.0*(x*y + w*z), w*w + x*x - y*y - z*z);
        
        gps_point = (gps_points[point[0]==gps_points[:,0]]).flatten()
        
        pos_x, pos_y, pos_z = gps_point[1:4]
        
        heading_points = np.vstack((heading_points, np.array([point[0], pos_x, pos_y, pos_z, yaw])))

plt.figure()
plt.axis('equal')

# Plot all the GPS points and draw lines from one to the next
x = heading_points[:,1]
y = heading_points[:,2]
plt.quiver(x[:-1], y[:-1], x[1:]-x[:-1], y[1:]-y[:-1], scale_units='xy', angles='xy', scale=1)

# Transform angles from radians to degrees and plot the heading at the position of the GPS points
angles = heading_points[:,4] * 180 / math.pi
plt.quiver(x, y, 1, 0, scale_units='xy', angles=angles, scale=1, pivot='tail', color='r')

plt.xlabel('x')
plt.ylabel('y')
plt.show()

