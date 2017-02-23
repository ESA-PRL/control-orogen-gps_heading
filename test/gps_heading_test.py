#!/usr/bin/env python

"Test script to validate the GPS heading component."

import numpy as np
import matplotlib.pyplot as plt
import math
import re

# Get the GPS data points

gps = "gps.txt"
gps_points = np.empty(shape=[0, 4])

with open(gps) as f:
    for line in f:
        regex = re.search('(\d+)\s(?:\d{1,3}\s){13}([-\d\.e]+)\s([-\d\.e]+)\s([-\d\.e]+)', line, re.IGNORECASE)
        if regex:
            point = np.array([regex.group(1), regex.group(2), regex.group(3), regex.group(4)], dtype=float)
            gps_points = np.vstack((gps_points, point))

# Get the heading based on the GPS data points

heading = "heading.txt"
all_heading_points = np.empty(shape=[0, 5])
new_heading_points = np.empty(shape=[0, 5])
old_heading_points = np.empty(shape=[0, 5])

previous_yaw = 0

with open(heading) as f:
    for line in f:
        regex = re.search('(\d+)\s(?:\d{1,3}\s){13}(?:[-\d\.e]+\s){12}([-\d\.e]+)\s([-\d\.e]+)\s([-\d\.e]+)\s([-\d\.e]+)', line, re.IGNORECASE)
        if regex:
            point = np.array([regex.group(1), regex.group(2), regex.group(3), regex.group(4), regex.group(5)], dtype=float)
            
            x, y, z, w = point[1:5]
            
            # Get the heading from the quaternion
            yaw = math.atan2(2.0*(x*y + w*z), w*w + x*x - y*y - z*z);
            gps_point = (gps_points[point[0]==gps_points[:,0]]).flatten()
            pos_x, pos_y, pos_z = gps_point[1:4]
            
            all_heading_points = np.vstack((all_heading_points, np.array([point[0], pos_x, pos_y, pos_z, yaw])))
            
            if math.fabs(yaw - previous_yaw) < 1e-3:
                new_heading_points = np.vstack((new_heading_points, np.array([point[0], pos_x, pos_y, pos_z, yaw])))
            else:           
                old_heading_points = np.vstack((old_heading_points, np.array([point[0], pos_x, pos_y, pos_z, yaw])))
                
            previous_yaw = yaw

plt.figure()
plt.axis('equal')

# Plot all the GPS points and draw lines from one to the next
x = all_heading_points[:,1]
y = all_heading_points[:,2]
plt.quiver(x[:-1], y[:-1], x[1:]-x[:-1], y[1:]-y[:-1], scale_units='xy', angles='xy', scale=1)

# Transform angles from radians to degrees and plot the heading at the position of the GPS points
x = old_heading_points[:,1]
y = old_heading_points[:,2]
angles = old_heading_points[:,4] * 180 / math.pi
plt.quiver(x, y, 1, 0, scale_units='xy', angles=angles, scale=20, pivot='tail', color='r')

x = new_heading_points[:,1]
y = new_heading_points[:,2]
angles = new_heading_points[:,4] * 180 / math.pi
plt.quiver(x, y, 1, 0, scale_units='xy', angles=angles, scale=20, pivot='tail', color='b')

plt.xlabel('x')
plt.ylabel('y')
plt.show()

