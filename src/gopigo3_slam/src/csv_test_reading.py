#!/usr/bin/env python3
import numpy as np

file = "waypoints.csv"

goals = np.loadtxt(file, delimiter=';', skiprows=1, usecols=[1,2,3], dtype=float)
# Printing data stored
print(goals)
x1 = goals[2,1]
print(x1*2)