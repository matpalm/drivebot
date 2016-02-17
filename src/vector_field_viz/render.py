#!/usr/bin/env python
import json
import math
import matplotlib.pyplot as plt
import numpy as np
import sys

Y, X = np.mgrid[1:10:50j,1:10:50j]
U = np.ones_like(X) * 0
V = np.ones_like(Y) * 0

for line in sys.stdin:
    r = json.loads(line)
    i, j, theta = r['i'], r['j'], r['theta']
    U[i][j] = math.cos(theta)
    V[i][j] = math.sin(theta)

fig0, ax0 = plt.subplots()
strm = ax0.streamplot(X, Y, U, V, density=5, linewidth=1, cmap=plt.cm.autumn)
plt.show()
        
    
                             
