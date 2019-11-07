# This import registers the 3D projection, but is otherwise unused.
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import

import numpy as np
import matplotlib.pyplot as plt
import math

"""
plt.rcParams['legend.fontsize'] = 10

fig = plt.figure()
ax = fig.gca(projection='3d')

# Prepare arrays x, y, z
theta = np.linspace(-4 * np.pi, 4 * np.pi, 100)
z = np.linspace(-2, 2, 100)
r = z**2 + 1
x = r * np.sin(theta)
y = r * np.cos(theta)

ax.plot(x, y, z, label='parametric curve')
ax.legend()

plt.show()
"""
x0 = 0
y0 = 0
angle0 = math.pi/2
x1 = 25
y1 = 25
angle1 = math.pi/2


slopex0 = math.cos(angle0) * math.sqrt((x1-x0)**2 + (y1-y0)**2) * 2
slopey0 = math.sin(angle0) * math.sqrt((x1-x0)**2 + (y1-y0)**2) * 2
slopex1 = math.cos(angle1) * math.sqrt((x1-x0)**2 + (y1-y0)**2) * 2
slopey1 = math.sin(angle1) * math.sqrt((x1-x0)**2 + (y1-y0)**2) * 2


ax = 2 * x0 - 2 * x1 + slopex1 + slopex0
bx = -2 * slopex0 -slopex1 - 3 * x0 + 3 * x1
cx = slopex0
dx = x0

ay = 2 * y0 - 2 * y1 + slopey1 + slopey0
by = -2 * slopey0 - slopey1 - 3 * y0 + 3 * y1
cy = slopey0
dy = y0

def xt(t0):
    return ax*(t0**3) + bx*(t0**2) + cx*t0 + dx

def yt(t0):
    return ay*(t0**3) + by*(t0**2) + cy*t0 + dy

steps = 0.001
t = np.arange(0,1+steps,steps)
x = []
y = []
for i in range(int(1/steps)):
    x.append(xt(t[i]))
    y.append(yt(t[i]))


screen_size = 10

plt.title('Spline Interpolation')
plt.xlabel('x(t)')
plt.ylabel('y(t)')
plt.axis([x0-screen_size,x1+screen_size,y0-screen_size,y1+screen_size])
plt.plot(x,y,'bo')


plt.show()
