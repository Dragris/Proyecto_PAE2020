import numpy as np
from matplotlib import pylab as plt
import matplotlib.pyplot as pplt

data = np.genfromtxt("movement.log", comments="#", delimiter=',')
x = data[:, 0]
y = data[:, 1]
w = data[:, 2]
vl = data[:, 3]
vr = data[:, 4]

left, bottom, width, height = (0, 0, 4096, 4096)
habitacion = pplt.Rectangle((left, bottom), width, height, facecolor="green", alpha=0.1)
obstaculo1 = pplt.Rectangle((left, 1000), 320, 500, facecolor="black", alpha=0.1)
obstaculo2 = pplt.Rectangle((left, 2000), 640, 300, facecolor="black", alpha=0.1)
obstaculo3 = pplt.Rectangle((left, 3000), 320, 300, facecolor="black", alpha=0.1)
obstaculo4 = pplt.Rectangle((left, 3300), 640, 200, facecolor="black", alpha=0.1)
obstaculo5 = pplt.Rectangle((left, 3500), 512, 200, facecolor="black", alpha=0.1)
obstaculo6 = pplt.Rectangle((left, 3700), 320, 200, facecolor="black", alpha=0.1)

fig, ax = plt.subplots(figsize=[10, 10])

ax.add_patch(habitacion)
ax.add_patch(obstaculo1)
ax.add_patch(obstaculo2)
ax.add_patch(obstaculo3)
ax.add_patch(obstaculo4)
ax.add_patch(obstaculo5)
ax.add_patch(obstaculo6)

q = ax.quiver(x, y, np.cos(w), np.sin(w), color='red', scale=30)
plt.plot(x, y, 'o--')
xInit_point = x[0]
yInit_point = y[0]
plt.plot(xInit_point, yInit_point, 'gs')
xEnd_point = x[-1]
yEnd_point = y[-1]
plt.plot(xEnd_point, yEnd_point, 'k*')
plt.show()
