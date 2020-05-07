import matplotlib as mpl
import matplotlib.pyplot as plt
from random import randint

fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111)

points = [(10, 0, -30), (10, 50, -30), (20, 50, -30), (20, 0, -30), (30, 0, -30), (30, 50, -30), (40, 50, -30), (40, 0, -30), (50, 0, -30)]
xs = []
ys = []
for p in points:
  xs.append(p[0])
  ys.append(p[1])
  
ax.plot(xs, ys)

points = [(-50, 50, -30), (-40, 50, -30), (-40, 0, -30), (-30, 0, -30), (-30, 50, -30), (-20, 50, -30), (-20, 0, -30), (-10, 0, -30), (-10, 50, -30), (0, 50, -30), (0, 0, -30), (10, 0, -30)]
xs = []
ys = []
for p in points:
  xs.append(p[0])
  ys.append(p[1])
  
ax.plot(xs, ys)

points = [(-50, 0, -30), (-40, 0, -30), (-40, -50, -30), (-30, -50, -30), (-30, 0, -30), (-20, 0, -30), (-20, -50, -30), (-10, -50, -30), (-10, 0, -30), (0, 0, -30), (0, -50, -30), (10, -50, -30)]
xs = []
ys = []
for p in points:
  xs.append(p[0])
  ys.append(p[1])
  
ax.plot(xs, ys)

points = [(10, -50, -30), (10, 0, -30), (20, 0, -30), (20, -50, -30), (30, -50, -30), (30, 0, -30), (40, 0, -30), (40, -50, -30), (50, -50, -30)]
xs = []
ys = []
for p in points:
  xs.append(p[0])
  ys.append(p[1])
  
ax.plot(xs, ys)

plt.show()