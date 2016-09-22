import matplotlib.pyplot as plt 
from matplotlib import collections  as mc
import math

d=[(0, 0), (2, 1), (4, 1), (5, .5), (7, 1.2), (8, 1.2), (9,0)]

fig, ax = plt.subplots()
print "ax:", ax
ax.set_xlim(-1, 10) 
ax.set_ylim(0, 2) 

lines = []
color = []
for i in range(len(d)-1):

    l = len(lines)

    if l % 3 == 0:
        l = plt.axvline(d[i][0], color="yellow")
        color.append("green")
    elif l % 3 == 1:
        color.append("blue")
    elif l % 3 == 2:
        color.append("red")
    lines.append((d[i], d[i+1]))

lc = mc.LineCollection(lines, colors=color, linewidths=2)
ax.add_collection(lc)

lines = []
color = []
for i in range(len(d)-1):

    l = len(lines)

    if l % 3 == 0:
        l = plt.axvline(d[i][0], color="yellow")
        color.append("green")
    elif l % 3 == 1:
        color.append("blue")
    elif l % 3 == 2:
        color.append("red")

    lines.append(((d[i][0], d[i][1]/3), (d[i+1][0], d[i+1][1]/3)))

lc = mc.LineCollection(lines, colors=color, linewidths=2)
ax.add_collection(lc)
ax.autoscale()
ax.margins(0.1)

while True:
    plt.pause(0.05)










