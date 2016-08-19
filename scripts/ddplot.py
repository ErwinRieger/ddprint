#!/usr/bin/python

import matplotlib.pyplot as plt 
from matplotlib import collections  as mc
import math, pickle, sys

(plot1, plot2) = pickle.load(open(sys.argv[1]))

fig = plt.figure(1)
# fig.suptitle(sys.argv[1], x=0.01, horizontalalignment="left")
fig.canvas.set_window_title(sys.argv[1])

ax = x=plt.subplot(211)

plt.ylabel('Vel')
plt.title('XY and E-Velocity')
plt.grid(True)

lc = mc.LineCollection(plot1.Lines, colors=plot1.Colors, linestyles=plot1.Styles, linewidth=3)
ax.add_collection(lc)

for i in range(len(plot1.Ticks)):
    (x, y, nr) = plot1.Ticks[i]
    plt.axvline(x, color="yellow")
    plt.text(x*1.01, y*1.01, "%d" % nr)

ax.autoscale()
plt.xlim(xmin=0)

ax2 = plt.subplot(212, sharex=ax)
ax2.set_xlim(0)

plt.ylabel('Vel')
plt.title('E-Advance')
plt.grid(True)

lc = mc.LineCollection(plot2.Lines, colors=plot2.Colors, linestyles=plot2.Styles, linewidth=3)
ax2.add_collection(lc)

for i in range(len(plot1.Ticks)):
    (x, y, nr) = plot1.Ticks[i]
    plt.axvline(x, color="yellow")

plt.axhline(0, color="black")

ax2.autoscale()
plt.xlim(xmin=0)


fig.tight_layout()
plt.show()










