
import matplotlib.pyplot as plt 
from matplotlib import collections  as mc
import math, pickle, sys

d = pickle.load(open(sys.argv[1]))

fig = plt.figure(1)
ax = x=plt.subplot(211)
# print "ax:", ax
# ax.set_xlim(-1, 10) 
# ax.set_ylim(0, 2) 

# plt.xlabel('t')
plt.ylabel('Vel')
plt.title('XY and E-Velocity')
plt.grid(True)

for i in range(len(d["xylines"]) / 3):
    x = d["xylines"][i*3][0][0]
    plt.axvline(x, color="yellow")

    # x = d["xylines"][i*3+1][0][0]
    y = d["xylines"][i*3+1][0][1]
    plt.text(x*1.01, y*1.01, "%d" % i)

lc = mc.LineCollection(d["xylines"]+d["elines"], colors=d["colors"], linestyles=d["linestyles"])
ax.add_collection(lc)

# lc = mc.LineCollection(d["elines"], colors=d["colors"], linestyles=d["linestyles"])
# ax.add_collection(lc)

ax.autoscale()
plt.xlim(xmin=0)
# ax.margins(0.02)

ax2 = plt.subplot(212, sharex=ax)
ax2.set_xlim(0)
# plt.xlabel('t')
plt.ylabel('Vel')
plt.title('E-Advance')
plt.grid(True)

for i in range(len(d["xylines"]) / 3):
    x = d["xylines"][i*3][0][0]
    plt.axvline(x, color="yellow")

# lc = mc.LineCollection(d["e2lines"], colors=d["colors"], linestyles=d["linestyles"])
lc = mc.LineCollection(d["e2lines"], colors="grey", linestyles=d["linestyles"])
ax2.add_collection(lc)

ax2.autoscale()
plt.xlim(xmin=0)

fig.tight_layout()
plt.show()










