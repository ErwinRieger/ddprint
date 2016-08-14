
import matplotlib.pyplot as plt 
from matplotlib import collections  as mc
import math, pickle, sys

d = pickle.load(open(sys.argv[1]))

fig, ax = plt.subplots()
print "ax:", ax
# ax.set_xlim(-1, 10) 
# ax.set_ylim(0, 2) 

for i in range(len(d["xylines"]) / 3):
    plt.axvline(d["xylines"][i*3][0][0], color="yellow")

lc = mc.LineCollection(d["xylines"], colors=d["colors"], linestyles=d["linestyles"])
ax.add_collection(lc)

lc = mc.LineCollection(d["elines"], colors=d["colors"], linestyles=d["linestyles"])
ax.add_collection(lc)

ax.autoscale()
# ax.margins(0.02)

fig.tight_layout()

while True:
    plt.pause(0.05)










