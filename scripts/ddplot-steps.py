#!/usr/bin/python

from ddprintconstants import *

import matplotlib.pyplot as plt 
from matplotlib import collections  as mc
import math, pickle, sys

plot = pickle.load(open(sys.argv[1]))

# fig = plt.figure(1)
# fig.suptitle(sys.argv[1], x=0.01, horizontalalignment="left")

fig, ax = plt.subplots()
fig.canvas.set_window_title(sys.argv[1])

# ax = plt.subplot()

plt.ylabel('Vel')
plt.title('XY and E-Velocity')
plt.grid(True)

# lc = mc.LineCollection(plot1.Lines, colors=plot1.Colors, linestyles=plot1.Styles, linewidth=3)
# x.add_collection(lc)

t = 0.0
tplot = 0

colors = ["r.", "r.", "r.", "m."]

for i in range(len(plot.moves)):

    move = plot.moves[i]

    plt.axvline(t, color="yellow")
    plt.text(t*1.01, 10, "%d" % move["number"])

    if move["stepType"] == "raw":

        times = [t, t, t, t, t]

        for (timer, steps) in move["pulses"]:

            tstep = timer / fTimer
            t += tstep

            for i in range(4):

                if steps[i]:
            
                    f = 1.0 / (t - times[i])
                    # print "f:", f
                    times[i] = t

                    # if t >= tplot:
                    if True:
                        ax.plot(t, f, colors[i])
                        # if i<=3:
                            # tplot += 0.01
                        # else:
                            # tplot += 0.001

    else:

        for timer in move["accelPulses"]:

            tstep = timer / fTimer
            f = 1.0 / tstep

            if t >= tplot:
                ax.plot(t, f, "go")
                tplot += 0.01
            t += tstep

        tstep = move["linearTimer"] / fTimer
        f = 1.0 / tstep

        for i in range(move["linearSteps"]):

            if t >= tplot:
                ax.plot(t, f, "bo")
                tplot += 0.01
            t += tstep

        for timer in move["deccelPulses"]:

            tstep = timer / fTimer
            f = 1.0 / tstep
            if t >= tplot:
                ax.plot(t, f, "ro")
                tplot += 0.01
            t += tstep

ax.autoscale()
plt.xlim(xmin=0)

fig.tight_layout()
plt.show()










