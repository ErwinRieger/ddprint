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
te = 0.0

acolors = ["gx", "gv", "g,", "g*"]
lcolors = ["bx", "bv", "b,", "b*"]
dcolors = ["rx", "rv", "r,", "r*"]

edirection = 1

freq = [
        -1.0, # xyz
        -1.0  # E
        ]

def plotFreq(axis, ax, t, f, color, force = False):

    global freq

    # assert(f > 50)

    if axis <= 2:
        axis = 0
    else:
        axis = 1

    if f != freq[axis] or force:

        if axis == 1 and edirection < 1:
            ax.plot(t, -f, color)
        else:
            ax.plot(t, f, color)
        freq[axis] = f

for i in range(len(plot.moves)):

    move = plot.moves[i]

    plt.axvline(t, color="yellow")
    plt.text(t, 100, "%d" % move["number"])

    if "dirbits" in move.keys():
        edirection = move["dirbits"] & 0x8

    if move["stepType"] == "raw":

        times = [t, t]
        # te = t

        for (timer, steps) in move["pulses"]:

            tstep = timer / fTimer

            t += tstep

            if steps[move["leadAxisXYZ"]]:
        
                f = 1.0 / (t - times[0])
                times[0] = t
                # print "f:", f
                plotFreq(move["leadAxisXYZ"], ax, t, f, dcolors[move["leadAxisXYZ"]])

            if steps[3]:

                f = 1.0 / (t - times[1])
                times[1] = t
                # print "f:", f
                plotFreq(3, ax, t, f, "mo")

    else:

        for timer in move["accelPulses"]:

            tstep = timer / fTimer
            f = 1.0 / tstep
            plotFreq(move["leadAxis"], ax, t, f, acolors[move["leadAxis"]])
            t += tstep

        if move["linearSteps"]:
            tstep = move["linearTimer"] / fTimer
            f = 1.0 / tstep
            plotFreq(move["leadAxis"], ax, t, f, lcolors[move["leadAxis"]], force = True)

            # for i in range(move["linearSteps"]):
                # plotFreq(move["leadAxis"], ax, t, f, lcolors[move["leadAxis"]])
                # t += tstep

            if move["linearSteps"] > 1:
                t += tstep * move["linearSteps"]
                plotFreq(move["leadAxis"], ax, t, f, lcolors[move["leadAxis"]], force = True)

        for timer in move["deccelPulses"]:

            tstep = timer / fTimer
            f = 1.0 / tstep
            # ax.plot(t, f, "ro")
            plotFreq(move["leadAxis"], ax, t, f, dcolors[move["leadAxis"]])
            t += tstep

ax.autoscale()
plt.xlim(xmin=0)

fig.tight_layout()
plt.show()










