#!/usr/bin/python

from ddprintconstants import *

import matplotlib.pyplot as plt 
from matplotlib import collections  as mc
import math, pickle, sys, pprint

plot = pickle.load(open(sys.argv[1]))

fig, ax = plt.subplots()
fig.canvas.set_window_title(sys.argv[1])

plt.ylabel('Vel')
plt.title('XY and E-Velocity')
plt.grid(True)

acolors = ["gx", "gv", "g>", "g*"]
lcolors = ["bx", "bv", "b>", "b*"]
dcolors = ["rx", "rv", "r>", "r*"]

rdcolors = ["mx", "mv", "m>", "m*"]

edirection = 1

freq = [
        -1.0, # x
        -1.0, # y
        -1.0, # z
        -1.0  # E
        ]

# Y-position (time) where the next step will be drawn. Increased by
# tStep after step is drawn.
ty = 0.0
# Time between last and current step, corresponds to the timer value of
# the previous step. Used to compute frequency/speed.
tStep = 0.0
tSteps = [0, 0, 0, 0, 0]

# aktuelle frequenz wird durch den vorhergehenden
# step (dessen timervalue) bestimmt.
def F(timer):

    global ty, tStep, tSteps

    if tStep == 0:
        f = 0.0
    else:
        f = 1.0 / tStep

    res = (ty, f)

    ty += tStep
    tStep = timer / fTimer
    # preset tStep for following raw moves
    tSteps = [tStep, tStep, tStep, tStep, tStep]
    return res

def plotFreq(axis, ax, t, f, color, force = False):

    global freq

    if f != freq[axis] or force:

        if axis == 3 and edirection < 1:
            ax.plot(t, -f, color, ms=15)
        else:
            ax.plot(t, f, color, ms=15)
        freq[axis] = f

for i in range(len(plot.moves)):

    move = plot.moves[i]

    plt.axvline(ty, color="yellow")
    plt.text(ty, 100, "%d" % move["number"])

    if "dirbits" in move.keys():
        edirection = move["dirbits"] & 0x8

    if move["stepType"] == "raw":

        for (timer, steps) in move["pulses"]:

            print steps, timer
            unusedDims = range(5)
            tStep = timer / fTimer

            for i in range(5):

                if steps[i]:

                    # tstep fuer diese dimension
                    f = 1.0 / tSteps[i]

                    print "f:", f
                    plotFreq(i, ax, ty, f, rdcolors[i])

                    tSteps[i] = tStep

                    del unusedDims[unusedDims.index(i)]

            for i in unusedDims:
                tSteps[i] += tStep

            # Time for next step
            ty += tStep

    else:

        for timer in move["accelPulses"]:

            (x, y) = F(timer)
            plotFreq(move["leadAxis"], ax, x, y, acolors[move["leadAxis"]])

        if move["linearSteps"]:

            tstep = move["linearTimer"] / fTimer
            f = 1.0 / tstep
            tlin = tstep * (move["linearSteps"]-1)

            ax.plot(ty, f, lcolors[move["leadAxis"]], ms=15)
            ax.plot(ty+tlin, f, lcolors[move["leadAxis"]], ms=15)
            ax.plot((ty, ty+tlin), (f, f), "-b")

            ty += tlin

        for timer in move["deccelPulses"]:

            (x, y) = F(timer)
            plotFreq(move["leadAxis"], ax, x, y, dcolors[move["leadAxis"]])

ax.autoscale()
plt.xlim(xmin=0)

fig.tight_layout()
plt.show()










