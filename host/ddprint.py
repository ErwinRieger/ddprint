#!/usr/bin/python
# -*- coding: utf-8 -*-
#/*
# This file is part of ddprint - a direct drive 3D printer firmware.
# 
# Copyright 2015 erwin.rieger@ibrieger.de
# 
# ddprint is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# ddprint is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with ddprint.  If not, see <http://www.gnu.org/licenses/>.
#*/

# python vim debugging:
# :compiler python
# :set makeprg=./stepstream.py\ pre\ test_files/z_leveltest_5cm.gcode
# :setlocal makeprg=./stepstream.py\ pre\ test_files/z_leveltest_5cm.gcode

#
#
#
# Problem bei der pfadplanung/movejoin:
#
# Wir wissen die geschwindigkeiten der umliegenden moves nicht, die nominalgeschwindigkeiten
# sind in diesem zusammenhang wertlos - sie dienen nur als begrenzung nach oben...
#
# Ausserdem beeinflussen sich die moves ja gegenseitig.
#
# Ausserdem können wir beim forward-join keine definitiven endgeschwindigkeiten bestimmen, da diese ja
# beim backward-join wieder geändert werden können.
#
# Dadurch können wir keine definierte startgeschwindigkeit des folge-moves festlegen und somit haut auch
# das stückeln in 'pathBlocks' nicht hin.
#
#
# Was wir machen können:
#
# * berechne intervall Vemin - Vemax
# * davon ausgehend Vsmin und Vsmax des folgemoves usw.
#

import traceback, logging, pprint, sys
import argparse

logging.basicConfig(level=logging.DEBUG)

import ddprintutil as util, gcodeparser, packedvalue, ddhome
import ddtest

from ddprofile import PrinterProfile, MatProfile, NozzleProfile
from ddplanner import Planner
from ddprinter import Printer, RxTimeout

#
# todo check max move length (max_length_steps)
# todo Optimierung joinMovesFwd(): z.b. beschleunigung über mehrere gleich gerichtete steps (kommt das oft vor?)?
#
#
# Nicht so dringend:
#
# todo check for cura flavor gcode (extrude-length computing)
# todo resend funktioniert nur mit dem aktuellen kommando, es dürfen nicht mehrere kommandos verloren gehen.
#
# Move-Join, gestückelte verarbeitung, streaming:
# -----------------------------------------------
#
# Pfad, path: Eine aus mehreren moves bestehende bewegung, gekennzeichnet dadurch, dass anfangs- und
# end-geschwindigkeit 0 (genauer: jerk) sind. Z.b. eine Z-ebene.
#
# Start eines path: 
#       * gekennzeichnet durch: path == [], v0 = 0 (bzw. jerk)
#
# Ende eines path: 
#       * gekennzeichnet durch: E-only move (G10) oder aufruf von finishMoves(), v1 = 0 (bzw. jerk)
#
# PathBlock: Teil eines path. Kriterium zum abtrennen eines PathBlocks:
#
#   * Endgeschwindigkeit ist 0 (bzw. <= jerk)
#
# xxxxxxxxxxx denk xxxxxxxxxxxxxxx
#
# * sammle moves
# * mache die betrachtung für jeden neuen move, optimiert wäre alle N neue moves
# * Gehe vorwärts durch 
#    - wenn man einen move findet, der langsam oder lange genug ist, um innerhalb dieses moves eine
#      volle beschleunigung von 0 auf vnominal und zurück auf 0 zu machen, so kann an diesem
#      move M getrennt werden.
#      Begründung: Move M wirkt als puffer oder "entkoppler" zwischen den teilpfaden. Es ist egal,
#      was in zukunft mit der anfangsgeschwindigkeit V0 oder mit der endgeschwindigkeit V1 passiert, es wird
#      jeweils keine auswirkung auf die jeweils "andere seite" haben.
#
# * Dieses kriterium kann noch entschärft werden:
#   - Falls man den planForward schritt so ausführt, das die anfangsgeschwindigkeit V0 von M bekannt und
#     unveränderlich (durch weitere planugsschritte) ist, so reicht als bedingung für den entkoppler move:
#
#      Wenn man einen move findet, der langsam oder lange genug ist, um innerhalb dieses moves eine
#      volle beschleunigung von V0 auf vnominal und zurück auf 0 zu machen, so kann an diesem
#      move M getrennt werden.
#      Begründung: Move M wirkt als puffer oder "entkoppler" zwischen den teilpfaden. Es ist egal,
#
#
#
#
#
#
# Retraction:
# -----------
#
# G10, G11, end-of-print-retraction
#
# Compute extrude length from extrude volume:
# -------------------------------------------
#
# From UM-Marlin:
#   volume_to_filament_length[e] = 1.0 / (M_PI * (material[e].diameter / 2.0) * (material[e].diameter / 2.0));
#
#
#
# Microstepping:
# --------------
# Der UM2 macht microstepping, siehe folgende liste, das ist aber bereits in den steps_per_unit berücksichtigt.
# micorsteps = [16, 16, 8, 16]
#
# Homing:
# -------
#
# Für jede dimension:
#   * erzeuge ein move kommando, dass ausreichend lang in die richtige richtung fährt. Geschiwndigkeit: HOMING_FEEDRATE
#   * druckkopf gegen endstop fahren lassen
#   * etwas zurück (HOME_RETRACT_MM) und langsam nochmals gegen den endstop
#   * aktuelle position vom drucker abfragen current_pos_steps
#
#
# xxx Moves für jede dimension:
#   * set dir
#   * big move 1.5, HOMING_FEEDRATE
#   * set reverse dir
#   * HOME_RETRACT_MM, HOMING_FEEDRATE
#   * set dir
#   * 1.5*HOME_RETRACT_MM, HOMING_FEEDRATE/3
#   * set reverse dir
#   * xxx solange bis endstop öffnet, HOMING_FEEDRATE/3
#
# --> spezialisierte homing funktion in firmware mit eigener beschleunigungsberechnung und
#     ablauf im mainthread -> stepper interrupt kann sauber gehalten werden.
# --> stepper irq prüft keine endstops mehr, das wird durch reine software enstops und einen
#     10ms zeitschleife erledigt.
#
#
# USB packet format:
# --------------------
#
# * Startbyte (SOH, 0x81)
# * Command counter 8 bit 0
# * Commandbyte         buffered (print) commands (G1...), direct commands (Temp, start, stop)
#    + commands < 128: buffered commands
#    + commands >= 128: direct commands
# * Length of command packet 32 bits, this includes the length of the variable command payload.
#   Or length of sub block (1-512).
# * if length > 0: command specific parameters/data
# * checksum 16 bits
#
# Speicherung eines buffered commands:
#   - entferne startbye, commandcounter am anfang
#   - entferne checksum am ende
#   - speichere das kommando
#
#
# Move segment data, OLD:
#   * Header data:
#       + number of accel steps, naccel, 16 bits
#       + number of linear steps, nlinear, 16 bits
#       + constant linear timer value, 16 bits
#       + number of deccel steps, ndeccel, 16 bits
#
#   * accel steps: naccel*(stepbits (8bit) + timer value(16bits))
#
#   * steps: nlinear * (stepbits (8bit))
#
#   * deccel steps: ndeccel*(stepbits (8bit) + timer value(16bits))
#
# Move segment data, new with bresenham in firmware:
#   * Header data:
#       + index lead axis, 8 bits
#       + array of absolute steps, 5 * 32 bits
#       + number of accel steps, naccel, 16 bits
#       + constant linear timer value, 16 bits
#       + number of deccel steps, ndeccel, 16 bits
#
#   * accel steps: naccel*(timer value(16bits))
#
#   * deccel steps: ndeccel*(timer value(16bits))
#
# Note: 16 bit is enough for 819mm/327mm/232mm (XY,Z,E) acceleration/linear/decceleration distances. 
#
#
#
#
# Commands: 
#
from ddprintcommands import *

#
# Printer states:
#
from ddprintstates import *

#
# Drucker koordinatensystem (UM):
# ----------------------------
#
# X: positive richtung nach rechts, endstop in MIN richtung
# Y: positive richtung nach hinten, endstop in MAX richtung
# Z: positive richtung nach unten, endstop in MAX richtung
#

############################################################################
# Constants
############################################################################

def plotArrow(f, v, startv, color="blue"):

    f.write("set arrow from %f,%f,%f to %f,%f,%f linetype 3 linewidth 5 linecolor rgb \"%s\"\n" % (startv[0], startv[1], startv[2], startv[0]+v[0], startv[1]+v[1], startv[2]+v[2], color))

def getMaxKoord(v, maxkoord):
    for i in range(2):
        if v[i] > maxkoord[0]:
            maxkoord[0] = v[i]
    if v[2] > maxkoord[1]:
        maxkoord[1] = v[2]

def plotSpeedChange(v1, v2, jerk, diff, title="Velocity X/Y/Z", fn="/tmp/2v.gnuplot"):

    # maxx = (max(v1[0], v2[0]) + diff[0]) * 2
    # maxy = (max(v1[1], v2[1]) + diff[1]) * 2
    maxkoord = [0, 0]

    f = open(fn, "w")
    f.write("set xyplane 0\n")
    f.write("set grid\n")

    if jerk[0]:
        v = [jerk[0], 0, 0]
        getMaxKoord(v, maxkoord)
        plotArrow(f, v, v1, "green")

    if jerk[1]:
        v = [0, jerk[1], 0]
        getMaxKoord(v, maxkoord)
        plotArrow(f, v, v1, "green")

    if jerk[2]:
        v = [0, 0, jerk[2]]
        getMaxKoord(v, maxkoord)
        plotArrow(f, v, v1, "green")

    getMaxKoord(jerk, maxkoord)
    plotArrow(f, jerk, v1)

    getMaxKoord(diff, maxkoord)
    plotArrow(f, diff, v1, "purple")

    getMaxKoord(v1, maxkoord)
    getMaxKoord(v2, maxkoord)

    f.write("set xrange [0:%f]\n" % (maxkoord[0]*1.2))
    f.write("set yrange [0:%f]\n" % (maxkoord[0]*1.2))
    f.write("set zrange [0:%f]\n" % (maxkoord[1]*1.2))

    f.write("splot \"-\" using 1:2:3:4:5:6 with vectors filled head title \"%s\"\n" % title)

    f.write("0 0 0 ")
    for v in v1[:3]:
        f.write("%f " % v)
    f.write("\n")

    f.write("0 0 0 ")
    for v in v2[:3]:
        f.write("%f " % v)
    f.write("\n")

    f.write("e\n")
    f.close()

def plot2v(v1, v2, jerk, diff, title="Velocity X/Y/Z", fn="/tmp/2v.gnuplot"):

    # maxx = (max(v1[0], v2[0]) + diff[0]) * 2
    # maxy = (max(v1[1], v2[1]) + diff[1]) * 2
    maxkoord = [0, 0]

    f = open(fn, "w")
    f.write("set xyplane 0\n")
    f.write("set grid\n")

    if jerk[0]:
        v = [jerk[0], 0, 0]
        getMaxKoord(v, maxkoord)
        plotArrow(f, v, v1, "green")

    if jerk[1]:
        v = [0, jerk[1], 0]
        getMaxKoord(v, maxkoord)
        plotArrow(f, v, v1, "green")

    if jerk[2]:
        v = [0, 0, jerk[2]]
        getMaxKoord(v, maxkoord)
        plotArrow(f, v, v1, "green")

    getMaxKoord(jerk, maxkoord)
    plotArrow(f, jerk, v1)

    if diff:
        getMaxKoord(diff, maxkoord)
        plotArrow(f, diff, v1, "purple")

    getMaxKoord(v1, maxkoord)
    getMaxKoord(v2, maxkoord)

    f.write("set xrange [0:%f]\n" % (maxkoord[0]*1.2))
    f.write("set yrange [0:%f]\n" % (maxkoord[0]*1.2))
    f.write("set zrange [0:%f]\n" % (maxkoord[1]*1.2))

    f.write("splot \"-\" using 1:2:3:4:5:6 with vectors filled head title \"%s\"\n" % title)

    f.write("0 0 0 ")
    for v in v1[:3]:
        f.write("%f " % v)
    f.write("\n")

    f.write("0 0 0 ")
    for v in v2[:3]:
        f.write("%f " % v)
    f.write("\n")

    f.write("e\n")
    f.close()

# plot x/y/E
def plot2vE(v1, v2, jerk, diff):
    plot2v(v1[:2] + [v1[3]], v2[:2] + [v2[3]],jerk[:2] + [jerk[3]],diff[:2] + [diff[3]], title="Velocity X/Y/E")

def plot4v(nom1, nom2, v1, v2, jerk, diff, fn = "/tmp/4v.gnuplot"):

    maxx = (max(nom1[0], nom2[0]) + diff[0]) * 2
    maxy = (max(nom1[1], nom2[1]) + diff[1]) * 2

    f = open(fn, "w")
    f.write("set xyplane 0\n")
    f.write("set grid\n")
    f.write("set xrange [0:%f]\n" % max(maxx, maxy))
    f.write("set yrange [0:%f]\n" % max(maxx, maxy))
    f.write("set zrange [0:100]\n")

    if jerk[0]:
        plotArrow(f, [jerk[0], 0, 0], v1, "green")

    if jerk[1]:
        plotArrow(f, [0, jerk[1], 0], v1, "green")

    if jerk[2]:
        plotArrow(f, [0, 0, jerk[2]], v1, "green")

    plotArrow(f, jerk, v1)

    plotArrow(f, diff, v1, "purple")

    f.write("splot \"-\" using 1:2:3:4:5:6 with vectors filled head\n")


    f.write("0 0 0 ")
    for v in nom1[:3]:
        f.write("%f " % v)
    f.write("\n")


    for v in nom1[:3]:
        f.write("%f " % v)
    for i in range(3):
        v = v1[i]
        f.write("-%f " % (nom1[i] - v))
    f.write("\n")


    f.write("0 0 0 ")
    for v in nom2[:3]:
        f.write("%f " % v)
    f.write("\n")


    for v in nom2[:3]:
        f.write("%f " % v)
    for i in range(3):
        v = v2[i]
        f.write("-%f " % (nom2[i] - v))
    f.write("\n")

    f.write("e\n")
    f.close()

def initParser(args, mode=None, gui=None):

    printerProfileName = "UM2" # xxx get from commandline args

    # profile = profile.Profile(printerProfileName)
    # parser.setProfile(profile)

    # Create printer profile singleton instance
    PrinterProfile(printerProfileName)
    # Create material profile singleton instance
    mat = MatProfile(args.mat)

    # Overwrite settings from profile with command line arguments:
    if args.t0:
        mat.override("bedTemp", args.t0)
    if args.t1:
        mat.override("hotendBaseTemp", args.t1)

    nozzle = NozzleProfile(args.nozzle)

    # Create the Printer singleton instance
    printer = Printer(gui)

    # Create the planner singleton instance
    planner = Planner(args, gui)

    # Create parser singleton instance
    # parser = gcodeparser.UM2GcodeParser()
    parser = gcodeparser.UM2GcodeParser()

    return (parser, planner, printer)

def main():

    argParser = argparse.ArgumentParser(description='%s, Direct Drive USB Print.' % sys.argv[0])
    argParser.add_argument("-d", dest="device", action="store", type=str, help="Device to use, default: /dev/ttyACM0.", default="/dev/ttyACM0")
    argParser.add_argument("-b", dest="baud", action="store", type=int, help="Baudrate, default 115200.", default=115200)
    # argParser.add_argument("-b", dest="baud", action="store", type=int, help="Baudrate, default 230400.", default=230400)
    # argParser.add_argument("-b", dest="baud", action="store", type=int, help="Baudrate, default 500000.", default=500000)
    # argParser.add_argument("-b", dest="baud", action="store", type=int, help="Baudrate, default 1000000.", default=1000000)

    argParser.add_argument("-t0", dest="t0", action="store", type=int, help="Temp 0 (heated bed), default comes from mat. profile.")
    argParser.add_argument("-t1", dest="t1", action="store", type=int, help="Temp 1 (hotend 1), default comes from mat. profile.")

    argParser.add_argument("-mat", dest="mat", action="store", help="Name of material profile to use [pla, abs...], default is pla.", default="pla_3mm")
    argParser.add_argument("-noz", dest="nozzle", action="store", help="Name of nozzle profile to use [nozzle40, nozzle80...], default is nozzle40.", default="nozzle40")

    argParser.add_argument("-np", dest="noPrime", action="store_const", const=True, help="Debug: don't prime nozzle, to test extrusion-less moves.")

    # fake endstops as long we have no real ones
    argParser.add_argument("-F", dest="fakeendstop", action="store", type=bool, help="Debug: fake endstops", default=False)
    argParser.add_argument("-nc", dest="noCoolDown", action="store", type=bool, help="Debug: don't wait for heater cool down after print.", default=False)

    argParser.add_argument("-fr", dest="feedrate", action="store", type=float, help="Feedrate for move commands.", default=0)

    subparsers = argParser.add_subparsers(dest="mode", help='Mode: mon(itor)|print|store|reset|pre(process).')

    sp = subparsers.add_parser("autoTune", help=u"Autotune hotend PID values.")

    sp = subparsers.add_parser("binmon", help=u"Monitor serial printer interface (binary responses).")

    sp = subparsers.add_parser("changenozzle", help=u"Heat hotend and change nozzle.")

    sp = subparsers.add_parser("mon", help=u"Monitor serial printer interface (asci).")

    sp = subparsers.add_parser("print", help=u"Download and print file at once.")
    sp.add_argument("gfile", help="Input GCode file.")

    # sp = subparsers.add_parser("store", help=u"Store file as USB.G on sd-card.")
    # sp.add_argument("gfile", help="Input GCode file.")

    sp = subparsers.add_parser("writeEepromFloat", help=u"Store float value into eeprom.")
    sp.add_argument("name", help="Valuename.")
    sp.add_argument("value", action="store", type=float, help="value (float).")

    # sp = subparsers.add_parser("reset", help=u"Try to stop/reset printer.")

    sp = subparsers.add_parser("pre", help=u"Preprocess gcode, for debugging purpose.")
    sp.add_argument("gfile", help="Input GCode file.")

    sp = subparsers.add_parser("dumpeeprom", help=u"dump eeprom settings.")

    sp = subparsers.add_parser("factoryReset", help=u"FactoryReset of eeprom settings, new bed leveling needed.")

    sp = subparsers.add_parser("test", help=u"Debug: tests for debugging purpose.")

    sp = subparsers.add_parser("disableSteppers", help=u"Disable stepper current (this dis-homes the printer).")

    sp = subparsers.add_parser("home", help=u"Home the printer.")

    sp = subparsers.add_parser("moverel", help=u"Debug: Move axis manually, relative coords.")
    sp.add_argument("axis", help="Axis (XYZAB).", type=str)
    sp.add_argument("distance", action="store", help="Move-distance (+/-) in mm.", type=float)

    sp = subparsers.add_parser("moveabs", help=u"Debug: Move axis manually, absolute coords.")
    sp.add_argument("axis", help="Axis (XYZAB).", type=str)
    sp.add_argument("distance", action="store", help="Move-distance (+/-) in mm.", type=float)

    sp = subparsers.add_parser("insertFilament", help=u"Insert filament (heatup, forward filament).")

    sp = subparsers.add_parser("removeFilament", help=u"Remove filament (heatup, retract filament).")

    sp = subparsers.add_parser("bedLeveling", help=u"Do bed leveling sequence.")

    sp = subparsers.add_parser("bedLevelAdjust", help=u"Adjust bedleveling offset - dangerous.")
    sp.add_argument("distance", action="store", help="Adjust-distance (+/-) in mm.", type=float)

    sp = subparsers.add_parser("heatHotend", help=u"Heat up hotend (to clean it, etc).")

    sp = subparsers.add_parser("genTempTable", help=u"Generate extrusion rate limit table.")

    sp = subparsers.add_parser("getEndstops", help=u"Get current endstop state.")

    sp = subparsers.add_parser("getFilSensor", help=u"Get current filament position.")

    sp = subparsers.add_parser("getpos", help=u"Get current printer and virtual position.")

    sp = subparsers.add_parser("getTemps", help=u"Get current temperatures (Bed, Extruder1, [Extruder2]).")

    sp = subparsers.add_parser("getTempTable", help=u"Output temperature-speed table.")

    sp = subparsers.add_parser("getStatus", help=u"Get current printer status.")

    sp = subparsers.add_parser("zRepeatability", help=u"Debug: Move Z to 10 random positions to test repeatability.")

    sp = subparsers.add_parser("stop", help=u"Stop print, cooldown, home, disable steppers.")

    sp = subparsers.add_parser("stepResponse", help=u"Measure and plot stepResponse of hotend PID.")

    sp = subparsers.add_parser("retract", help=u"Debug: Do the end-of-print retract manually after heating up.")

    sp = subparsers.add_parser("fanspeed", help=u"Set fan speed manually.")
    sp.add_argument("speed", help="Fanspeed 0 - 255.", type=int)

    sp = subparsers.add_parser("testFilSensor", help=u"Debug: move filament manually, output filament sensor measurement.")
    sp.add_argument("distance", action="store", help="Move-distance (+/-) in mm.", type=float)

    sp = subparsers.add_parser("calibrateFilSensor", help=u"Debug: helper to determine the ratio of stepper to flowrate sensor.")
    # sp.add_argument("distance", action="store", help="Move-distance (+/-) in mm.", type=float)

    args = argParser.parse_args()
    # print "args: ", args

    (parser, planner, printer) = initParser(args, mode=args.mode)

    steps_per_mm = PrinterProfile.getStepsPerMMVector()

    if args.mode == 'autoTune':

        util.zieglerNichols(args, parser)

    elif args.mode == 'changenozzle':

        util.changeNozzle(args, parser)

    elif args.mode == "binmon":
        printer.initSerial(args.device, args.baud)
        while True:
            try:
                (cmd, length, payload) = printer.readResponse()        
            except RxTimeout:
                pass
            else:
                print "Response cmd    :", cmd
                print "Response len    :", length
                print "Response payload:", payload.encode("hex")
                printer.checkErrorResponse(cmd, length, payload, False)

    elif args.mode == 'print':

        util.commonInit(args, parser)

        t0 = MatProfile.getBedTemp()
        t1 = MatProfile.getHotendBaseTemp()

        # Send heat up  command
        print "\nPre-Heating bed...\n"
        printer.heatUp(HeaterBed, t0)
        print "\nPre-Heating extruder...\n"
        printer.heatUp(HeaterEx1, 150)

        # Send printing moves
        # f = open(args.gfile)
        f = parser.preParse(args.gfile)

        # Send priming moves
        if not args.noPrime:
            util.prime(parser)

        lineNr = 0
        printStarted = False

        for line in f:
            parser.execute_line(line)

            #
            # Send more than one 512 byte block for dlprint
            #
            if lineNr > 1000 and (lineNr % 250) == 0:
                # check temp and start print

                if  not printStarted:

                    print "\nHeating bed (t0: %d)...\n" % t0
                    printer.heatUp(HeaterBed, t0, t0)
                    print "\nHeating extruder (t1: %d)...\n" % t1
                    printer.heatUp(HeaterEx1, t1, wait=0.95 * t1)

                    # Send print command
                    printer.sendCommandParamV(CmdMove, [MoveTypeNormal])
                    printStarted = True

                else:

                    # Stop sending moves on error
                    status = printer.getStatus()
                    pprint.pprint(status)
                    if not printer.stateMoving(status):
                        break

            lineNr += 1

        print "Parsed %d gcode lines." % lineNr

        # 
        # Add a move to lift the nozzle from the print if not ultigcode flavor
        # 
        if not parser.ultiGcodeFlavor:
            util.endOfPrintLift(parser)

        planner.finishMoves()
        printer.sendCommand(CmdEOT)

        # XXX start print if less than 1000 lines or temp not yet reached:
        if not printStarted:

            print "\nHeating bed (t0: %d)...\n" % t0
            printer.heatUp(HeaterBed, t0, t0)
            print "\nHeating extruder (t1: %d)...\n" % t1
            printer.heatUp(HeaterEx1, t1, wait=0.95 * t1)

            # Send print command
            printer.sendCommandParamV(CmdMove, [MoveTypeNormal])

        printer.waitForState(StateIdle)

        printer.coolDown(HeaterEx1)
        printer.coolDown(HeaterBed)

        ddhome.home(parser, args.fakeendstop)

        printer.sendCommand(CmdDisableSteppers)

        if not args.noCoolDown:
            printer.coolDown(HeaterEx1, wait=150)
            printer.coolDown(HeaterBed, wait=55)

        printer.readMore()

        ### Simulator/profiling
        ### printer.sendCommand(CmdExit)

    elif args.mode == "pre":

        # Virtuelle position des druckkopfes falls 'gehomed'
        homePosMM = util.MyPoint(
            X = planner.X_HOME_POS,
            Y = planner.Y_HOME_POS,
            Z = planner.Z_HOME_POS, #  - 20,
            )
        parser.set_position(homePosMM)

        f = parser.preParse(args.gfile)
        lineNr = 0
        for line in f:
            parser.execute_line(line)
            lineNr += 1

        print "Parsed %d gcode lines." % lineNr

        planner.finishMoves()

    elif args.mode == "mon":
        printer.initSerial(args.device, args.baud)
        while True:
            printer.readMore()

    elif args.mode == 'dumpeeprom':

        printer.commandInit(args)
        resp = printer.query(CmdGetEepromVersion)
        if util.handleGenericResponse(resp):
            print "Eepromversion: ", util.getResponseString(resp[2], 1)

        # pprint.pprint(printer.query(CmdGetEepromSettings))
        add_homeing = printer.getAddHomeing()
        print "add_homeing: ", add_homeing

    elif args.mode == 'factoryReset':

        printer.commandInit(args)
        printer.sendCommand(CmdEepromFactory)

    elif args.mode == 'disableSteppers':

        printer.commandInit(args)
        printer.sendCommand(CmdDisableSteppers)

    elif args.mode == 'moverel':

        assert(args.axis.upper() in "XYZAB")

        printer.commandInit(args)
        axis = util.dimIndex[args.axis.upper()]
        util.manualMove(parser, axis, args.distance, args.feedrate)

    elif args.mode == 'moveabs':

        assert(args.axis.upper() in "XYZAB")

        printer.commandInit(args)
        axis = util.dimIndex[args.axis.upper()]
        util.manualMove(parser, axis, args.distance, args.feedrate, True)

    elif args.mode == 'insertFilament':

        util.insertFilament(args, parser)

    elif args.mode == 'removeFilament':

        util.removeFilament(args, parser)

    elif args.mode == 'bedLeveling':

        util.bedLeveling(args, parser)

    elif args.mode == 'bedLevelAdjust':

        util.bedLevelAdjust(args, parser)

    elif args.mode == 'heatHotend':

        util.heatHotend(args, parser)

    elif args.mode == 'genTempTable':

        util.genTempTable(printer)

    elif args.mode == 'getEndstops':

        printer.commandInit(args)
        res = printer.getEndstops()
        print "Endstop state: ", res
    
    elif args.mode == 'getFilSensor':

        printer.commandInit(args)
        res = printer.query(CmdGetFilSensor)
        print "Filament pos:", res, "counts %.3f mm" % (res*25.4/1000)

    elif args.mode == 'getpos':

        printer.commandInit(args)

        res = printer.getPos()

        curPosMM = util.MyPoint(
            X = res[0] / float(steps_per_mm[0]),
            Y = res[1] / float(steps_per_mm[1]),
            Z = res[2] / float(steps_per_mm[2]),
            A = res[3] / float(steps_per_mm[3]),
            # B = res[4] / float(steps_per_mm[4]),
            )

        (homePosMM, homePosStepped) = planner.getHomePos()

        print "Printer pos [steps]:", res
        print "Printer pos [mm]:", curPosMM
        print "Virtual home pos [mm]: ", homePosMM

    elif args.mode == 'getTemps':

        printer.commandInit(args)
        printer.getTemps()

    elif args.mode == 'getTempTable':

        printer.commandInit(args)
        (baseTemp, tempTable) = printer.getTempTable()
        print "tempTable: ", pprint.pprint(tempTable)
        util.printTempTable(printer, baseTemp, tempTable)

    elif args.mode == 'getStatus':

        printer.commandInit(args)
        status = printer.getStatus()
        print "Status: "
        pprint.pprint(status)

    elif args.mode == 'home':

        printer.commandInit(args)
        ddhome.home(parser, args.fakeendstop)

    elif args.mode == 'zRepeatability':

        util.zRepeatability(parser)

    elif args.mode == 'stepResponse':

        util.stepResponse(args, parser)

    elif args.mode == 'retract':

        util.retract(args, parser)

    elif args.mode == 'stop':

        printer.commandInit(args)
        util.stopMove(args, parser)

    elif args.mode == 'fanspeed':

        printer.commandInit(args)
        printer.sendCommandParamV(CmdFanSpeed, [packedvalue.uint8_t(args.speed)])

    elif args.mode == 'testFilSensor':
        ddtest.testFilSensor(args, parser)

    elif args.mode == 'calibrateFilSensor':
        ddtest.calibrateFilSensor(args, parser)

    elif args.mode == 'test':

        printer.commandInit(args)
        util.downloadTempTable(printer)
        printer.readMore()

    elif args.mode == "writeEepromFloat":

        util.writeEEpromFloat(args, parser)

    else:
        print "Unknown command: ", args.mode
        assert(0)

if __name__ == "__main__":

    res = 0

    try:
      main()
    except:
        print "Exception: ", traceback.format_exc()
        res = 1

    sys.exit(res)










