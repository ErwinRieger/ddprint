#!/usr/bin/env python 
# encoding: utf-8 
#
#/*
# This file is part of ddprint - a direct drive 3D printer firmware.
# 
# Copyright 2015 erwin.rieger@ibrieger.de
# 
# ddprint is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by # the Free Software Foundation, either version 3 of the License, or
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

import logging, datetime, traceback
logging.basicConfig(filename=datetime.datetime.now().strftime("/tmp/ddprint_%y.%m.%d_%H:%M:%S.log"), level=logging.DEBUG)

import npyscreen , time, curses, sys, threading, Queue
import argparse
import ddprint, stoppableThread, ddhome, movingavg
import ddprintutil as util

from serial import SerialException
from ddprintcommands import *
from ddprintstates import *
from ddprofile import MatProfile, PrinterProfile
from ddprinter import FatalPrinterError

class SyncCall:
    def __init__(self, meth, *args):
        self.meth = meth
        self.args = args

    def call(self):
        self.meth(*self.args)
    
class SyncCallUpdate(SyncCall):
    def call(self):
        SyncCall.call(self)
        self.meth.im_self._need_update = True
  
"""
The text displayed next to the widget (if label=True) is generated by the translate_value method. This takes no options and returns a string. It makes sense to subclass the Slider object and overload this method. It probably makes sense to ensure that the string generated is of a fixed length. Thus the default code looks like:

    stri = "%s / %s" %(self.value, self.out_of)
    l = (len(str(self.out_of)))*2+4
    stri = stri.rjust(l)
    return stri
"""
class AdjSlider(npyscreen.Slider):

    def translate_value(self):

        stri = "%s" %(self.value - 15)
        return stri

class TempAdjSlider(AdjSlider):

    def __init__(self, *args, **keywords):
        self.app = keywords["app"]
        super(TempAdjSlider, self).__init__(*args, **keywords)

    def adjustTempLevel(self):

        self.app.log("AdjustTempLevel(): %s" %  (self.value - 15))
        self.app.printer.adjTemp(HeaterEx1, (self.value-15))
        # self.app.log( "Adjust temp level done")

    def set_value(self, val):
        
        super(TempAdjSlider, self).set_value(val)

        # Catch not-yet-set app case
        # try:
        self.app.cmdQueue.put(SyncCall(self.adjustTempLevel))
        # except AttributeError:
            # pass

    def get_value(self):
        return super(TempAdjSlider, self).get_value()

    value = property(get_value, set_value)

class TitleTempAdjSlider(npyscreen.TitleSlider):
    _entry_type = TempAdjSlider

class MainForm(npyscreen.FormBaseNew): 

    # Preheat bed (90%) and heater (50%)
    class Preheat_Button(npyscreen.MiniButtonPress):
        def whenPressed(self):
            return self.parent.preparePreheat()
    
    class Print_Button(npyscreen.MiniButtonPress):
        def whenPressed(self):
            return self.parent.preparePrintFile()
    
    class Quit_Button(npyscreen.MiniButtonPress):
        def whenPressed(self):
            # xxx check state
            # xxx cleanup?
            return self.parent.quit()
   
    class Stop_Button(npyscreen.MiniButtonPress):
        def whenPressed(self):
            if npyscreen.notify_yes_no("Stop Print?", title="Stop Print?"):
                self.parent.printThread.stop()
                return self.parent.prepareStopMove()
    
    def msgBox(self, msg):
        npyscreen.notify_confirm(msg)

    def quit(self, message=None):

        if message:
            self.msgBox(message)

        return self.exit_editing()

    def create(self):

        self.keypress_timeout = 1
        self.lastUpdate = time.time()
        self.gripAvg = movingavg.MovingAvg(10)

        self.printThread = None
        # self.threadStopCount = 0

        # Output queue, used by the printer thread to output 
        # information.
        self.guiQueue = Queue.Queue()
        # Command queue of the printer thread
        self.cmdQueue = Queue.Queue()

        self.printerState = None
        self.stateNames = ["IDLE", "INIT", "MOVING", "DWELL"]

        # t  = self.add(npyscreen.TitleText, name = "Text:",) 
        # t.entry_widget.add_handlers({ curses.ascii.NL: self.msgBox}) 

        w = self.columns/2
        h = self.lines/2

        #
        # Upper left side: the input/configuration area
        #
        rely = 2
        self.printerProfile = self.add(npyscreen.TitleFixedText, name = "PrinterProfile      :", relx=1, rely=rely, use_two_lines=False, begin_entry_at=23,
                width=w-1)
        self.printerProfile.editable = False

        rely += 1
        self.nozzleProfile = self.add(npyscreen.TitleFixedText, name = "Nozzle Profile      :", relx=1, rely=rely, use_two_lines=False, begin_entry_at=23,
                width=w-1)
        self.nozzleProfile.editable = False

        rely += 1
        self.matProfile = self.add(npyscreen.TitleFixedText, name = "Material Profile    :", relx=1, rely=rely, use_two_lines=False, begin_entry_at=23,
                width=w-1)
        self.matProfile.editable = False

        rely += 1
        self.smatProfile = self.add(npyscreen.TitleFixedText, name = "Specific Mat Profile:", relx=1, rely=rely, use_two_lines=False, begin_entry_at=23,
                width=w-1)
        self.smatProfile.editable = False

        rely += 2
        """
        Slider, TitleSlider
        Slider presents a horizontal slider. The following additional arguments to the constructor are useful:

            out_of=100
            The maximum value of the slider.
            step=1
            The increments by which a user my increase or decrease the value.
            lowest=0
            The minimum value a user can select. Note that sliders are not designed to allow a user to select negative values. lowest should be >= 0
            label=True
            Whether to print a text label next to the slider. If so, see the translate_value method.
            block_color = None
            The colour of the blocks that show the level of the slider. By default (None) the same value as the colour of the slider itself.
        """
        self.tempAdjust = self.add(TitleTempAdjSlider, name = "Temp. Level adjust  :", relx=1, rely=rely, use_two_lines=False, begin_entry_at=23,
                width=w-1, out_of=30, block_color="CONTROL", app=self)
        self.tempAdjust.value = 15

        rely += 2
        self.fn = self.add(npyscreen.TitleFilename, name = "GCode File          :", relx=1, rely=rely, use_two_lines=False, begin_entry_at=23,
                width=w-1)

        rely += 2
        self.add(MainForm.Preheat_Button, name = "Preheat", relx=1, rely=rely)
        self.add(MainForm.Print_Button, name = "Print", relx=10, rely=rely)
        self.add(MainForm.Stop_Button, name = "Stop", relx=20, rely=rely)
        self.add(MainForm.Quit_Button, name = "Quit", relx=30, rely=rely)

        #
        # Upper right side: the status area
        #
        rely = 2
        self.pState = self.add(npyscreen.TitleFixedText, name =   "Printer State       :", relx=w, rely=rely, use_two_lines=False, begin_entry_at=23)
        self.pState.editable = False

        rely += 1
        self.curT0 = self.add(npyscreen.TitleFixedText, name =    "T0 - Bed Temp       :", relx=w, rely=rely, use_two_lines=False, begin_entry_at=23)
        self.curT0.editable = False

        rely += 1
        self.curT1 = self.add(npyscreen.TitleFixedText, name =    "T1 - Hotend 1 Temp  :", relx=w, rely=rely, use_two_lines=False, begin_entry_at=23) 
        self.curT1.editable = False

        rely += 1
        self.curPWM = self.add(npyscreen.TitleFixedText, name =   "PWM - Hotend 1      :", relx=w, rely=rely, use_two_lines=False, begin_entry_at=23) 
        self.curPWM.editable = False

        rely += 1
        self.swapSize = self.add(npyscreen.TitleFixedText, name = "Size Swap File      :", relx=w, rely=rely, use_two_lines=False, begin_entry_at=25)
        self.swapSize.editable = False

        rely += 1
        self.sdrSize = self.add(npyscreen.TitleFixedText, name =  "Size SDReader       :", relx=w, rely=rely, use_two_lines=False, begin_entry_at=25)
        self.sdrSize.editable = False

        rely += 1
        self.sbSisze = self.add(npyscreen.TitleFixedText, name =  "Size StepBuffer     :", relx=w, rely=rely, use_two_lines=False, begin_entry_at=25)
        self.sbSisze.editable = False

        rely += 1
        self.underrun = self.add(npyscreen.TitleFixedText, name = "Stepbuffer underruns:", relx=w, rely=rely, use_two_lines=False, begin_entry_at=23)
        self.underrun.editable = False

        rely += 1
        self.extRate = self.add(npyscreen.TitleFixedText, name =  "Extrusion Rate      :", relx=w, rely=rely, use_two_lines=False, begin_entry_at=27)
        self.extRate.editable = False

        rely += 1
        self.extGrip = self.add(npyscreen.TitleFixedText, name =  "Feeder Grip         :", relx=w, rely=rely, use_two_lines=False, begin_entry_at=23)
        self.extGrip.editable = False

        rely += 1
        self.errors = self.add(npyscreen.TitleFixedText, name =   "Errors              :", relx=w, rely=rely, use_two_lines=False, begin_entry_at=23, color="WARNING") 
        # self.errors.editable = False

        #
        # Left side log window - the communication log
        #
        t = self.add(npyscreen.FixedText, value = "Communication Log:", relx=1, rely=h, color='LABEL') 
        t.editable = False

        self.commLog = self.add(npyscreen.BufferPager, maxlen=h, relx=1, rely=h+2, width=w-2) # , height=h-2, width=w-2)
        self.commLog.editable = False
        self.commLog._need_update = False
        # self.commLog.buffer(["Scrolling buffer:"])

        #
        # Right side log window - the application log
        #
        t = self.add(npyscreen.FixedText, value = "Application Log:", relx=w, rely=h, color='LABEL') 
        t.editable = False

        self.appLog = self.add(npyscreen.BufferPager, maxlen=h, relx=w, rely=h+2, width=w-3) # , height=h-2, width=w-2)
        self.appLog.editable = False
        self.appLog._need_update = False

    def while_waiting(self): 
       
        if not self.printThread:
            self.printThread = stoppableThread.StoppableThread(target=self.printWorker)
            self.printThread.daemon = True
            self.printThread.start()

        while not self.guiQueue.empty():
            obj = self.guiQueue.get()
            obj.call()

        #
        # Note: update() can raise a TypeError if we receive non-printable characters over the
        # usb-serial link. Ignore this exceptions here.
        #
        try:
            for w in self._widgets__: # [self.commLog, self.appLog]:
                if hasattr(w, '_need_update') and w._need_update:
                    w._need_update = False
                    w.update()
        except TypeError:
            self._log("while_waiting(): Ignoring exception: ", traceback.format_exc())


    def printWorker(self):

        parser = argparse.ArgumentParser(description='%s, Direct Drive USB Print.' % sys.argv[0])
        parser.add_argument("-d", dest="device", action="store", type=str, help="Device to use, default: /dev/ttyACM0.", default="/dev/ttyACM0")
        parser.add_argument("-b", dest="baud", action="store", type=int, help="Baudrate, default 500000.", default=500000)
        parser.add_argument("-f", dest="file", action="store", type=str, help="Gcode to print")
        parser.add_argument("-F", dest="fakeendstop", action="store", type=bool, help="fake endstops", default=False)
        parser.add_argument("-nc", dest="noCoolDown", action="store", type=bool, help="Debug: don't wait for heater cool down after print.", default=False)
        parser.add_argument("-t0", dest="t0", action="store", type=int, help="Temp 0 (heated bed), default comes from mat. profile.")
        parser.add_argument("-t1", dest="t1", action="store", type=int, help="Temp 1 (hotend 1), default comes from mat. profile.")
        parser.add_argument("-kAdvance", dest="kAdvance", action="store", type=float, help="K-Advance factor, default comes from mat. profile.")
        parser.add_argument("-mat", dest="mat", action="store", help="Name of material profile to use [pla, abs...], default is pla.", default="pla_1.75mm")
        parser.add_argument("-rl", dest="retractLength", action="store", type=float, help="Retraction length, default comes from printer profile.", default=0)
        parser.add_argument("-inctemp", dest="inctemp", action="store", type=int, help="Increase extruder temperature niveau (layer bonding).", default=0)
        parser.add_argument("-smat", dest="smat", action="store", help="Name of specific material profile to use.")
        parser.add_argument("-noz", dest="nozzle", action="store", help="Name of nozzle profile to use [nozzle40, nozzle80...], default is nozzle40.", default="nozzle40")

        self.args = parser.parse_args()
        self.args.mode = "dlprint"

        # print "args: ", self.args

        (self.parser, self.planner, self.printer) = ddprint.initParser(self.args, gui=self)
        # util.commonInit(self.args, self.parser)

        self.guiQueue.put(SyncCallUpdate(self.printerProfile.set_value, PrinterProfile.get().name))
        self.guiQueue.put(SyncCallUpdate(self.nozzleProfile.set_value, self.args.nozzle))
        self.guiQueue.put(SyncCallUpdate(self.matProfile.set_value, self.args.mat))
        if self.args.smat:
            self.guiQueue.put(SyncCallUpdate(self.smatProfile.set_value, self.args.smat))
        else:
            self.guiQueue.put(SyncCallUpdate(self.smatProfile.set_value, "---"))
        self.guiQueue.put(SyncCallUpdate(self.fn.set_value, self.args.file))
        
        try:
            self.printer.commandInit(self.args, PrinterProfile.getSettings())
        except SerialException:
            msg = "Can't open serial device '%s' (baudrate: %d)!\n\nPress OK to exit." % (self.args.device, self.args.baud)
            self.guiQueue.put(SyncCall(self.quit, msg))
            return

        self.mat_t0 = MatProfile.getBedTemp()
        self.mat_t0_reduced = MatProfile.getBedTempReduced()
        self.mat_t1 = MatProfile.getHotendStartTemp() + self.planner.l0TempIncrease

        while True:

            try:

                workDone = False

                while not self.cmdQueue.empty():
                    obj = self.cmdQueue.get()
                    self.log("call()...")
                    obj.call()
                    # self.log("...done")
                    workDone = True

                if time.time() - self.lastUpdate > 2.5:
                    # status = self.printer.getStatus()
                    # self.guiQueue.put(SyncCall(self.updateStatus, status))
                    self.printer.getStatus()

                    workDone = True
                    self.lastUpdate = time.time()

                if not workDone:
                    time.sleep(0.1)

            except stoppableThread.StopThread:
                self.printThread.incStopCount()
                self.log("caught StopThread, continue....")

    def updateTemps(self, t0, t1, targetT1):

        if t0 != None:

            if self.printerState >= StateInit:
                self.curT0.set_value("%8.1f / %.0f (%.0f) °C" % (t0, self.mat_t0, self.mat_t0_reduced))
            else:
                self.curT0.set_value("%8.1f / 0 (0) °C" % t0)

            self.curT0.update()

        if t1 != None:
            self.curT1.set_value( "%8.1f / %.0f °C" % (t1, targetT1))
            self.curT1.update()

    def updateStatus(self, status):

        self.printerState = status["state"]
        self.pState.set_value( "%8s" % self.stateNames[status["state"]])
        self.pState.update()
        self.updateTemps(status["t0"], status["t1"], status["targetT1"])
        self.curPWM.set_value( "%8d" % status["pwmOutput"])
        self.curPWM.update()
        self.swapSize.set_value( "%8s" % util.sizeof_fmt(status["Swap"]))
        self.swapSize.update()
        self.sdrSize.set_value( "%8s" % util.sizeof_fmt(status["SDReader"]))
        self.sdrSize.update()
        self.sbSisze.set_value( "%8s" % util.sizeof_fmt(status["StepBuffer"]))
        self.sbSisze.update()
        if status["StepBufUnderRuns"] > 0:
            self.underrun.set_value( "%8s" % str(status["StepBufUnderRuns"]))
        else:
            self.underrun.set_value( "       0" )

        self.underrun.update()

        self.extRate.set_value("%8s" % "todo mm³/s")
        self.extRate.update()

        slippage = status["slippage"]
        self.gripAvg.add(slippage)

        if slippage:
            if slippage <= (1/.85):
                self.extGrip.entry_widget.color = "GOOD"
            elif slippage <= (1/0.75):
                self.extGrip.entry_widget.color = "WARNING"
            else:
                self.extGrip.entry_widget.color = "DANGER"
            self.extGrip.set_value( "%8.1f, %8.1f %%" % (100.0/slippage, 100.0/self.gripAvg.mean()) )
        else:
            self.extGrip.entry_widget.color = "GOOD"
            self.extGrip.set_value( "   ?   ")

        self.extGrip.update()

    def display(self, clear=False):

        super(MainForm, self).display(clear)

        # try:
            # npyscreen.Form.display(self, clear)
        # except TypeError:
            # self._log("display(): Ignoring exception: ", traceback.format_exc())
            # return

        self.curses_pad.vline( 1, self.columns/2-1, curses.ACS_VLINE, self.lines-2)
        self.curses_pad.hline( self.lines/2-1, 1, curses.ACS_HLINE, self.columns/2-2)
        self.curses_pad.hline( self.lines/2-1, self.columns/2, curses.ACS_HLINE, self.columns/2-1)

    def preparePreheat(self):

        if self.printerState != StateIdle:
            self.msgBox("Can't start print - printer is not in IDLE state!.")
            return

        self.cmdQueue.put(SyncCall(self.preheat))

    def preheat(self):

        t = int(self.mat_t0 * 0.9)
        self.log( "\nPre-Heating bed (t0: %d)...\n" % t)
        self.printer.heatUp(HeaterBed, t)

        time.sleep(10)

        t = int(self.mat_t1 * 0.5)
        self.log( "\nPre-Heating extruder (t1: %d)...\n" % t)
        self.printer.heatUp(HeaterEx1, t)

    def preparePrintFile(self):

        if self.printerState != StateIdle:
            self.msgBox("Can't start print - printer is not in IDLE state!.")
            return

        # Unset printerstate to prevent second start of print, printerstate will be set by
        # updateStatus() again.
        self.printerState = None

        self.errors.set_value("")
        self.cmdQueue.put(SyncCall(self.startPrintFile))

    def prepareStopMove(self):
        self.cmdQueue.put(SyncCall(util.stopMove, self.args, self.parser))

    def startPrintFile(self):

        try:

            self.parser.reset()
            self.planner.reset()

            ddhome.home(self.parser, self.args.fakeendstop)

            # util.downloadTempTable(self.planner)
            # print "xxx pmwumbau, temptable"
            # util.downloadDummyTempTable(self.printer)
            print "xxx newdownloadTempTable"
            util.newdownloadTempTable(self.planner)

            self.printer.sendPrinterInit()

            # Send heat up  command
            self.log( "\nPre-Heating bed (t0: %d)...\n" % self.mat_t0)
            self.printer.heatUp(HeaterBed, self.mat_t0)

            time.sleep(10)

            t = int(self.mat_t1 * 0.5)
            self.log( "\nPre-Heating extruder (t1: %d)...\n" % t)
            self.printer.heatUp(HeaterEx1, t)

            f = self.parser.preParse(self.fn.get_value())
            
            util.prime(self.parser)

            lineNr = 0
            printStarted = False
            lastUpdate = time.time()

            for line in f:
                self.parser.execute_line(line)

                #
                # Check temp and start print
                #
                if time.time() > (lastUpdate + 0.5):

                    if lineNr > 1000 and not printStarted:

                        self.log( "\nHeating bed (t0: %d)...\n" % self.mat_t0 )
                        self.printer.heatUp(HeaterBed, self.mat_t0, wait=self.mat_t0)

                    
                        # avoid big overswing
                        self.log( "\nHeating extruder (t1: %d)...\n" % (self.mat_t1*0.9) )
                        self.printer.heatUp(HeaterEx1, self.mat_t1*0.9, self.mat_t1*0.9-2)

                        self.log( "\nHeating extruder (t1: %d)...\n" % self.mat_t1 )
                        self.printer.heatUp(HeaterEx1, self.mat_t1, self.mat_t1-2)

                        # Send print command
                        self.printer.sendCommandParamV(CmdMove, [MoveTypeNormal])
                        printStarted = True

                    status = self.printer.getStatus()
                    # self.guiQueue.put(SyncCall(self.updateStatus, status))
                    if printStarted and not self.printer.stateMoving(status):
                        break
               
                    # xxxx move to own method
                    if not self.cmdQueue.empty():
                        obj = self.cmdQueue.get()
                        self.log("hack call()...")
                        obj.call()
                        # self.log("hack ...done")

                    time.sleep(0.5) # give some cpu time to main/ui thread

                    lastUpdate = time.time()

                lineNr += 1

            print "Parsed %d gcode lines." % lineNr

            # 
            # Add a move to lift the nozzle from the print
            # 
            util.endOfPrintLift(self.parser)

            self.planner.finishMoves()
            self.printer.sendCommand(CmdEOT)

            # Start print if less than 1000 lines or temp not yet reached:
            if not printStarted:

                self.log( "\nHeating bed (t0: %d)...\n" % self.mat_t0 )
                self.printer.heatUp(HeaterBed, self.mat_t0, self.mat_t0)
                self.log( "\nHeating extruder (t1: %d)...\n" % self.mat_t1 )
                self.printer.heatUp(HeaterEx1, self.mat_t1, self.mat_t1-1)

                # Send print command
                self.printer.sendCommandParamV(CmdMove, [MoveTypeNormal])

            status = self.printer.getStatus()
            # self.guiQueue.put(SyncCall(self.updateStatus, status))
            while status['state'] != StateIdle:

                # xxxx move to own method
                if not self.cmdQueue.empty():
                    obj = self.cmdQueue.get()
                    self.log("hack 2 call()...")
                    obj.call()
                    self.log("hack 2 ...done")

                time.sleep(2)
                status = self.printer.getStatus()
                # self.guiQueue.put(SyncCall(self.updateStatus, status))

            self.printer.coolDown(HeaterEx1)
            self.printer.coolDown(HeaterBed)

            ddhome.home(self.parser, self.args.fakeendstop)

            self.printer.sendCommand(CmdDisableSteppers)

        except stoppableThread.StopThread:
            # Stop of current action requested
            self.printThread.incStopCount()
            # self.log("printFile(): Caught StopThread, bailing out.")
            return

        except FatalPrinterError, ex:
            self.log("printFile(): Caught FatalPrinterError: ", ex.msg)
            # Reset line numbers in case of a printer restart.
            self.printer.resetLineNumber()

        except:
            self.log("printFile(): Caught exception: ", traceback.format_exc())

    # def stopMove(self):
        # util.stopMove(self.parser)

    # Non-thread save version
    def _log(self, *args):
        s = util.stringFromArgs(*args)
        logging.info(s)
        self.appLog.buffer([s])
        self.appLog._need_update = True

    def _logError(self, err):
        # xxx manage error-log here ...
        logging.error( err )
        self.errors.set_value( err )
        self.errors.update()

    # Thread save version
    def log(self, *args):
        # self.appLog.buffer([util.stringFromArgs(*args)])
        s = util.stringFromArgs(*args)
        logging.info(s)
        self.guiQueue.put(SyncCallUpdate(self.appLog.buffer, [s]))

    def logError(self, *args):
        # xxx special error handling
        self.guiQueue.put(SyncCall(self._logError, util.stringFromArgs(*args)))

    def logSend(self, *args):
        # self.commLog.buffer([util.stringFromArgs(*args)])
        s = util.stringFromArgs(*args)
        logging.info("SEND: %s", s)
        self.guiQueue.put(SyncCallUpdate(self.commLog.buffer, [s]))

    def logRecv(self, *args):
        # self.commLog.buffer(["    " + util.stringFromArgs(*args) ])
        s = util.stringFromArgs(*args)
        logging.info("REPLY: %s", s)
        self.guiQueue.put(SyncCallUpdate(self.commLog.buffer, ["    " + s]))

    # Stdout redirection 
    def write(self, s):
        # self.appLog.buffer(["STDOUT:" + s])
        s = s.strip()
        if s:
            logging.info("STDOUT: %s", s)
            self.guiQueue.put(SyncCallUpdate(self.appLog.buffer, ["STDOUT:" + s]))

    def tempCb(self, t0=None, t1=None, targetT1=None):
        logging.info("tempCb: %s %s %s", str(t0), str(t1), str(targetT1))
        self.guiQueue.put(SyncCall(self.updateTemps, t0, t1, targetT1))

    def statusCb(self, status):
        self.guiQueue.put(SyncCall(self.updateStatus, status))

class Application(npyscreen.NPSAppManaged):

    def main(self): 

        F  = MainForm(name = "DDPrint UI")

        # Stdout redirection, xxx can be removed if all print's are replaced by log() calls...
        sys.stdout = F

        # Run eventloop, blocking call.
        F.edit() 

if __name__ == "__main__": 

    App = Application() 
    App.run() 








