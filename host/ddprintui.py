#!/usr/bin/env python 
# encoding: utf-8 
#
#/*
# This file is part of ddprint - a 3D printer firmware.
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

import logging, datetime, traceback, os, reprap_svr
logging.basicConfig(filename=datetime.datetime.now().strftime("/tmp/ddprint_%y.%m.%d_%H:%M:%S.log"), level=logging.DEBUG)

import npyscreen, time, curses, sys, threading, queue
import argparse
import stoppableThread, ddhome, movingavg, ddargs
import ddprintutil as util

# from serial import SerialException
from ddprintcommands import *
from ddprintstates import *
from ddprinter import FatalPrinterError
from ddplanner import initParser
from ddprintconstants import A_AXIS

class SyncCall:
    def __init__(self, meth, *args):
        self.meth = meth
        self.args = args

    def call(self):
        self.meth(*self.args)
    
class SyncCallUpdate(SyncCall):
    def call(self):
        SyncCall.call(self)
        self.meth.__self__._need_update = True
  
"""
The text displayed next to the widget (if label=True) is generated by the translate_value method. This takes no options and returns a string. It makes sense to subclass the Slider object and overload this method. It probably makes sense to ensure that the string generated is of a fixed length. Thus the default code looks like:

    stri = "%s / %s" %(self.value, self.out_of)
    l = (len(str(self.out_of)))*2+4
    stri = stri.rjust(l)
    return stri
"""
class AdjSlider(npyscreen.Slider):

    def translate_value(self):

        stri = "%.0f" % (self.value - 15)
        return stri

class TempAdjSlider(AdjSlider):

    def __init__(self, *args, **keywords):
        self.app = keywords["app"]
        super(TempAdjSlider, self).__init__(*args, **keywords)

    def adjustTempLevel(self):

        v = int(self.value - 15)
        self.app.log("AdjustTempLevel(): %d" % v)
        self.app.printer.adjTemp(HeaterEx1, v)
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

# Main TUI class
class MainForm(npyscreen.FormBaseNew): 

    def __init__(self, name, args):

        super(npyscreen.FormBaseNew, self).__init__(name)

        self.args = args

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
    
    def msgBox(self, msg, wide=False):
        npyscreen.notify_confirm(msg, wide=wide)

    # Display message and quit on fatal error
    def quit(self, message="", exception=None, traceback=None):

        wide = False
        if exception:
            wide = True
            message += "\nException: " + str(exception) 

        if traceback:
            wide = True
            message += "\n" + traceback

        if message:
            self.msgBox(message, wide=True)

        return self.exit_editing()

    def create(self):

        self.keypress_timeout = 1
        self.lastUpdate = time.time()

        self.gripAvg1 = movingavg.MovingAvg(10)
        self.gripAvg10 = movingavg.MovingAvg(100)

        self.frAvg1 = movingavg.MovingAvg(10)
        self.frAvg10 = movingavg.MovingAvg(100)

        self.printThread = None
        self.reprapThread = None
        # self.threadStopCount = 0

        # Output queue, used by the printer thread to output 
        # information.
        self.guiQueue = queue.Queue()
        # Command queue of the printer thread
        self.cmdQueue = queue.Queue()

        self.reprapServer = reprap_svr.ReprapServer(self)

        self.printerState = None
        self.stateNames = ["IDLE", "INIT", "ERASING", "PRINTING"]

        self.lastEPos = None
        self.lastTime = None

        # t  = self.add(npyscreen.TitleText, name = "Text:",) 
        # t.entry_widget.add_handlers({ curses.ascii.NL: self.msgBox}) 

        w = self.columns//2
        h = self.lines//2

        #
        # Upper left side: the input/configuration area
        #
        rely = 2
        self.printerProfileName = self.add(npyscreen.TitleFixedText, name = "PrinterProfile      :", relx=1, rely=rely, use_two_lines=False, begin_entry_at=23,
                max_width=w-1)
        self.printerProfileName.editable = False

        rely += 1
        self.nozzleProfile = self.add(npyscreen.TitleFixedText, name =  "Nozzle Profile      :", relx=1, rely=rely, use_two_lines=False, begin_entry_at=23,
                max_width=w-1)
        self.nozzleProfile.editable = False

        rely += 1
        self.matProfile = self.add(npyscreen.TitleFixedText, name =     "Material Profile    :", relx=1, rely=rely, use_two_lines=False, begin_entry_at=23,
                max_width=w-1)
        self.matProfile.editable = False

        rely += 1
        self.smatProfile = self.add(npyscreen.TitleFixedText, name =    "Specific Mat Profile:", relx=1, rely=rely, use_two_lines=False, begin_entry_at=23,
                max_width=w-1)
        self.smatProfile.editable = False

        rely += 1
        self.kAdvance = self.add(npyscreen.TitleFixedText, name =    "K-Advance           :", relx=1, rely=rely, use_two_lines=False, begin_entry_at=23,
                max_width=w-1)
        self.kAdvance.editable = False

        rely += 1
        self.wp = self.add(npyscreen.TitleFixedText, name =    "WorkingPoint        :", relx=1, rely=rely, use_two_lines=False, begin_entry_at=23,
                max_width=w-1)
        self.wp.editable = False

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
                max_width=w-1, out_of=30, block_color="CONTROL", app=self)
        self.tempAdjust.value = 15

        rely += 2
        self.fn = self.add(npyscreen.TitleFilename, name = "GCode File          :", relx=1, rely=rely, use_two_lines=False, begin_entry_at=23,
                max_width=w-1)

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
        self.swapSize = self.add(npyscreen.TitleFixedText, name = "Size Swap File/Swap :", relx=w, rely=rely, use_two_lines=False, begin_entry_at=23)
        self.swap = self.add(npyscreen.TitleFixedText, relx=w+int(w*0.5), max_width=int(0.25*w), rely=rely, use_two_lines=False, begin_entry_at=0)
        self.swapSize.editable = False
        self.swap.editable = False

        # rely += 1
        # self.sdrSize = self.add(npyscreen.TitleFixedText, name =  "Size SDReader       :", relx=w, rely=rely, use_two_lines=False, begin_entry_at=25)
        # self.sdrSize.editable = False

        # rely += 1
        # self.sbSisze = self.add(npyscreen.TitleFixedText, name =  "Size StepBuffer     :", relx=w, rely=rely, use_two_lines=False, begin_entry_at=25)
        # self.sbSisze.editable = False

        # rely += 1
        # self.underrun = self.add(npyscreen.TitleFixedText, name = "Stepbuffer underruns:", relx=w, rely=rely, use_two_lines=False, begin_entry_at=23)
        # self.underrun.editable = False

        rely += 1
        self.extGripC = self.add(npyscreen.TitleFixedText, name =  "Feeder Grip (avg)   :", relx=w, rely=rely, use_two_lines=False, begin_entry_at=23)
        self.extGrip = self.add(npyscreen.TitleFixedText, relx=w+int(w*0.5), max_width=int(0.25*w), rely=rely, use_two_lines=False, begin_entry_at=0)
        self.extGripC.editable = False
        self.extGrip.editable = False

        rely += 1
        self.slowdown = self.add(npyscreen.TitleFixedText, name =  "Slowdown            :", relx=w, rely=rely, use_two_lines=False, begin_entry_at=23)
        self.slowdown.editable = False

        rely += 1
        self.extRateC = self.add(npyscreen.TitleFixedText, name =  "Extrusion Rate (avg):", relx=w, rely=rely, use_two_lines=False, begin_entry_at=23)
        self.extRate = self.add(npyscreen.TitleFixedText, relx=w+int(w*0.5), max_width=int(0.25*w), rely=rely, use_two_lines=False, begin_entry_at=0)
        self.extRateC.editable = False
        self.extRate.editable = False

        rely += 1
        self.printDuration = self.add(npyscreen.TitleFixedText, name =   "Print duration      :", relx=w, rely=rely, use_two_lines=False, begin_entry_at=23)
        self.printDuration.editable = False

        rely += 1
        self.undergrip = self.add(npyscreen.TitleFixedText, name =       "Grip-low warning    :", relx=w, rely=rely, use_two_lines=False, begin_entry_at=23)
        self.undergrip.editable = False

        rely += 1
        self.undertemp = self.add(npyscreen.TitleFixedText, name =       "Temperature-low warn:", relx=w, rely=rely, use_two_lines=False, begin_entry_at=23)
        self.undertemp.editable = False

        rely += 1
        self.errors = self.add(npyscreen.TitleFixedText, name =   "Errors              :", relx=w, rely=rely, use_two_lines=False, begin_entry_at=23, color="WARNING") 
        # self.errors.editable = False

        #
        # Left side log window - the communication log
        #
        # t = self.add(npyscreen.FixedText, value = "Communication Log:", relx=1, rely=h, color='LABEL') 
        # t.editable = False

        # self.commLog = self.add(npyscreen.BufferPager, maxlen=h, relx=1, rely=h+2, max_width=w-2) # , height=h-2, width=w-2)
        # self.commLog.editable = False
        # self.commLog._need_update = False
        # self.commLog.buffer(["Scrolling buffer:"])

        #
        # Right side log window - the application log
        #
        t = self.add(npyscreen.FixedText, value = "Application Log:", relx=1, rely=h, color='LABEL') 
        t.editable = False

        self.appLog = self.add(npyscreen.BufferPager, maxlen=h, relx=1, rely=h+2, max_width=self.columns-4)
        self.appLog.editable = False
        self.appLog._need_update = False

    def while_waiting(self): 
       
        if not self.printThread:
            self.printThread = stoppableThread.StoppableThread(target=self.printWorker)
            self.printThread.daemon = True
            self.printThread.start()

        if not self.reprapThread:
            self.reprapThread = threading.Thread(target = self.reprapServer.run)
            self.reprapThread.daemon = True
            self.reprapThread.start()

        while not self.guiQueue.empty():
            obj = self.guiQueue.get()
            obj.call()

        #
        # Note: update() can raise a TypeError if we receive non-printable characters over the
        # usb-serial link. Ignore this exceptions here.
        #
        for w in self._widgets__:
            if hasattr(w, '_need_update') and w._need_update:
                w._need_update = False
                try:
                    w.update()
                except TypeError:
                    # self._log("while_waiting(): Ignoring exception: ", traceback.format_exc())
                    pass


    def printWorker(self):

      try:

        try:
            (self.printer, self.parser, self.planner) = initParser(self.args, gui=self)
        except IOError as ex:
            msg = "Can't open serial device '%s'." % self.args.device
            self.guiQueue.put(SyncCall(self.quit, msg, ex))
            return

        self.mat_t0 = self.planner.matProfile.getBedTemp()
        self.mat_t0_reduced = self.planner.matProfile.getBedTempReduced()
        self.mat_t0_wait = self.printer.printerProfile.getWeakPowerBedTemp()
        self.mat_t1 = self.planner.floorTemp + self.planner.l0TempIncrease

        self.guiQueue.put(SyncCallUpdate(self.printerProfileName.set_value, self.printer.printerProfile.name))
        self.guiQueue.put(SyncCallUpdate(self.nozzleProfile.set_value, self.args.nozzle))
        self.guiQueue.put(SyncCallUpdate(self.matProfile.set_value, self.args.mat))
        if self.args.smat:
            self.guiQueue.put(SyncCallUpdate(self.smatProfile.set_value, self.args.smat))
        else:
            self.guiQueue.put(SyncCallUpdate(self.smatProfile.set_value, "---"))
        self.guiQueue.put(SyncCallUpdate(self.kAdvance.set_value, self.planner.advance.getKAdv()))
        self.guiQueue.put(SyncCallUpdate(self.wp.set_value, self.args.workingPoint))
        self.guiQueue.put(SyncCallUpdate(self.fn.set_value, self.args.gfile))
        
        # try:
        self.printer.commandInit()
        # except SerialException, ex:
            # msg = "Can't open serial device '%s'!" % self.args.device
            # self.guiQueue.put(SyncCall(self.quit, msg, ex))
            # return
        ddhome.home(self.args, self.printer, self.parser, self.planner)
        util.downloadTempTable(self.printer, self.planner.nozzleProfile, self.planner.matProfile)

        while True:

            try:

                workDone = False

                while not self.cmdQueue.empty():
                    obj = self.cmdQueue.get()
                    # self.log("call()...")
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

      except Exception as ex:
          self.guiQueue.put(SyncCall(self.quit, "", ex, traceback.format_exc()))
          return

    def updateTemps(self, t0, t1, targetT1):

        if t0 != None:

            if self.printerState >= StateInit:
                self.curT0.set_value("%.1f / %.0f (%.0f) °C" % (t0, self.mat_t0, self.mat_t0_reduced))
            else:
                self.curT0.set_value("%.1f / 0 (0) °C" % t0)

            self.curT0.update()

        if t1 != None:
            self.curT1.set_value( "%.1f / %.0f °C" % (t1, targetT1))
            self.curT1.update()

    def updateStatus(self, status):

        self.printerState = status.state
        self.pState.set_value( "%s" % self.stateNames[status.state])
        self.pState.update()
        self.updateTemps(status.t0, status.t1, status.targetT1)
        self.curPWM.set_value( "%d" % status.pwmOutput)
        self.curPWM.update()
        self.swapSize.set_value( util.sizeof_fmt(status.swapsize))
        self.swapSize.update()
        self.swap.set_value( util.sizeof_fmt(status.Swap))
        self.swap.update()
        # self.sdrSize.set_value( util.sizeof_fmt(status.SDReader))
        # self.sdrSize.update()
        # self.sbSisze.set_value( util.sizeof_fmt(status.StepBuffer))
        # self.sbSisze.update()
        # if status.StepBufUnderRuns > 0:
            # self.underrun.set_value( "%8s" % str(status.StepBufUnderRuns))
        # else:
            # self.underrun.set_value( "       0" )
        # self.underrun.update()

        slippage = status.slippage
        self.gripAvg1.add(slippage)
        self.gripAvg10.add(slippage)

        slippageAvg1 = 0.0
        if self.gripAvg1.valid():
            slippageAvg1 = self.gripAvg1.mean()

        slippageAvg10 = 0.0
        if self.gripAvg10.valid():
            slippageAvg10 = self.gripAvg10.mean()

        color = "GOOD"

        if slippageAvg1 > 0:
        
            color = "DANGER"
            if slippage <= (1/.85):
                color = "GOOD"
            elif slippage <= (1/0.75):
                color = "WARNING"

            self.extGripC.set_value( "%.1f" % (100.0/slippageAvg1))
        else:
            self.extGripC.set_value( "--" )

        if slippageAvg10 > 0:
            self.extGrip.set_value( "%.1f %%" % (100.0/slippageAvg10) )
        else:
            self.extGrip.set_value( "--" )

        self.extGripC.entry_widget.color = color

        self.extGripC.update()
        self.extGrip.update()

        self.slowdown.set_value( "%.2f" % status.slowdown )
        self.slowdown.update()

        ePos = status.ePos
        t = time.time()

        e_steps_per_mm = self.printer.printerProfile.getStepsPerMMI(A_AXIS)

        if self.lastEPos != None:

            deltaE = ePos - self.lastEPos
            deltaTime = t - self.lastTime

            deltaEmm = float(deltaE)/e_steps_per_mm
            matArea = self.planner.matProfile.getMatArea()

            flowrate = (deltaEmm*matArea)/deltaTime
            self.frAvg1.add(flowrate)
            self.frAvg10.add(flowrate)

            if slippageAvg1 > 0 and slippageAvg10 > 0:
                fr1 = self.frAvg1.mean()
                fr10 = self.frAvg10.mean()
                self.extRateC.set_value("%.1f/%.1f" % (fr1/slippageAvg1, fr1))
                self.extRate.set_value("%.1f/%.1f mm³/s" % (fr10/slippageAvg10, fr10))
            else:
                self.extRateC.set_value( "--" )
                self.extRate.set_value( "--" )

            self.extRateC.entry_widget.color = color

            self.extRateC.update()
            self.extRate.update()

        self.lastEPos = ePos
        self.lastTime = t

        self.undergrip.set_value( "%s" % str(status.underGrip))
        self.undergrip.update()
        self.undertemp.set_value( "%s" % str(status.underTemp))
        self.undertemp.update()

        self.printDuration.set_value(self.printer.getPrintDuration())
        self.printDuration.update()

    def display(self, clear=False):

        # Todo: Why this exception ???
        """
            File "/usr/local/lib/python2.7/dist-packages/npyscreen/npyssafewrapper.py", line 41, in wrapper
                wrapper_no_fork(call_function)
            File "/usr/local/lib/python2.7/dist-packages/npyscreen/npyssafewrapper.py", line 97, in wrapper_no_fork
                return_code = call_function(_SCREEN)    
            File "/usr/local/lib/python2.7/dist-packages/npyscreen/apNPSApplication.py", line 25, in __remove_argument_call_main
                return self.main()
            File "./ddprintui.py", line 746, in main
                F.edit() 
            File "/usr/local/lib/python2.7/dist-packages/npyscreen/fm_form_edit_loop.py", line 47, in edit
                self.edit_loop()
            File "/usr/local/lib/python2.7/dist-packages/npyscreen/fm_form_edit_loop.py", line 41, in edit_loop
                self.handle_exiting_widgets(self._widgets__[self.editw].how_exited)
            File "/usr/local/lib/python2.7/dist-packages/npyscreen/fmForm.py", line 153, in handle_exiting_widgets
                self.how_exited_handers[condition]()
            File "/usr/local/lib/python2.7/dist-packages/npyscreen/fmForm.py", line 279, in find_next_editable
                self.display()
            File "./ddprintui.py", line 503, in display
                super(MainForm, self).display(clear)
            File "/usr/local/lib/python2.7/dist-packages/npyscreen/fmForm.py", line 321, in display
                w.update(clear=clear)
            File "/usr/local/lib/python2.7/dist-packages/npyscreen/wgmultiline.py", line 723, in update
                w.update(clear=True)
            File "/usr/local/lib/python2.7/dist-packages/npyscreen/wgtextbox.py", line 132, in update
                self._print()
            File "/usr/local/lib/python2.7/dist-packages/npyscreen/wgtextbox.py", line 299, in _print
                color
            TypeError: int,int,str,attr
        """
        try:
            # npyscreen.Form.display(self, clear)
            super(MainForm, self).display(clear)
        except TypeError:
            self._log("display(): Ignoring exception: ", traceback.format_exc())
            return

        self.curses_pad.vline( 1, self.columns//2-1, curses.ACS_VLINE, self.lines//2 - 2)
        self.curses_pad.hline( self.lines//2-1, 1, curses.ACS_HLINE, self.columns//2-2)
        self.curses_pad.hline( self.lines//2-1, self.columns//2, curses.ACS_HLINE, self.columns//2-1)

    def preparePreheat(self):

        if self.printerState > StateInit:
            self.msgBox("Can't start print - printer is not in IDLE state!.")
            return

        self.cmdQueue.put(SyncCall(self.preheat))
        self.cmdQueue.put(SyncCall(self.preheatHotend))

    def preheat(self):

        t = int(self.mat_t0 * 0.9)
        self.log( "Pre-Heating bed (t0: %d)...\n" % t)
        self.printer.heatUp(HeaterBed, t)

    def preheatHotend(self):

        if self.printerState > StateInit:
            return

        # Special handling of weak power supply
        temps = self.printer.getTemps()
        if temps[0] < self.mat_t0_wait:
            time.sleep(2)
            self.cmdQueue.put(SyncCall(self.preheatHotend))
            return

        t = int(self.mat_t1 * 0.5)
        self.log( "Pre-Heating extruder (t1: %d)...\n" % t)
        self.printer.heatUp(HeaterEx1, t)

    def preparePrintFile(self):

        if self.printerState > StateInit:
            self.msgBox("Can't start print - printer is not in IDLE state!.")
            return

        # Unset printerstate to prevent second start of print, printerstate will be set by
        # updateStatus() again.
        self.printerState = None

        self.errors.set_value("")

        fnGcode = self.fn.get_value()

        # Initialize printlog
        self.printLog = None
        fn = os.path.join(
                os.environ["HOME"],
                ".ddprint",
                "printlog",
                datetime.datetime.now().strftime("%y.%m.%d_%H:%M:%S-") + os.path.basename(fnGcode) + ".log")
        try:
            self.printLog = open(fn, "w")
        except IOError:
            self.logError("Can't open print log '%s'" % fn)

        # Update printlog with gcode file information
        try:
            stat = os.stat(fnGcode)
        except OSError as why:
            self.logError("Can't stat gcode input file: '%s'" % str(why))
            return

        self.logPrintLog("Gcode file    : %s\n" % fnGcode)
        self.logPrintLog("Gcode filesize: %s, %d bytes\n" % (util.sizeof_fmt(stat.st_size), stat.st_size))

        # Update printlog with used profiles:
        self.printer.printerProfile.logValues("Printer profile", self)
        self.planner.nozzleProfile.logValues("Nozzle profile", self)
        self.planner.matProfile.logValues("Material profile", self)
        self.logPrintLog("\n")

        self.cmdQueue.put(SyncCall(self.startPrintFile))

    def prepareStopMove(self):
        self.cmdQueue.put(SyncCall(util.stopMove, self.args, self.parser))

    def startPrintFile(self):

        # reset averages here...
        # self.parser.reset()
        # self.planner.reset()

        try:

            util.printFile(self.args, self.printer, self.parser, self.planner,
                self,
                self.fn.get_value(), self.mat_t0, self.mat_t0_wait, self.mat_t1)

            self.closePrintLog()

        except stoppableThread.StopThread:
            # Stop of current action requested
            self.printThread.incStopCount()
            # self.log("printFile(): Caught StopThread, bailing out.")
            return

        except FatalPrinterError as ex:
            self.log("printFile(): Caught FatalPrinterError: ", ex.msg)
            # Reset line numbers in case of a printer restart.
            self.printer.resetLineNumber()

        except Exception as ex:
            self.guiQueue.put(SyncCall(self.quit, "", ex, traceback.format_exc()))

    # def stopMove(self):
        # util.stopMove(self.parser)

    def closePrintLog(self):
        self.printLog.write("\nUser Notes:\n")
        self.printLog.write("  * Real printing time (Build time) : \n" + self.printer.getPrintDuration())
        self.printLog.write("  * Measured Weight (Plastic weight): \n")
        self.printLog.close()

    # Non-thread save version
    def _log(self, *args):
        s = util.stringFromArgs(*args)
        logging.info(s)
        self.appLog.buffer([s])
        self.appLog._need_update = True

        # Errors go into print log, too
        self.logPrintLog("Error: %s\n" % s)

    def _logError(self, err):
        # xxx manage error-log here ...
        logging.error( err )
        self.errors.set_value( err )
        self.errors.update()

    # Thread save version
    def log(self, *args):
        s = util.stringFromArgs(*args)
        logging.info(s)
        self.guiQueue.put(SyncCallUpdate(self.appLog.buffer, [s]))

    def logError(self, *args):
        # xxx special error handling
        self.guiQueue.put(SyncCall(self._logError, util.stringFromArgs(*args)))

    def logComm(self, *args):
        s = util.stringFromArgs(*args)
        # logging.info("CommLog: %s", s)
        # self.guiQueue.put(SyncCallUpdate(self.commLog.buffer, [s]))
        logging.info(s)
        self.guiQueue.put(SyncCallUpdate(self.appLog.buffer, [s]))

    def logPrintLog(self, s):
        if self.printLog:
            self.printLog.write(s)

    # Stdout redirection 
    def write(self, s):
        s = s.strip()
        if s:
            logging.info("STDOUT: %s", s)
            self.guiQueue.put(SyncCallUpdate(self.appLog.buffer, ["STDOUT:" + s]))

    def tempCb(self, t0=None, t1=None, targetT1=None):
        logging.info("tempCb: %s %s %s", str(t0), str(t1), str(targetT1))
        self.guiQueue.put(SyncCall(self.updateTemps, t0, t1, targetT1))

    def statusCb(self, status):
        self.guiQueue.put(SyncCall(self.updateStatus, status))
        self.reprapServer.setStatus(status)

class Application(npyscreen.NPSAppManaged):

    def __init__(self):

        super(npyscreen.NPSAppManaged, self).__init__()

        parser = argparse.ArgumentParser(description='%s - ddPrint TUI application.' % os.path.basename(sys.argv[0]))

        ddargs.addCommonArguments(parser)
        ddargs.addPrintArguments(parser)

        self.args = parser.parse_args()

        self.args.mode = "uiprint"

    def main(self): 

        F  = MainForm(name = "DDPrint UI", args=self.args)

        # Stdout redirection, xxx can be removed if all print's are replaced by log() calls...
        sys.stdout = F

        # Run eventloop, blocking call.
        # Todo: Why this exception ???
        """
        File "./ddprintui.py", line 778, in <module>
            App.run()
        File "/usr/local/lib/python2.7/dist-packages/npyscreen/apNPSApplication.py", line 30, in run
            return npyssafewrapper.wrapper(self.__remove_argument_call_main)
        File "/usr/local/lib/python2.7/dist-packages/npyscreen/npyssafewrapper.py", line 41, in wrapper
            wrapper_no_fork(call_function)
        File "/usr/local/lib/python2.7/dist-packages/npyscreen/npyssafewrapper.py", line 97, in wrapper_no_fork
            return_code = call_function(_SCREEN)
        File "/usr/local/lib/python2.7/dist-packages/npyscreen/apNPSApplication.py", line 25, in __remove_argument_call_main
            return self.main()
        File "./ddprintui.py", line 773, in main
            F.edit()
        File "/usr/local/lib/python2.7/dist-packages/npyscreen/fm_form_edit_loop.py", line 47, in edit
            self.edit_loop()
        File "/usr/local/lib/python2.7/dist-packages/npyscreen/fm_form_edit_loop.py", line 38, in edit_loop
            self._widgets__[self.editw].edit()
        File "/usr/local/lib/python2.7/dist-packages/npyscreen/wgwidget.py", line 447, in edit
            self._edit_loop()
        File "/usr/local/lib/python2.7/dist-packages/npyscreen/wgwidget.py", line 463, in _edit_loop
            self.get_and_use_key_press()
        File "/usr/local/lib/python2.7/dist-packages/npyscreen/wgwidget.py", line 599, in get_and_use_key_press
            self.handle_input(ch)
        File "/usr/local/lib/python2.7/dist-packages/npyscreen/wgwidget.py", line 95, in handle_input
            if self.parent.handle_input(_input):
        File "/usr/local/lib/python2.7/dist-packages/npyscreen/wgwidget.py", line 71, in handle_input
            self.handlers[_input](_input)
        File "/usr/local/lib/python2.7/dist-packages/npyscreen/fmForm.py", line 116, in _resize
            w._resize()
        File "/usr/local/lib/python2.7/dist-packages/npyscreen/wgwidget.py", line 315, in _resize
            self.resize()
        File "/usr/local/lib/python2.7/dist-packages/npyscreen/wgmultiline.py", line 671, in resize
            super(Pager, self).resize()
        File "/usr/local/lib/python2.7/dist-packages/npyscreen/wgmultiline.py", line 105, in resize
            self.display()
        File "/usr/local/lib/python2.7/dist-packages/npyscreen/wgwidget.py", line 418, in display
            self.update()
        File "/usr/local/lib/python2.7/dist-packages/npyscreen/wgmultiline.py", line 723, in update
            w.update(clear=True)
        File "/usr/local/lib/python2.7/dist-packages/npyscreen/wgtextbox.py", line 132, in update
            self._print()
        File "/usr/local/lib/python2.7/dist-packages/npyscreen/wgtextbox.py", line 299, in _print
            color
        TypeError: must be string without null bytes, not str
        """
        F.edit() 

if __name__ == "__main__": 



    App = Application() 
    App.run() 








