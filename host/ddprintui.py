#!/usr/bin/env python 
# encoding: utf-8 
#
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

import npyscreen , time, curses, sys, types, threading, Queue
import argparse
import ddprint
import ddprintutil as util

from ddprintcommands import *
from ddprintstates import *
from ddprofile import MatProfile

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
    
class MainForm(npyscreen.Form): 

    # Preheat bed (90%) and heater (50%)
    class Preheat_Button(npyscreen.MiniButtonPress):
        def whenPressed(self):
            self.parent.cmdQueue.put(SyncCall(self.parent.preheat))
    
    class Print_Button(npyscreen.MiniButtonPress):
        def whenPressed(self):
            return self.parent.preparePrintFile()
    
    class Quit_Button(npyscreen.MiniButtonPress):
        def whenPressed(self):
            # xxx check state
            # xxx cleanup?
            return self.parent.exit_editing()
    
    class Stop_Button(npyscreen.MiniButtonPress):
        def whenPressed(self):
            self.parent.msgBox("Not implemented.")
    
    def msgBox(self, msg):
        npyscreen.notify_confirm(msg)

    def create(self):

        self.keypress_timeout = 1
        # self.state = 0
        self.lastUpdate = time.time()

        self.printThread = None
        # Output queue, used by the printer thread to output 
        # information.
        self.guiQueue = Queue.Queue()
        # Command queue of the printer thread
        self.cmdQueue = Queue.Queue()

        self.stateNames = ["IDLE", "INIT", "MOVING"]

        # t  = self.add(npyscreen.TitleText, name = "Text:",) 
        # t.entry_widget.add_handlers({ curses.ascii.NL: self.msgBox}) 

        w = self.columns/2
        h = self.lines/2

        #
        # Upper left side: the action area
        #
        rely = 2
        self.fn = self.add(npyscreen.TitleFilename, name = "GCode File:", relx=1, rely=rely, use_two_lines=False, begin_entry_at=23,
                width=w-1)

        rely += 2
        self.add(MainForm.Preheat_Button, name = "Preheat", relx=1, rely=rely)
        self.add(MainForm.Print_Button, name = "Print", relx=10, rely=rely)
        self.add(MainForm.Stop_Button, name = "Stop", relx=20, rely=rely)
        self.add(MainForm.Quit_Button, name = "Quit", relx=30, rely=rely)

        #
        # Upper right side: the status area
        #
        rely = 1
        self.pState = self.add(npyscreen.TitleFixedText, name = "Printer State        :", relx=w, rely=rely, use_two_lines=False, begin_entry_at=23)
        self.pState.editable = False

        rely += 1
        self.curT0 = self.add(npyscreen.TitleFixedText, name = "T0 - Bed Temp        :", relx=w, rely=rely, use_two_lines=False,
            begin_entry_at=23, width = w/2) 
        self.curT0.editable = False

        self.curT1 = self.add(npyscreen.TitleFixedText, name = "T1 - Hotend 1 Temp   :", relx=int(w*1.5), rely=rely, use_two_lines=False, begin_entry_at=23) 
        self.curT1.editable = False

        rely += 1
        self.swapSize = self.add(npyscreen.TitleFixedText, name = "Size Swap File       :", relx=w, rely=rely, use_two_lines=False, begin_entry_at=23)
        self.swapSize.editable = False

        rely += 1
        self.sdrSize = self.add(npyscreen.TitleFixedText, name = "Size SDReader        :", relx=w, rely=rely, use_two_lines=False, begin_entry_at=23)
        self.sdrSize.editable = False

        rely += 1
        self.sbSisze = self.add(npyscreen.TitleFixedText, name = "Size StepBuffer      :", relx=w, rely=rely, use_two_lines=False, begin_entry_at=23)
        self.sbSisze.editable = False

        rely += 1
        self.underrun = self.add(npyscreen.TitleFixedText, name = "Step Buffer underruns:", relx=w, rely=rely, use_two_lines=False,
            begin_entry_at=23, width = w/2) 
        self.underrun.editable = False
        self.errors = self.add(npyscreen.TitleFixedText, name = "Errors:", relx=int(w*1.5), rely=rely, use_two_lines=False, begin_entry_at=8) 
        # self.errors.editable = False

        #
        # Left side log window - the communication log
        #
        t = self.add(npyscreen.FixedText, value = "Communication Log:", relx=1, rely=h, color='LABEL') 
        t.editable = False

        self.commLog = self.add(npyscreen.BufferPager, maxlen=h, relx=1, rely=h+1, width=w-2) # , height=h-2, width=w-2)
        self.commLog.editable = False
        self.commLog._need_update = False
        # self.commLog.buffer(["Scrolling buffer:"])

        #
        # Right side log window - the application log
        #
        t = self.add(npyscreen.FixedText, value = "Application Log:", relx=w, rely=h, color='LABEL') 
        t.editable = False

        self.appLog = self.add(npyscreen.BufferPager, maxlen=h, relx=w, rely=h+1, width=w-3) # , height=h-2, width=w-2)
        self.appLog.editable = False
        self.appLog._need_update = False

    def while_waiting(self): 
       
        if not self.printThread:
            self.printThread = threading.Thread(target=self.printWorker)
            self.printThread.daemon = True
            self.printThread.start()

        while not self.guiQueue.empty():
            obj = self.guiQueue.get()
            obj.call()

        for w in [self.commLog, self.appLog]:
            if w._need_update:
                w.update()
                w._need_update = False

    def printWorker(self):

        parser = argparse.ArgumentParser(description='%s, Direct Drive USB Print.' % sys.argv[0])
        parser.add_argument("-d", dest="device", action="store", type=str, help="Device to use, default: /dev/ttyACM0.", default="/dev/ttyACM0")
        parser.add_argument("-b", dest="baud", action="store", type=int, help="Baudrate, default 115200.", default=115200)
        parser.add_argument("-f", dest="file", action="store", type=str, help="Gcode to print")
        parser.add_argument("-F", dest="fakeendstop", action="store", type=bool, help="fake endstops", default=False)
        parser.add_argument("-t0", dest="t0", action="store", type=int, help="Temp 0 (heated bed), default comes from mat. profile.")
        parser.add_argument("-t1", dest="t1", action="store", type=int, help="Temp 1 (hotend 1), default comes from mat. profile.")
        parser.add_argument("-mat", dest="mat", action="store", help="Name of material profile to use [pla, abs...], default is pla.", default="pla")

        self.args = parser.parse_args()
        self.args.mode = "dlprint"

        # print "args: ", self.args

        (self.parser, self.planner, self.printer) = ddprint.initParser(self.args, gui=self)
        # util.commonInit(self.args, self.parser)
        self.printer.commandInit(self.args)

        self.mat_t0 = MatProfile.getBedTemp()
        self.mat_t1 = MatProfile.getHotendBaseTemp()

        # self.fn.set_value(self.args.file)
        self.guiQueue.put(SyncCall(self.fn.set_value, self.args.file))

        while True:
            # self.guiQueue.put("hi")
            # self.guiQueue.put(SyncCall(self.appLog.buffer, ["juhu"]))

            workDone = False

            while not self.cmdQueue.empty():
                obj = self.cmdQueue.get()
                obj.call()
                workDone = True

            if time.time() - self.lastUpdate > 2.5:
                status = self.printer.getStatus()
                self.guiQueue.put(SyncCall(self.updateStatus, status))

                workDone = True
                self.lastUpdate = time.time()

            if not workDone:
                time.sleep(0.1)

    def updateTemps(self, t0, t1):

        if t0 != None:
            self.curT0.set_value("%8.1f / %3.0f" % (t0, self.mat_t0))
            self.curT0.update()

        if t1 != None:
            self.curT1.set_value( "%8.1f / %3.0f" % (t1, self.mat_t1))
            self.curT1.update()

    def updateStatus(self, status):

        self.pState.set_value( "%8s" % self.stateNames[status["state"]])
        self.pState.update()
        self.updateTemps(status["t0"], status["t1"])
        self.swapSize.set_value( "%8s" % str(status["Swap"]))
        self.swapSize.update()
        self.sdrSize.set_value( "%8s" % str(status["SDReader"]))
        self.sdrSize.update()
        self.sbSisze.set_value( "%8s" % str(status["StepBuffer"]))
        self.sbSisze.update()
        self.underrun.set_value( "%8s" % str(status["StepBufUnderRuns"]))
        self.underrun.update()
        # self.display()

    def display(self, clear=False):
        npyscreen.Form.display(self, clear)
        self.curses_pad.vline( 1, self.columns/2-1, curses.ACS_VLINE, self.lines-2)
        self.curses_pad.hline( self.lines/2-1, 1, curses.ACS_HLINE, self.columns/2-2)
        self.curses_pad.hline( self.lines/2-1, self.columns/2, curses.ACS_HLINE, self.columns/2-1)

    def preheat(self):
        t = int(self.mat_t0 * 0.9)
        self._log( "\nPre-Heating bed (t0: %d)...\n" % t)
        self.printer.heatUp(HeaterBed, t)

        t = int(self.mat_t1 * 0.5)
        self._log( "\nPre-Heating extruder (t1: %d)...\n" % t)
        self.printer.heatUp(HeaterEx1, t)

    def preparePrintFile(self):
        self.errors.set_value("")
        self.cmdQueue.put(SyncCall(self.printFile))

    def printFile(self):

        self.parser.reset()
        self.planner.reset()

        util.home(self.parser, self.args.fakeendstop)
        self.printer.sendPrinterInit()

        # Send heat up  command
        self._log( "\nPre-Heating bed (t0: %d)...\n" % self.mat_t0)
        self.printer.heatUp(HeaterBed, self.mat_t0)

        t = int(self.mat_t1 * 0.5)
        self._log( "\nPre-Heating extruder (t1: %d)...\n" % t)
        self.printer.heatUp(HeaterEx1, t)

        # Send priming moves
        util.prime(self.parser)

        # Send printing moves
        f = self.parser.preParse(self.fn.get_value())

        lineNr = 0
        printStarted = False

        for line in f:
            self.parser.execute_line(line)

            #
            # Send more than one 512 byte block for dlprint
            #
            if lineNr > 1000 and (lineNr % 250) == 0:
                # check temp and start print

                if  not printStarted:

                    self._log( "\nHeating bed (t0: %d)...\n" % self.mat_t0 )
                    self.printer.heatUp(HeaterBed, self.mat_t0, wait=self.mat_t0)
                    self._log( "\nHeating extruder (t1: %d)...\n" % self.mat_t1 )
                    self.printer.heatUp(HeaterEx1, self.mat_t1, wait=self.mat_t1 - 10)

                    # Send print command
                    self.printer.sendCommandParam(CmdMove, p1=MoveTypeNormal, wantReply="ok")
                    printStarted = True

                else:

                    status = self.printer.getStatus()
                    self.guiQueue.put(SyncCall(self.updateStatus, status))
                    if status['state'] != StateStart:
                        break

            lineNr += 1

        self.parser.finishMoves()
        self.printer.sendCommand(CmdEOT, wantReply="ok")

        # XXX start print if less than 1000 lines or temp not yet reached:
        if not printStarted:

            self._log( "\nHeating bed (t0: %d)...\n" % self.mat_t0 )
            self.printer.heatUp(HeaterBed, self.mat_t0.t0, self.mat_t0)
            self._log( "\nHeating extruder (t1: %d)...\n" % self.mat_t1 )
            self.printer.heatUp(HeaterEx1, self.mat_t1, wait=self.mat_t1 - 10)

            # Send print command
            self.printer.sendCommandParam(CmdMove, p1=MoveTypeNormal, wantReply="ok")

        status = self.printer.getStatus()
        self.guiQueue.put(SyncCall(self.updateStatus, status))
        while status['state'] != StateInit:
            time.sleep(2)
            status = self.printer.getStatus()
            self.guiQueue.put(SyncCall(self.updateStatus, status))

        self.printer.coolDown(HeaterEx1)
        self.printer.coolDown(HeaterBed)

        util.home(self.parser, self.args.fakeendstop)

        self.printer.sendCommand(CmdDisableSteppers, wantReply="ok")

    def stringFromArgs(self, *args):
        r = ""
        for a in args:
            if type(a) == types.StringType:
                r += a
            else:
                r += str(a)
        return r

    # Non-thread save version
    def _log(self, *args):
        self.appLog.buffer([self.stringFromArgs(*args)])
        self.appLog._need_update = True

    def _logError(self, err):
        # xxx manage error-log here ...
        self.errors.set_value( err )
        self.errors.update()

    # Thread save version
    def log(self, *args):
        # self.appLog.buffer([self.stringFromArgs(*args)])
        self.guiQueue.put(SyncCallUpdate(self.appLog.buffer, [self.stringFromArgs(*args)]))

    def logError(self, *args):
        # xxx special error handling
        self.guiQueue.put(SyncCall(self._logError, self.stringFromArgs(*args)))

    def logSend(self, *args):
        # self.commLog.buffer([self.stringFromArgs(*args)])
        self.guiQueue.put(SyncCallUpdate(self.commLog.buffer, [self.stringFromArgs(*args)]))

    def logRecv(self, *args):
        # self.commLog.buffer(["    " + self.stringFromArgs(*args) ])
        self.guiQueue.put(SyncCallUpdate(self.commLog.buffer, ["    " + self.stringFromArgs(*args)]))

    # Stdout redirection 
    def write(self, s):
        # self.appLog.buffer(["STDOUT:" + s])
        self.guiQueue.put(SyncCallUpdate(self.appLog.buffer, ["STDOUT:" + s.strip()]))

    def tempCb(self, t0=None, t1=None):
        self.guiQueue.put(SyncCall(self.updateTemps, t0, t1))

# class TestApp(npyscreen.NPSApp): 
# class TestApp(npyscreen.NPSAppManaged): 
class TestApp(npyscreen.NPSAppManaged):

    def main(self): 

        F  = MainForm(name = "DDPrint UI")

        # Stdout redirection, xxx can be removed if all print's are replaced by log() calls...
        sys.stdout = F

        # Run eventloop, blocking call.
        F.edit() 

if __name__ == "__main__": 

    App = TestApp() 
    App.run() 








