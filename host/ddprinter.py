# -*- coding: utf-8 -*-
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

# # This needs pyserial version >= 2.6:
# try:
    # from serial.tools import list_ports
# except ImportError:
    # print "\nWARNING your python-serial version seems to be to old!\n"

#
# Note: pyserial 2.6.1 seems to have a bug with reconnect (read only garbage 
# at second connect).
# So i've mixed pyserial 2.5.x with the list_ports functions from 2.6.x
#
import time, struct, crc16
import list_ports
import dddumbui

from serial import Serial, SerialException, SerialTimeoutException
from ddprintcommands import *
from ddprintstates import *

############################################################################
#
# Constants
#
SOH = 0x81
############################################################################

class SERIALDISCON(SerialException):
    pass

class FatalPrinterError(Exception):
    def __init__(self, msg):
        self.msg = msg

    def __str__(self):
        return "FatalPrinterError: " + self.msg

class Printer(Serial):

    __single = None 

    # endStoreToken = "Done saving"
    # Number of rx errors till we assume the
    # line is dead.
    maxRXErrors = 10
    maxTXErrors = 10

    def __init__(self, gui=None):

        if Printer.__single:
            raise RuntimeError('A Printer already exists')

        Printer.__single = self

        if gui:
            self.gui = gui
        else:
            self.gui = dddumbui.DumbGui()

        Serial.__init__(self)

        self.usbId = None

        self.lastSend = 0

        self.gcodeData = None
        # self.gcodePos = 0

        self.lineNr = 0
        # The last (max. 256) commands sent, for command re-send
        self.lastCommands = {}

        # Retry counter on rx and tx errors
        self.rxErrors = 0
        self.txErrors = 0

        # Timespan where we monitor the serial line after the
        # print has finished.
        self.postMonitor = 0

        self.startTime = None

        self.wantReply = None
        self.wantAck = None
        # Part of a response read from printer
        self.recvPart = ""

        self.curDirBits = 0

    @classmethod
    def get(cls):
        return cls.__single

    # Check a printer response for an error
    def checkError(self, recvLine):

        if "Error:" in recvLine and  "Last Line" in recvLine:

            self.gui.log("Error:", recvLine)
            self.gui.logRecv("Error:", recvLine)

            # Error:Line Number is not Last Line Number+1, Last Line: 9            
            # Error:checksum mismatch, Last Line: 71388
            lastLine = int(recvLine[recvLine.index("Error"):].split(":")[2])
            time.sleep(0.1)

            if lastLine == (self.lineNr % 256):
                self.gui.log("Command was sent ok, faking ACK...:", lastLine)
                # Das ursprünglich gesendete kommando ging zwar duch, danach 
                # gab es jedoch einen komm. fehler, so dass das ACK nicht mehr
                # empfangen werden konnte. Desswegen kein resend notwendig, und es
                # kann so getan werden, als ob ein ACK empfangen wurde.
                return True

            # self.gui.log("Scheduling resend of command:", lastLine+1)
            self.gui.log("Scheduling resend of command:", lastLine)

            # assert(self.gcodePos == lastLine + 2)
            # self.gcodePos = lastLine + 1

            # Wait 0.1 sec, give firmware time to drain buffers
            return self.lastCommands[lastLine]

        for token in ["Error:", "cold extrusion", "SD init fail", "open failed"]:
            if token in recvLine:

                self.gui.logError("ERROR: reply from printer: '%s'" % recvLine)

                self.readMore(20)

                # self.reset()
                if "UNKNOWN COMMAND" in recvLine.upper():
                    raise FatalPrinterError(recvLine)

    # Read a response from printer, "handle" exceptions
    def safeReadline(self):

        result = ""

        while True:

            try:
                c = self.read()
                # self.gui.log("c: ", c)
            except SerialException as ex:
                self.gui.log("Readline() Exception raised:", ex)

                self.rxErrors += 1

                if self.rxErrors >= Printer.maxRXErrors:
                    self.gui.log("declare line is dead ...")
                    raise SERIALDISCON

                time.sleep(0.1)
                continue

            # Receive without error, reset error counter
            self.rxErrors = 0

            if not c:
                return result

            result += c

            if c == "\n" or ord(c) == 0x6:
                # result += "\n"
                return result

        # not reached

    # Monitor printer responses for a while (wait waitcount * 0.1 seconds)
    def readMore(self, waitcount=100):

        self.gui.log("waiting %.2f seconds for more messages..." % (waitcount/20.0))

        for i in range(waitcount):

            try:
                recvLine = self.safeReadline()        
            except SERIALDISCON:
                self.gui.logError("Line disconnected in readMore(), reconnecting!")
                self.reconnect()
                time.sleep(0.1)
                continue

            if recvLine:
                if ord(recvLine[0]) > 20:
                    self.gui.logRecv("Reply: ", recvLine,)
                else:
                    self.gui.logRecv("Reply: 0x%s" % recvLine.encode("hex"))

    def initSerial(self, device, br=250000):
        self.port = device
        self.baudrate = br
        self.timeout = 0.05
        self.writeTimeout = 10
        self.open()

        # Store usb information for later re-connection even if device
        # name has changed:
        comports = list_ports.comports()

        # ('/dev/ttyACM0', 'ttyACM0', 'USB VID:PID=2341:0042 SNR=75237333536351815111')]
        for (dev, name, usbid) in comports:
            if dev == device or name == device:
                self.gui.log("Found usbid %s for device %s" % (usbid, dev))
                self.usbId = usbid
                break

        # Read left over garbage
        recvLine = self.safeReadline()        
        while recvLine:
            self.gui.logRecv("Initial read: ", recvLine)
            self.gui.logRecv(recvLine.encode("hex"), "\n")
            recvLine = self.safeReadline()        

    def commandInit(self, args):
        self.initSerial(args.device, args.baud)
        self.resetLineNumber()

    def resetLineNumber(self):
        self.lineNr = 0
        self.sendCommand(CmdResetLineNr)
    
    def sendPrinterInit(self):
        self.curdirbits = 0
        self.sendCommand(CmdPrinterInit, wantReply="ok")

    # Send a command to the printer, add a newline if 
    # needed.
    def send(self, cmd):

        # assert(len(cmd) <= 521)

        while True:

            self.gui.logSend("\nSending %d bytes: " % len(cmd), cmd[:10].encode("hex"))
            # self.gui.log("serial: ", self.lineNr % 256, ord(cmd[1]))
            try:
                self.write(cmd)
            except SerialTimeoutException:
                self.gui.log("Tryed sending %d bytes: " % len(cmd), cmd[:10].encode("hex"))
                self.gui.log("SerialTimeoutException on send!!!")
                pass
            except SerialException as ex:
                self.gui.log("send() Exception raised:", ex)

                self.txErrors += 1

                if self.txErrors >= Printer.maxTXErrors:
                    self.gui.log("declare line is dead ...")
                    # raise SERIALDISCON
                    self.gui.logError("Line disconnected in send(). Trying reconnect!")
                    self.reconnect()

                time.sleep(0.1)
                continue

            # Sent something, reset error counter
            self.txErrors = 0
            return

        # not reached


    def _Fletcher16(self, data):
        sum1 = 0
        sum2 = 0
                          
        for c in data:
            sum1 = (sum1 + ord(c)) % 255
            sum2 = (sum2 + sum1) % 255

        return (sum2 << 8) | sum1
       
    def buildBinaryCommand(self, binCmd, binPayload=None, lineNr=None):

        binary  = struct.pack("<B", SOH)
        binary += struct.pack("<B", self.lineNr % 256)
        binary += binCmd

        if binPayload:

            payloadSize = lenPayload = len(binPayload)

            lenFmt = "<H"
            if ord(binCmd) != CmdBlock:

                lenFmt = "<I"
                lenPayload = min(payloadSize, 256)

            binary += struct.pack(lenFmt, payloadSize)
            binary += binPayload[:lenPayload]
        else:
            binary += struct.pack("<I", 0);

        # checkSum = self.Fletcher16(binary)
        checkSum = crc16.crc16xmodem(binary)

        # self.gui.log("checkSum: ", checkSum, "0x%x" % checkSum)
        binary += struct.pack("<H", checkSum)

        # Store for later possible command resend
        self.lastCommands[self.lineNr % 256] = binary

        if lineNr != None:
            self.lineNr = lineNr
        else:
            self.lineNr += 1

        return binary

    def sendBinaryCommand(self, binCmd, wantReply=None, binPayload=None, lineNr=None):
        self.gui.logSend("sendCommand: ", CommandNames[ord(binCmd)])
        binary = self.buildBinaryCommand(binCmd, binPayload, lineNr)
        self.send2(binary, wantReply)

    def sendCommand(self, cmd, wantReply=None, binPayload=None, lineNr=None):
        self.sendBinaryCommand(struct.pack("<B", cmd), wantReply=wantReply, binPayload=binPayload, lineNr=lineNr)

    def sendCommandParam(self, cmd, wantReply=None, lineNr=None, p1=None, p2=None, p3=None, p4=None):
        self.sendCommandParamV(cmd, wantReply=wantReply, lineNr=lineNr, params=(p1, p2, p3, p4))

    def sendCommandParamV(self, cmd, params, wantReply=None, lineNr=None):
        assert(params[0] != None)
        payload = ""
        for p in params:
            if p:
                payload += p.pack()
        self.sendCommand(cmd, wantReply=wantReply, binPayload=payload, lineNr=lineNr)

    def reconnect(self):

        # XXX add timeout, or otherwise prevent re-connection to power-cycled printer?!

        comports = list_ports.comports()

        # ('/dev/ttyACM0', 'ttyACM0', 'USB VID:PID=2341:0042 SNR=75237333536351815111')]
        for (dev, name, usbid) in comports:
            if usbid == self.usbId:
                self.gui.log("reconnect(): found device %s, previous device: %s" % (dev, self.port))
                self.close()

                try:
                    self.initSerial(dev, br=self.baudrate)
                except SerialException as ex:
                    self.gui.log("reconnect() Exception raised:", ex)
                    time.sleep(1)
                    self.reconnect()

                return

        time.sleep(0.1)

    # Send command or Query info from printer
    def send2(self, binary, wantReply=None):

        # self.startTime = time.time()

        wantAck = True

        recvPart = None

        # Send cmd/query
        startTime = time.time()

        """
        # Simulate random checksum error
        if random.randint(0, 100) < 2:
            tmp = binary[:-1] + "0"
            assert(len(tmp) == len(binary))
            self.send(tmp)
        else:
            self.send(binary)
        """

        self.send(binary)

        # Wait for response, xxx without timeout/retry for now
        while True:

            try:
                recvLine = self.safeReadline()        
            except SERIALDISCON:
                self.gui.logError("Line disconnected in send2(), reconnecting!")
                self.reconnect()

                startTime = time.time()
                self.send(binary)
                continue

            if not recvLine:
                continue

            if wantAck and recvLine == chr(0x6):
                dt = time.time() - startTime
                n = len(binary)
                self.gui.logRecv("ACK, Sent %d bytes in %.2f ms, %.2f Kb/s" % (n, dt*1000, n/(dt*1000)))

                if not wantReply:
                    return True

                wantAck = False
                continue

            if recvPart:
                recvLine = recvPart + recvLine
                recvPart = None

            if recvLine[-1] != "\n":
                recvPart = recvLine
                continue

            resendCommand = self.checkError(recvLine)
            if resendCommand == True:
                # Command ok without ack
                if wantReply:
                    self.gui.log("Warning: skipping wantReply:", wantReply)
                return True
            elif resendCommand:
                # command resend
                startTime = time.time()
                self.send(resendCommand)
                continue

            if wantReply and recvLine.startswith(wantReply):
                self.gui.logRecv("Got Required reply: ", recvLine,)
                assert(not wantAck)
                return recvLine
            else:
                self.gui.logRecv("Reply: ", recvLine,)

            if (time.time() - startTime) > 2:
                # Timeout firmware response
                self.gui.log("Error send Timeout, resending.")
                startTime = time.time()
                self.send(binary)
                continue

        # Notreached

    # Query info from printer
    def query(self, cmd, binPayload=None):

        self.gui.logSend("query: ", CommandNames[cmd])
        binary = self.buildBinaryCommand(struct.pack("<B", cmd), binPayload=binPayload);

        reply = self.send2(binary, "Res:")

        if reply == True:
            # Command was ok, but no response due to reconnect, restart query:
            return self.query(cmd, binPayload)

        return eval(reply[4:])

    def getStatus(self):
        valueNames = ["state", "t0", "t1", "Swap", "SDReader", "StepBuffer", "StepBufUnderRuns", "targetT1"]
        statusList = self.query(CmdGetStatus)

        statusDict = {}
        for i in range(len(valueNames)):
            statusDict[valueNames[i]] = statusList[i]

        return statusDict

    def waitForState(self, destState, wait=1):

        status = self.getStatus()
        while status['state'] != destState:
            time.sleep(wait)
            status = self.getStatus()

    def stateMoving(self, status=None):

        if not status:
            status = self.getStatus()

        return status['state'] == StateStart or status['state'] == StateDwell

    def getTemps(self):
        temps = self.query(CmdGetCurrentTemps)
        targetTemps = self.query(CmdGetTargetTemps)
        self.gui.tempCb(temps[0], temps[1], targetTemps[1])
        return temps

    ####################################################################################################

    def heatUp(self, heater, temp, wait=None):

        payload = struct.pack("<BH", heater, temp) # Parameters: heater, temp
        self.sendCommand(CmdSetTargetTemp, binPayload=payload, wantReply="ok")

        while wait !=  None:
            time.sleep(2)
            temps = self.getTemps()

            if temps[heater] >= wait:
                break

    ####################################################################################################

    def coolDown(self, heater, temp=0, wait=None):

        payload = struct.pack("<BH", heater, temp)
        self.sendCommand(CmdSetTargetTemp, binPayload=payload, wantReply="ok")

        while wait !=  None:
            time.sleep(2)
            temps = self.getTemps()

            if temps[heater] <= wait:
                break



