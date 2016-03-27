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
import dddumbui, cobs

from serial import Serial, SerialException, SerialTimeoutException
from ddprintcommands import *
from ddprintstates import *

############################################################################
#
# Constants
#
# SOH = 0x81
SOH = 0x0 # COBs
############################################################################

class SERIALDISCON(SerialException):
    pass

class FatalPrinterError(Exception):
    def __init__(self, msg):
        self.msg = msg

    def __str__(self):
        return "FatalPrinterError: " + self.msg

class RxTimeout(Exception):
    def __init__(self, msg=""):
        self.msg = msg

    def __str__(self):
        return "RxTimeout: " + self.msg

class RxChecksumError(Exception):
    def __init__(self, msg=""):
        self.msg = msg

    def __str__(self):
        return "RxChecksumError: " + self.msg

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

        # Command sequence number [1..255]
        self.lineNr = 1

        # The last (max. 256) commands sent, for command re-send
        self.lastCommands = {}

        # Retry counter on rx and tx errors
        self.rxErrors = 0
        self.txErrors = 0

        self.startTime = None

        self.curDirBits = 0

    @classmethod
    def get(cls):
        return cls.__single

    def commandResend(self, lastLine):

        self.gui.log("Command resend:", lastLine)

        # Wait 0.1 sec, give firmware time to drain buffers
        time.sleep(0.1)

        if lastLine == self.lineNr:
            self.gui.log("Command was sent ok, faking ACK...:", lastLine)
            # Das ursprünglich gesendete kommando ging zwar duch, danach 
            # gab es jedoch einen komm. fehler, so dass das ACK nicht mehr
            # empfangen werden konnte. Desswegen kein resend notwendig, und es
            # kann so getan werden, als ob ein ACK empfangen wurde.
            return ResendWasOK

        self.gui.log("Scheduling resend of command:", lastLine)
        return self.lastCommands[lastLine]

    # Check a printer response for an error
    def checkErrorResponse(self, respCode, length, payload, handleError=True):

        if respCode == RespUnknownCommand:

            self.gui.logError("ERROR: RespUnknownCommand '0x%x'" % ord(payload))
            if handleError:
                raise FatalPrinterError(ResponseNames[respCode])

        elif respCode == RespKilled:

            reason = ord(payload[0])
            if reason in [RespHardwareEndstop, RespSoftwareEndstop]:

                (x, y, z, xtrig, ytrig, ztrig) = struct.unpack("<iiiBBB", payload[1:])
                self.gui.logError("ERROR: PRINTER KILLED! Reason: %s, X: %d, Y: %d, Z: %d, trigger: x: %d, y: %d, z: %d" % (RespCodeNames[reason], x, y, z, xtrig, ytrig, ztrig))

            elif reason == RespUnknownBCommand:

                self.gui.logError("ERROR: PRINTER KILLED! Reason: %s, command: 0x%x" % (RespCodeNames[reason], ord(payload[1])))

            elif reason == RespAssertion:

                # Useless python struct 'p' format...
                (line, slen) = struct.unpack("<HB", payload[1:4])
                filename = payload[4:4+slen]
                self.gui.logError("ERROR: PRINTER KILLED! Reason: %s, Line: %d, File: %s" % (RespCodeNames[reason], line, filename))

            else:

                self.gui.logError("ERROR: PRINTER KILLED! Reason: %s" % RespCodeNames[reason])

            if handleError:
                raise FatalPrinterError(ResponseNames[respCode])

        elif respCode == RespRXError:

            (lastLine, flags) = struct.unpack("<BB", payload)

            flagstr = ""
            if flags & 0x10:
                flagstr += "FrameError "
            if flags & 0x8:
                flagstr += "Overrun  "
            if flags & 0x4:
                flagstr += "Parity  "

            # flags: M_UCSRxA & 0x1C;
            self.gui.logRecv("RespRXError: LastLine %d, flags: 0x%x (%s)" % (lastLine, flags, flagstr))
            if handleError:
                return self.commandResend(lastLine)

        elif respCode == RespRXCRCError:

            lastLine = ord(payload)
            self.gui.logRecv("RespRXCRCError:", lastLine)
            if handleError:
                return self.commandResend(lastLine)

        elif respCode == RespSerNumberError:

            lastLine = ord(payload)
            self.gui.logRecv("RespSerNumberError:", lastLine)
            if handleError:
                return self.commandResend(lastLine)

        elif respCode == RespRXTimeoutError:

            lastLine = ord(payload)
            self.gui.logRecv("RespRXTimeoutError:", lastLine)
            if handleError:
                return self.commandResend(lastLine)

        elif respCode < 128: # Range of errorcodes

            self.gui.logError("ERROR: unknown response code '0x%x'" % respCode)
            if handleError:
                raise FatalPrinterError(ResponseNames[respCode])

    # Read a response from printer, "handle" exceptions
    def safeReadline(self):

        result = ""

        while True:

            try:
                c = self.read()
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

    def readWithTimeout(self, length):

        res = ""
        timeout = 0
        while len(res) < length:

            try:
                c = self.read(length - len(res))
            except SerialException as ex:
                self.gui.log("Readline() Exception raised:", ex)

                self.rxErrors += 1

                if self.rxErrors >= Printer.maxRXErrors:
                    self.gui.log("declare line is dead ...")
                    raise SERIALDISCON

                time.sleep(0.1)
                continue

            # Receive without error, reset error counter
            if c:
                self.rxErrors = 0
                res += c

            timeout += self.timeout

            if timeout > 3:
                print "timeout reading, data read: %d bytes, '%s'" % (len(res), res)
                raise RxTimeout()

        # for c in res:
            # self.gui.log("c: 0x%x" % ord(c))
        return res

    def readResponse(self):

        s = self.readWithTimeout(1) # SOH
# xxx loop til 0x0 found
        assert(s == cobs.nullByte)

        rc = self.readWithTimeout(1) # read response code
        crc = crc16.crc16xmodem(rc)
        cmd = ord(rc)

        l = self.readWithTimeout(1) # read length byte
        crc = crc16.crc16xmodem(l, crc)

        length = ord(l) - 1
        self.gui.logRecv("Response 0x%x, reading %d b" % (cmd, length))

        payload = ""
        if length:
            payload = self.readWithTimeout(length) # read payload
            crc = crc16.crc16xmodem(payload, crc)

        (cflags, checkSum) = struct.unpack("<BH", self.readWithTimeout(3))

        if cflags == 0x2:
            checkSum -= 0x100;
        elif cflags == 0x3:
            checkSum -= 0x01;
        elif cflags == 0x4:
            checkSum -= 0x101;

        if checkSum != crc:
            print "RxChecksumError our: 0x%x, fw: 0x%x, payload: %s, cflags: 0x%x" % (crc, checkSum, (rc + l + payload).encode("hex"), cflags)
            """
            # Drain input
            try:
                s = self.read()
                print "RxChecksumError, drain input: %s" % s.encode("hex")
            except SerialException:
                pass

            while s:
                try:
                    s = self.read()
                    print "drain input: %s" % s.encode("hex")
                except SerialException:
                    pass
            """
            raise RxChecksumError()

        # Decode COBS encode payload
        payload = cobs.decodeCobs(payload)
        # Todo: remove length from result tuple, its not useful
        return (cmd, length, payload)

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
            self.gui.logRecv("Initial read: '%s'" % recvLine)
            self.gui.logRecv(recvLine.encode("hex"), "\n")
            recvLine = self.safeReadline()        

        print "Bootloader wait..."
        time.sleep(1)

    def commandInit(self, args):
        self.initSerial(args.device, args.baud)
        self.resetLineNumber()

    def resetLineNumber(self):
        self.lineNr = 1
        self.sendCommand(CmdResetLineNr)
   
    def incLineNumber(self):
        self.lineNr += 1
        if self.lineNr == 256:
            self.lineNr = 1

    def sendPrinterInit(self):
        self.curdirbits = 0
        self.sendCommand(CmdPrinterInit)

    # Send a command to the printer, add a newline if 
    # needed.
    def send(self, cmd):

        # assert(len(cmd) <= 521)

        while True:

            try:
                self.write(cmd)
            except SerialTimeoutException:
                self.gui.log("Tryed sending %d bytes: " % len(cmd), cmd[:10].encode("hex"))
                self.gui.log("SerialTimeoutException on send!!!")
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

    def buildBinaryCommand(self, binCmd, binPayload=None):

        payloadSize = 0
        if binPayload:
            payloadSize = len(binPayload)
            # print "Send payload: %s" % binPayload.encode("hex")
            # print "Send payload: ", payloadSize, cobs.LenCobs
            assert(payloadSize <= cobs.LenCobs)

        #
        # Not block: SOH(1) line(1) cmd(1) size(4) [ data ] chksum(2) -> max payload = 254 (COBS) - 9 = 245
        # Block:     SOH(1) line(1) cmd(1) size(2) [ data ] chksum(2)
        #
        binary  = struct.pack("<B", SOH)

        header = struct.pack("<BBB", self.lineNr, binCmd, payloadSize+0x01)

        binary += header
        checksum = crc16.crc16xmodem(header)

        if binPayload:
            binary += binPayload
            checksum = crc16.crc16xmodem(binPayload, checksum)

        # print "cflags, c1: 0x%x, 0x%x" % (cflags, checksum)
        cflags = 0x1
        if checksum == 0:
            cflags = 0x4
            checksum = 0x0101
        elif (checksum & 0xFF00) == 0:
            cflags = 0x2
            checksum = 0x0100 | (checksum & 0xFf)
        elif (checksum & 0x00FF) == 0:
            cflags = 0x3
            checksum = 0x0001 | (checksum & 0xFF00)

        # self.gui.log("checkSum: ", checkSum, "0x%x" % checkSum)
        binary += struct.pack("<BH", cflags, checksum)

        # Store for later possible command resend
        self.lastCommands[self.lineNr] = binary

        self.incLineNumber()

        return binary

    # Use qury() if you need the result of the command
    def sendCommand(self, cmd, binPayload=None):

        cobsBlock = binPayload and cobs.encodeCobsString(binPayload)
        self.sendCommandC(cmd, cobsBlock)

    # Use qury() if you need the result of the command
    def sendCommandC(self, cmd, cobsBlock):

        binary = self.buildBinaryCommand(cmd, cobsBlock)
        self.send2(cmd, binary)

    # Use qury() if you need the result of the command
    def sendCommandParamV(self, cmd, params):

        assert(params[0] != None)
        payload = ""
        for p in params:
            if p:
                payload += p.pack()
        self.sendCommand(cmd, binPayload=payload)

    # Query info from printer, use this to receive returned data, use [sendCommand, sendCommandParamV] for 
    # 'write only' commands.
    # Note: Commands must be 'restartable' without sideeffects (see commandResend(), ResendWasOK)
    def query(self, cmd, binPayload=None, doLog=True):

        if doLog:
            self.gui.logSend("query: ", CommandNames[cmd])

        cobsBlock = binPayload and cobs.encodeCobsString(binPayload)
        binary = self.buildBinaryCommand(cmd, cobsBlock)

        reply = self.send2(cmd, binary)

        if reply == ResendWasOK:
            # Command was ok, but no response due to reconnect, restart query:
            return self.query(cmd, binPayload)

        return reply

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
    def send2(self, sendCmd, binary):

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

        self.gui.logSend("*** sendCommand %s (0x%x, len: %d) *** " % (CommandNames[sendCmd], sendCmd, len(binary)))
        # print "send: ", binary.encode("hex")
        self.send(binary)

        # print "Waiting for reply code 0x%x (or ack)" % sendCmd

        # Wait for response, xxx without timeout/retry for now
        while True:

            try:
                # recvLine = self.safeReadline()        
                (cmd, length, payload) = self.readResponse()        
            except SERIALDISCON:
                self.gui.logError("Line disconnected in send2(), reconnecting!")
                self.reconnect()

                startTime = time.time()
                self.send(binary)
                continue

            except RxTimeout:
                self.gui.logError("RxTimeout, resending command!")
                startTime = time.time()
                self.send(binary)
                continue

            except RxChecksumError:
                self.gui.logError("RxChecksumError, resending command!")
                startTime = time.time()
                self.send(binary)
                continue

            n = len(binary)
            dt = time.time() - startTime
            if cmd == 0x6:
                self.gui.logRecv("ACK, Sent %d bytes in %.2f ms, %.2f Kb/s" % (n, dt*1000, n/(dt*1000)))
                return (cmd, length, payload)

            # if cmd == RespGenericString:
                # print "Got Generic String: '%s'" % payload
                # continue

            resendCommand = self.checkErrorResponse(cmd, length, payload)

            if resendCommand == ResendWasOK:

                # Command ok without ack
                return ResendWasOK

            elif resendCommand:

                # command resend
                startTime = time.time()
                self.send(resendCommand)
                continue

            self.gui.logRecv("ACK (0x%x/0x%x), Sent %d bytes in %.2f ms, %.2f Kb/s" % (cmd, sendCmd, n, dt*1000, n/(dt*1000)))

            if cmd == sendCmd:
                # print "got reply:", payload
                return (cmd, length, payload)

            print "unknown reply:", cmd, length, payload
            assert(0)

        # Notreached

    def getStatus(self):

        valueNames = ["state", "t0", "t1", "Swap", "SDReader", "StepBuffer", "StepBufUnderRuns", "targetT1"]

        (cmd, length, payload) = self.query(CmdGetStatus, doLog=False)

        tup = struct.unpack("<BffIHHHH", payload)

        statusDict = {}
        for i in range(len(valueNames)):
            # statusDict[valueNames[i]] = statusList[i]
            statusDict[valueNames[i]] = tup[i]

        return statusDict

    def getAddHomeing(self):

        (cmd, length, payload) = self.query(CmdGetEepromSettings, doLog=False)
        tup = struct.unpack("<ffff", payload)
        return tup

    def waitForState(self, destState, wait=1):

        status = self.getStatus()

        while status['state'] != destState:
            time.sleep(wait)
            # try:
            status = self.getStatus()
            # except RxTimeout:
                # print "GetStatus raised RxTimeout error!"

    def stateMoving(self, status=None):

        if not status:
            status = self.getStatus()

        return status['state'] == StateStart or status['state'] == StateDwell

    def getTemps(self, ):
        (cmd, length, payload) = self.query(CmdGetCurrentTemps)
        if len(payload) == 8:
            temps = struct.unpack("<ff", payload)
        else:
            temps = struct.unpack("<fff", payload)

        (cmd, length, payload) = self.query(CmdGetTargetTemps)
        if length == 3:
            targetTemps = struct.unpack("<BH", payload)
        else:
            targetTemps = struct.unpack("<BHH", payload)
        self.gui.tempCb(temps[0], temps[1], targetTemps[1])
        return temps

    ####################################################################################################

    def heatUp(self, heater, temp, wait=None):

        payload = struct.pack("<BH", heater, temp) # Parameters: heater, temp
        self.sendCommand(CmdSetTargetTemp, binPayload=payload)

        while wait !=  None:
            time.sleep(2)
            temps = self.getTemps()

            if temps[heater] >= wait:
                break

    ####################################################################################################

    def coolDown(self, heater, temp=0, wait=None):

        payload = struct.pack("<BH", heater, temp)
        self.sendCommand(CmdSetTargetTemp, binPayload=payload)

        while wait !=  None:
            time.sleep(2)
            temps = self.getTemps()

            if temps[heater] <= wait:
                break

    ####################################################################################################

    def isHomed(self):

        (cmd, length, payload) = self.query(CmdGetHomed)
        tup = struct.unpack("<BBB", payload)
        return tup == (1, 1, 1)

    ####################################################################################################

    def getPos(self):

        (cmd, length, payload) = self.query(CmdGetPos)
        tup = struct.unpack("<iiii", payload)
        return tup

    def getEndstops(self):

        # Check, if enstop was pressed
        (cmd, length, payload) = self.query(CmdGetEndstops)
        tup = struct.unpack("<BiBiBi", payload)
        return tup

    ####################################################################################################

    def endStopTriggered(self, dim, fakeHomingEndstops=False):

        # Check, if enstop was pressed
        tup = self.getEndstops()

        if tup[dim*2] or fakeHomingEndstops:
            print "Endstop %d hit at position: %d" % (dim, tup[dim*2+1])
            return True

        print "Endstop %d open at position: %d" % (dim, tup[dim*2+1])
        return False







