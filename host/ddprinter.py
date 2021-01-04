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

#
# Note: pyserial 2.6.1 seems to have a bug with reconnect (read only garbage 
# at second connect).
#
import time, struct, crc_ccitt_kermit, termios, pprint, sys
import dddumbui, cobs, ddprintutil as util, types

from serial import Serial, SerialException, SerialTimeoutException
from ddconfig import debugComm
from ddprofile import PrinterProfile
from ddprintcommands import *
from ddprintconstants import *
from ddprintstates import *

############################################################################
#
# Constants
#
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

    _single = None 

    # Number of rx errors till we assume the
    # line is dead.
    # maxRXErrors = 10
    maxRXErrors = 3
    # maxTXErrors = 10
    maxTXErrors = 3

    def __init__(self, gui=None):

        if Printer._single:
            raise RuntimeError('Error: a Printer instance already exists')

        Printer._single = self

        if gui:
            self.gui = gui
        else:
            self.gui = dddumbui.DumbGui()

        Serial.__init__(self)

        # Command sequence number [1..255]
        self.lineNr = 1

        # The last (max. 256) commands sent, for command re-send
        self.lastCommands = {}

        # Retry counter on rx and tx errors
        self.rxErrors = 0
        self.txErrors = 0

        self.printStartedAt = None
        self.printEndedAt = None

        self.printerProfile = None

        # xxx debug
        self.commandInitDone = False

    @classmethod
    def get(cls):

        if not cls._single:
            raise RuntimeError('Printer instance not created, yet')

        return cls._single

    def commandResend(self, lastLine):

        self.gui.log("Command resend, self.lineNr: %d, lastLine: %d" % (self.lineNr, lastLine))

        # Wait 0.1 sec, give firmware time to drain buffers
        time.sleep(0.1)

        if self.lineNr == lastLine:

            self.gui.log("Command was sent ok, faking ACK...:", lastLine)
            # Das ursprünglich gesendete kommando ging zwar duch, danach 
            # gab es jedoch einen komm. fehler, so dass das ACK nicht mehr
            # empfangen werden konnte. Desswegen kein resend notwendig, und es
            # kann so getan werden, als ob ein ACK empfangen wurde.
            return ResendWasOK
        # elif self.lineNr > lastLine:
        # elif self.lineNr < lastLine:

        self.gui.log("Scheduling resend of command:", lastLine)
        return self.lastCommands[lastLine]

    # Check a printer response for an error
    def checkErrorResponse(self, respCode, payload, handleError=True):

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

            elif reason in [RespSDReadError, RespSDWriteError]:

                (errorCode, spiStatus) = struct.unpack("<BB", payload[1:])
                self.gui.logError("ERROR: PRINTER KILLED! Reason: %s, ErrorCode: %d (0x%x), spiStatus: 0x%x" % (RespCodeNames[reason], errorCode, errorCode, spiStatus))

            elif reason == RespMinTemp:

                (heater,) = struct.unpack("<B", payload[1])
                self.gui.logError("ERROR: PRINTER KILLED! Reason: %s, Heater: %d" % (RespCodeNames[reason], heater))

            elif reason == RespMaxTemp:

                (heater,) = struct.unpack("<B", payload[1])
                self.gui.logError("ERROR: PRINTER KILLED! Reason: %s, Heater: %d" % (RespCodeNames[reason], heater))

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
            self.gui.logComm("RespRXError: LastLine %d, flags: 0x%x (%s)" % (lastLine, flags, flagstr))
            if handleError:
                return self.commandResend(lastLine)

        elif respCode == RespRXCRCError:

            lastLine = ord(payload)
            self.gui.logComm("RespRXCRCError:", lastLine)
            if handleError:
                return self.commandResend(lastLine)

        elif respCode == RespSerNumberError:

            lastLine = ord(payload)
            self.gui.logComm("RespSerNumberError:", lastLine)
            if handleError:
                return self.commandResend(lastLine)

        elif respCode == RespRXTimeoutError:

            lastLine = ord(payload)
            # self.gui.logComm("RespRXTimeoutError:", lastLine)
            if handleError:
                return self.commandResend(lastLine)

        elif respCode == RespUnderrun:

            (p1, p2, p3, p4, p5) = struct.unpack("<IIIII", payload)
            self.gui.logError("ERROR: lastsize: %d, lastsize2: %d, minTimer: %d, swap: %d, sdreader: %d" % (p1, p2, p3, p4, p5))
            if handleError:
                raise FatalPrinterError(ResponseNames[respCode])

        elif respCode < 128: # Range of errorcodes

            self.gui.logError("ERROR: unknown response code '0x%x'" % respCode)
            if handleError:
                raise FatalPrinterError(ResponseNames[respCode])

    # Check a printer response for an error
    def handleUnsolicitedMsg(self, respCode, payload):

        if respCode == RespUnsolicitedMsg:

            msgType = ord(payload[0])
            self.gui.logComm("RespUnsolicitedMsg:", msgType)

            try:

                if msgType == ExtrusionLimitDbg:
                    (tempIndex, actSpeed, targetSpeed, grip) = struct.unpack("<hhhh", payload[1:])
                    self.gui.logComm("Limit extrusion index: %d, act: %d, target: %d, grip: %d" % (tempIndex, actSpeed, targetSpeed, grip))
                elif msgType == PidDebug:
                    (pid_dt, pTerm, iTerm, dTerm, pwmOutput, e) = struct.unpack("<ffffif", payload[1:])
                    self.gui.logComm("PidDebug: pid_dt: %f, pTerm: %f, iTerm: %f, dTerm: %f, pwmOutput: %d, e: %f" % (pid_dt, pTerm, iTerm, dTerm, pwmOutput, e))
                elif msgType == RespSDReadError:
                    # payload: SD errorcode, SPI status byte
                    (errorCode, spiStatus) = struct.unpack("<BB", payload[1:])
                    self.gui.logError("SDReadErr: errCode: 0x%x, spiStat: 0x%x" % (errorCode, spiStatus))
                elif msgType == FilSensorDebug:
                    # payload: deltaStepperSteps(float) deltaSensorSteps(i32) filSensorCalibration(float) slip(float) s(float)
                    (deltaStepperSteps, deltaSensorSteps, filSensorCalibration, slip, s) = struct.unpack("<fifff", payload[1:])
                    self.gui.logError("FilSensorDebug: deltaStepperSteps: %f deltaSensorSteps: %d filSensorCalibration: %f slip: %f s: %f" % (deltaStepperSteps, deltaSensorSteps, filSensorCalibration, slip, s))
                elif msgType == GenericMessage:
                    # payload: generic message string
                    (slen,) = struct.unpack("<B", payload[1:2])
                    message = payload[2:2+slen]
                    self.gui.logComm("GenericMessage: '%s'" % message)
                elif msgType == BufDebug:
                    (swapdev, sdreader) = struct.unpack("<II", payload[1:])
                    self.gui.logComm("BufDebug: swapdev: %d, sdreader: %d" % (swapdev, sdreader))

            except struct.error:
                self.gui.logComm("handleUnsolicitedMsg: warning, error unpacking struct! Len data: %d" % len(payload))

            return True # consume message

        return False # continue message processing


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
                # print "timeout reading, data read: %d bytes, '%s'" % (len(res), res)
                raise RxTimeout()

        return res

    def readResponse(self):

        s = self.readWithTimeout(1) # SOH

        # xxx loop til SOH found
        assert(s == cobs.nullByte)

        rc = self.readWithTimeout(1) # read response code
        crc = crc_ccitt_kermit.crc16_kermit(rc, 0xffff)
        cmd = ord(rc)

        l = self.readWithTimeout(1) # read length byte
        crc = crc_ccitt_kermit.crc16_kermit(l, crc)

        length = ord(l) - 1
        if debugComm:
            self.gui.logComm("Response 0x%x, reading %d b" % (cmd, length))

        payload = ""
        if length:
            payload = self.readWithTimeout(length) # read payload
            crc = crc_ccitt_kermit.crc16_kermit(payload, crc)

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
        return (cmd, payload)

    def initSerial(self, device, br, bootloaderWait=False):

        if self.isOpen():
            print "\nWARNING: initSerial() already done."
            self.resetLineNumber()
            return

        # Avoid reset of (arduino like) printer boards
        f = open(device)
        attrs = termios.tcgetattr(f)
        attrs[2] = attrs[2] & ~termios.HUPCL
        termios.tcsetattr(f, termios.TCSAFLUSH, attrs)
        f.close()

        self.port = device
        self.baudrate = br
        self.timeout = 0.05
        self.writeTimeout = 10
        # self.setDTR(False) # does not work?
        self.open()

        if bootloaderWait:
            # print "Initial bootloader wait..."
            time.sleep(0.1)

        # Read left over garbage
        recvLine = self.safeReadline()        
        while recvLine:
            # self.gui.logComm("Initial read: '%s'" % recvLine)
            self.gui.logComm("Initail read: " + recvLine.encode("hex"), "\n")
            recvLine = self.safeReadline()        

        self.resetLineNumber()

    # Initialize serial interface and download printer settings.
    def commandInit(self, args):

        if not self.isOpen() and args.mode != "pre":
            self.initSerial(args.device, args.baud, True)

        if not self.printerProfile:
            self.initPrinterProfile(args)

        if args.mode == "pre":
            self.commandInitDone = True
            return 

        settings = self.printerProfile.getSettingsI(args.pidset)

        # todo: move all settings into CmdSetHostSettings call
        self.sendCommandParamV(CmdSetFilSensorCal, [packedvalue.float_t(settings["filSensorCalibration"])])

        self.sendCommandParamV(CmdSetPIDValues, [
            packedvalue.float_t(settings["Kp"]),
            packedvalue.float_t(settings["Ki"]),
            packedvalue.float_t(settings["Kd"]),
            packedvalue.float_t(settings["KpC"]),
            packedvalue.float_t(settings["KiC"]),
            packedvalue.float_t(settings["KdC"]),
            packedvalue.uint16_t(int(settings["Tu"] * 1000)), # xxx Tu < 65 s
            ])

        self.sendCommandParamV(CmdSetIncTemp, [packedvalue.uint8_t(HeaterEx1), packedvalue.int16_t(args.inctemp)]);

        self.sendCommandParamV(CmdSetStepsPerMM, [
            packedvalue.uint16_t(settings["stepsPerMMX"]),
            packedvalue.uint16_t(settings["stepsPerMMY"]),
            packedvalue.uint16_t(settings["stepsPerMMZ"])
            ])

        self.sendCommandParamV(CmdSetHostSettings, [
            packedvalue.uint32_t(settings["buildVolX"]),
            packedvalue.uint32_t(settings["buildVolY"]),
            packedvalue.uint32_t(settings["buildVolZ"])
            ])

        if not self.commandInitDone:
            self.sendCommand(CmdPrinterInit)

        self.commandInitDone = True

    def resetLineNumber(self):
        self.lineNr = 1
        self.sendCommand(CmdResetLineNr)
   
    def incLineNumber(self):
        self.lineNr += 1
        if self.lineNr == 256:
            self.lineNr = 1

    def prevLineNumber(self):
        if self.lineNr == 1:
            return 255
        return self.lineNr-1

    # Send a command to the printer
    def send(self, cmd):

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
        checksum = crc_ccitt_kermit.crc16_kermit(header, 0xffff)

        if binPayload:
            binary += binPayload
            checksum = crc_ccitt_kermit.crc16_kermit(binPayload, checksum)

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

    # Use query() if you need the result of the command
    def sendCommand(self, cmd, binPayload=None):

        cobsBlock = binPayload and cobs.encodeCobsString(binPayload)
        self.sendCommandC(cmd, cobsBlock)

    # Use query() if you need the result of the command
    def sendCommandC(self, cmd, cobsBlock):

        binary = self.buildBinaryCommand(cmd, cobsBlock)
        self.send2(cmd, binary)

    # Use query() if you need the result of the command
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
    def query(self, cmd, binPayload=None, doLog="notused-variable"):

        # if doLog:
            # self.gui.logComm("query: ", CommandNames[cmd])

        cobsBlock = binPayload and cobs.encodeCobsString(binPayload)
        binary = self.buildBinaryCommand(cmd, cobsBlock)

        reply = self.send2(cmd, binary)

        if reply == ResendWasOK:
            # Command was ok, but no response due to reconnect, restart query:
            return self.query(cmd, binPayload)

        return reply

    def reconnect(self):

        # XXX add timeout, or otherwise prevent re-connection to power-cycled printer?!
        self.close()

        try:
            self.initSerial(self.port, br=self.baudrate)
        except SerialException as ex:
            self.gui.log("reconnect() Exception raised:", ex)
            time.sleep(1)
            self.reconnect()

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

        if debugComm:
            self.gui.logComm("*** sendCommand %s (0x%x, len: %d, seq: %d) *** " % (CommandNames[sendCmd], sendCmd, len(binary), self.prevLineNumber()))
        # print "send: ", binary.encode("hex")
        self.send(binary)

        # print "Waiting for reply code 0x%x (or ack)" % sendCmd

        # Wait for response, xxx without timeout/retry for now
        while True:

            try:
                # recvLine = self.safeReadline()        
                (cmd, payload) = self.readResponse()        
            except SERIALDISCON:
                self.gui.logError("Line disconnected in send2(), reconnecting!")
                self.reconnect()

                startTime = time.time()
                self.send(binary)
                continue

            except RxTimeout:
                self.gui.logComm("RxTimeout, resending command...")
                startTime = time.time()
                self.send(binary)
                continue

            except RxChecksumError:
                self.gui.logComm("RxChecksumError, resending command...")
                startTime = time.time()
                self.send(binary)
                continue

            n = len(binary)
            dt = time.time() - startTime
            if cmd == 0x6:
                if debugComm:
                    self.gui.logComm("ACK, Sent %d bytes in %.2f ms, %.2f Kb/s" % (n, dt*1000, n/(dt*1000)))
                return (cmd, payload)

            # if cmd == RespGenericString:
                # print "Got Generic String: '%s'" % payload
                # continue

            if self.handleUnsolicitedMsg(cmd, payload):
                continue

            resendCommand = self.checkErrorResponse(cmd, payload)

            if resendCommand == ResendWasOK:

                # Command ok without ack
                return ResendWasOK

            elif resendCommand:

                # command resend
                startTime = time.time()
                self.send(resendCommand)
                continue

            if debugComm:
                self.gui.logComm("ACK (0x%x/0x%x), Sent %d bytes in %.2f ms, %.2f Kb/s" % (cmd, sendCmd, n, dt*1000, n/(dt*1000)))

            if cmd == sendCmd:
                # print "got reply:", payload
                return (cmd, payload)

            print "unknown reply: 0x%x" % cmd, payload
            assert(0)

        # Notreached

    def getStatus(self):

        valueNames = ["state", "t0", "t1", "Swap", "SDReader", "StepBuffer", "StepBufUnderRuns", "targetT1", "pwmOutput", "slippage", "extruder_pos", "minBuffer", "underTemp", "underGrip"]

        (cmd, payload) = self.query(CmdGetStatus, doLog=False)

        # print "len payload:", len(payload)


        # XXXXXX debug workaround, payload 37, needed: 36
        if len(payload) > 40:
            print "WARNING: payload to long, stripping bytes #", len(payload)-36
            payload = payload[:36]

        tup = struct.unpack("<BffIHIhHBfiIHH", payload)

        statusDict = {}
        for i in range(len(valueNames)):

            valueName = valueNames[i]

            statusDict[valueName] = tup[i]

        # print "statusDict:", statusDict

        # Update print duration
        if statusDict["state"] == StateInit and self.printEndedAt == None:
            self.printEndedAt = time.time()

        self.gui.statusCb(statusDict)

        for (statusCmd, statusName, tasknames) in [(CmdGetTaskStatus, "TOP:", ("idle", "tempcontrol", "tempheater", "filsensor", "ubscommand", "txbuffer", "swapdev", "fillbuffer", "tasksum")), (CmdGetIOStats, "IOStats:", ("read", "readSum", "write", "writeSum"))]:

            ## XXX debug iotimings
            (cmd, payload) = self.query(statusCmd, doLog=True)

            # print "cmd, len payload:", cmd, len(payload)

            lenp =(len(payload) / 4) * 4;

            # XXXXXX debug workaround,
            if len(payload) > lenp:
                print "WARNING: payload to long, stripping bytes #", len(payload)-lenp
                payload = payload[:lenp]

            structFmt = "<" + "I" * (len(payload) / 4)
            tup = struct.unpack(structFmt, payload)

            tupindex = 0
            print statusName
            print "%15s %10s %10s %10s" % ("Task", "#Calls", "TSum", "TLongest")
            for taskname in tasknames:
                nCalls = tup[tupindex]
                tSum = tup[tupindex+1]
                print "%15s %10d %10d %10d" % (taskname, nCalls, tSum, tup[tupindex+2])
                tupindex+=3

        return statusDict

    # Prettyprint pritner status
    def ppStatus(self, statusDict, msg=""):

        slipstr = "----"
        if statusDict["slippage"]:
            slipstr = "%4.2f" % (1.0/statusDict["slippage"])
        if msg:
            print msg
        print "State: %d, Bed: %5.1f, Hotend: %5.1f(%5.1f), Pwm: %3d, Swap: %10s, MinBuffer: %3d, Grip: %s" % \
            (statusDict["state"], statusDict["t0"], statusDict["t1"], statusDict["targetT1"], statusDict["pwmOutput"], util.sizeof_fmt(statusDict["Swap"]), statusDict["minBuffer"], slipstr)

    # Get printer (-profile) name from printer eeprom
    def getPrinterName(self):

        resp = self.query(CmdGetPrinterName)
        pn = util.getResponseString(resp[1], 1)
        return pn

    def setPrinterName(self, args):

        self.initSerial(args.device, args.baud, True)

        self.sendCommandParamV(CmdSetPrinterName,
            [packedvalue.pString_t(args.name)])

    #
    # The used printer profile is normally determined by the printername
    # read from the printer. Printername can be overridden by command
    # line argument, for example in preprocessing mode.
    #
    # To read from printer, initSerial() has to be called before.
    #
    def initPrinterProfile(self, args):

        if args.mode == "pre":
            self.printerProfile = PrinterProfile(args.printer)
        else:
            if not self.isOpen():
                self.initSerial(args.device, args.baud, True)
            self.printerProfile = PrinterProfile(self.getPrinterName())

    # Set a gpio port on printer
    def setGpio(self, pin, value):

        self.sendCommandParamV(CmdSetGpio, [packedvalue.uint8_t(pin), packedvalue.uint8_t(value)])

    def waitForState(self, destState, wait=1, log=False):

        status = self.getStatus()
        if log:
            pprint.pprint(status)

        while status['state'] != destState:
            time.sleep(wait)
            status = self.getStatus()
            if log:
                pprint.pprint(status)

    def stateMoving(self, status=None):

        if not status:
            status = self.getStatus()

        return status['state'] == StateStart

    def getTemp(self, doLog = False):

        #
        # CmdGetCurrentTemps returns (bedtemp, T1 [, T2])
        #
        (cmd, payload) = self.query(CmdGetCurrentTemps, doLog=doLog)
        if len(payload) == 8:
            temps = struct.unpack("<ff", payload)
        else:
            temps = struct.unpack("<fff", payload)

        return temps

    def getTemps(self, doLog = False):

        temps = self.getTemp()
        (cmd, payload) = self.query(CmdGetTargetTemps, doLog=doLog)
        if len(payload) == 3:
            targetTemps = struct.unpack("<BH", payload)
        else:
            targetTemps = struct.unpack("<BHH", payload)

        self.gui.tempCb(temps[0], temps[1], targetTemps[1])
        return temps

    ####################################################################################################

    def heatUp(self, heater, temp, wait=None, log=False):

        assert(type(temp) == types.IntType)

        payload = struct.pack("<BH", heater, temp) # Parameters: heater, temp
        self.sendCommand(CmdSetTargetTemp, binPayload=payload)

        while wait !=  None:
            time.sleep(2)
            temps = self.getTemps()

            if log:
                print "\rTemp: %.2f (%.2f)" % (temps[heater], wait),
                sys.stdout.flush()

            if temps[heater] >= wait:
                break

        print "\n"

    ####################################################################################################
    #
    # Slowly increase temperature (from off or a low value) to avoid big overshot,
    # this is some sort *setpoint ramping*.
    #
    # Note: Without ramping, the Ziegler Nichols (0.9) pid settings on the UM2 gives about 40%
    #       overshot when starting from room temperature, the Jennyprinter gives about 35% overshot.
    #
    def heatUpRamp(self, heater, tdest, log=False):

        assert(type(tdest) == types.IntType)

        # Ramp up time: tu+tg from the autotune step response.
        timeConstant = self.printerProfile.getTuI() + self.printerProfile.getTgI() 

        status = self.getStatus()
        startTemp = status["t1"]
        startTime = time.time()

        a = tdest / timeConstant

        print "Start temprapmp...", tdest, timeConstant, a

        if a <= 0:
            payload = struct.pack("<BH", heater, tdest)
            self.sendCommand(CmdSetTargetTemp, binPayload=payload)
            return

        step = 1

        while True:

            status = self.getStatus()
            temp = status["t1"]

            if temp < tdest-2:

                t = min(startTemp + (time.time() - startTime) * a, tdest)

                print "temp is below dest", temp, t, tdest

                payload = struct.pack("<BH", heater, t)
                self.sendCommand(CmdSetTargetTemp, binPayload=payload)

                yield(temp)

            else:

                print "temp reached"
                payload = struct.pack("<BH", heater, tdest)
                self.sendCommand(CmdSetTargetTemp, binPayload=payload)
                yield(temp)
                break

    ####################################################################################################
    # Start printing process, and record print start time
    def startPrint(self):

        self.printStartedAt = time.time()
        self.printEndedAt = None
        self.sendCommandParamV(CmdMove, [MoveTypeNormal])

    ####################################################################################################

    def adjTemp(self, heater, adj):

        payload = struct.pack("<Bh", heater, adj) # Parameters: heater, adjtemp
        self.sendCommand(CmdSetIncTemp, binPayload=payload)

    ####################################################################################################

    def setTempPWM(self, heater, pwmvalue):

        payload = struct.pack("<Bh", heater, pwmvalue) # Parameters: heater, adjtemp
        self.sendCommand(CmdSetTempPWM, binPayload=payload)

    ####################################################################################################

    def coolDown(self, heater, temp=0, wait=None, log=False):

        assert(type(temp) == types.IntType)

        if log:
            print "coolDown(): cooling down hotend..."

        if heater > HeaterBed:
            # Switch on PID mode
            payload = struct.pack("<BB", heater, 0)
            self.sendCommand(CmdSetTempPWM, binPayload=payload)

        payload = struct.pack("<BH", heater, temp)
        self.sendCommand(CmdSetTargetTemp, binPayload=payload)

        while wait !=  None:
            time.sleep(2)
            temps = self.getTemps()

            if log:
                print "\rTemp: %.2f (%.2f)" % (temps[heater], wait),
                sys.stdout.flush()

            if temps[heater] <= wait:
                break

        if log:
            print ""

    ####################################################################################################

    def isHomed(self):

        (cmd, payload) = self.query(CmdGetHomed, doLog=False)
        return struct.unpack("<B", payload)[0] == 1

    ####################################################################################################

    def getPos(self):

        (cmd, payload) = self.query(CmdGetPos, doLog=False)
        tup = struct.unpack("<iiii", payload)
        return tup

    ####################################################################################################

    def getEndstops(self):

        # Check, if enstop was pressed
        (cmd, payload) = self.query(CmdGetEndstops, doLog=False)
        tup = struct.unpack("<BiBiBi", payload)
        return tup

    ####################################################################################################

    def endStopTriggered(self, dim, fakeHomingEndstops=False):

        # Check, if enstop was pressed
        tup = self.getEndstops()

        if tup[dim*2] or fakeHomingEndstops:
            print "Endstop %s hit at position: %d" % (dimNames[dim], tup[dim*2+1])
            return True

        print "Endstop %s open at position: %d" % (dimNames[dim], tup[dim*2+1])
        return False

    ####################################################################################################

    # Is this useful (anymore)?
    def getTempTable(self):

        (cmd, payload) = self.query(CmdGetTempTable)
        basetemp = struct.unpack("<H", payload[:2])[0]

        l = ord(payload[2])
        fmt = "<" + l*"H"
        return (basetemp, struct.unpack(fmt, payload[3:]))

    ####################################################################################################

    def getFilSensor(self):

        (cmd, payload) = self.query(CmdGetFilSensor)
        t = struct.unpack("<i", payload)
        return t[0]

    ####################################################################################################

    def getFreeMem(self):

        (cmd, payload) = self.query(CmdGetFreeMem)
        t = struct.unpack("<I", payload)
        return t[0]

    def getFSReadings(self, notused_nNeadings=None):

        (cmd, payload) = self.query(CmdGetFSReadings)

        readings = []
        for i in range(10):
            (ts, dy) = struct.unpack("<Ih", payload[i*6:(i+1)*6])
            readings.append((ts, dy))
        return readings

    ####################################################################################################

    # Get current stepper direction bits
    def getDirBits(self):

        (cmd, payload) = self.query(CmdGetDirBits, doLog=False)
        return struct.unpack("<B", payload)[0]

    ####################################################################################################

    # Get print duration, string-formatted
    def getPrintDuration(self):

        delta = 0.0

        if self.printStartedAt:

            if self.printEndedAt:
                delta = self.printEndedAt - self.printStartedAt
            else:
                delta = time.time() - self.printStartedAt

        hours, remainder = divmod(delta, 3600)
        minutes, seconds = divmod(remainder, 60)
        return '{:0}:{:02}:{:02}'.format(int(hours), int(minutes), int(seconds))

    ####################################################################################################

    # Read a gpio port from printer
    def readGpio(self, pin):

        payload = struct.pack("<B", pin)
        (cmd, payload) = self.query(CmdReadGpio, binPayload=payload)
        return struct.unpack("<I", payload)[0]

    ####################################################################################################
    #
    # Read analog value from a gpio port from printer
    #
    def readAnalogGpio(self, pin):

        payload = struct.pack("<B", pin)
        (cmd, payload) = self.query(CmdReadAnalogGpio, binPayload=payload)
        return struct.unpack("<I", payload)[0]

    ####################################################################################################
    #
    # Emegency hard reset printer
    #
    def systemReset(self):

        self.sendCommand(CmdSystemReset)

    ####################################################################################################































































































































































































































































