# -*- coding: utf-8 -*-
#
#/*
# This file is part of ddprint - a 3D printer firmware.
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
import time, struct, crc_ccitt_kermit, termios, pprint, sys, argparse
import types
import dddumbui, cobs, intmath, ddprintutil as util

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

    # maxTXErrors = 10
    maxTXErrors = 3

    def __init__(self, args, gui=None):

        if gui:
            self.gui = gui
        else:
            self.gui = dddumbui.DumbGui()

        Serial.__init__(self, exclusive=True)

        self.args = args

        # Command sequence number [1..255]
        self.lineNr = 1

        # The last (max. 256) commands sent, for command re-send
        self.lastCommands = {}

        # Retry counter on tx errors
        self.sendErrors = 0
        self.txErrors = 0

        self.printStartedAt = None
        self.printEndedAt = None

        self.printerProfile = None

        # xxx debug
        self.commandInitDone = False

        self.stallwarn = argparse.Namespace(
            lastSwap = 0,
            lastSteps = 0,
            lastSDReader = 0,
            )

        self.baudrates = [1000000, 500000, 250000]
        self.defaultBaudrateIndex = 1 # Start with 500 Kbaud
        self.minBaudrateIndex = self.defaultBaudrateIndex
        self.baudrateIndex = self.minBaudrateIndex
        self.lastAutobaud = 0
        self.throttleAutoBaud = 60

        # debug
        # for i in [250000, 500000, 1000000]:
            # print "BRR value for baudrate %d: %d" % (i, (fCPU+i*8)/(i*16)-1)

        br = self.baudrates[self.baudrateIndex]
        print("Setting initial baudrate to:", br)
        self.baudrate = br

    def __del__(self):

        if self.baudrateIndex != self.defaultBaudrateIndex:
            br = self.baudrates[self.defaultBaudrateIndex]
            print("Resetting baudrate to:", br)
            self.setBaudRate(self.defaultBaudrateIndex)

    def checkStall(self, status):

        if (self.stallwarn.lastSwap == status.Swap):
            print("Swap did not change...")
        if (self.stallwarn.lastSteps == status.StepBuffer) and (self.stallwarn.lastSDReader == status.SDReader):
            print("stall ???")

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

        return self.lastCommands[lastLine]

    # Check a printer response for an error
    def checkErrorResponse(self, respCode, payload, handleError=True):

        if respCode == RespUnknownCommand:

            self.gui.logError("ERROR: RespUnknownCommand '0x%x'" % payload[0])
            if handleError:
                raise FatalPrinterError(ResponseNames[respCode])

        elif respCode == RespKilled:

            reason = payload[0]
            if reason in [RespHardwareEndstop, RespSoftwareEndstop]:

                (x, y, z, xtrig, ytrig, ztrig) = struct.unpack("<iiiBBB", payload[1:])
                self.gui.logError("ERROR: PRINTER KILLED! Reason: %s, X: %d, Y: %d, Z: %d, trigger: x: %d, y: %d, z: %d" % (RespCodeNames[reason], x, y, z, xtrig, ytrig, ztrig))

            elif reason == RespUnknownBCommand:

                self.gui.logError("ERROR: PRINTER KILLED! Reason: %s, command: 0x%x" % (RespCodeNames[reason], payload[1]))

            elif reason == RespAssertion:

                # Useless python struct 'p' format...
                (line, slen) = struct.unpack("<HB", payload[1:4])
                filename = payload[4:4+slen]
                self.gui.logError("ERROR: PRINTER KILLED! Reason: %s, Line: %d, File: %s" % (RespCodeNames[reason], line, filename.decode()))

            elif reason in [RespSDReadError, RespSDWriteError]:

                (errorCode, spiStatus) = struct.unpack("<BB", payload[1:])
                self.gui.logError("ERROR: PRINTER KILLED! Reason: %s, ErrorCode: %d (0x%x), spiStatus: 0x%x" % (RespCodeNames[reason], errorCode, errorCode, spiStatus))

            elif reason == RespMinTemp:

                heater = payload[1]
                self.gui.logError("ERROR: PRINTER KILLED! Reason: %s, Heater: %d" % (RespCodeNames[reason], heater))

            elif reason == RespMaxTemp:

                heater = payload[1]
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

            lastLine = payload[0]
            self.gui.logComm("RespRXCRCError:", lastLine)

            if handleError:

                rs = self.commandResend(lastLine)

                if rs == ResendWasOK:
                    return rs

                self.sendErrors += 1
                return rs

        elif respCode == RespSerNumberError:

            lastLine = payload[0]
            self.gui.logComm("RespSerNumberError:", lastLine)
            if handleError:
                return self.commandResend(lastLine)

        elif respCode == RespRXTimeoutError:

            lastLine = payload[0]
            self.gui.logComm("RespRXTimeoutError:", lastLine)
            if handleError:

                rs = self.commandResend(lastLine)

                if rs == ResendWasOK:
                    return rs

                self.sendErrors += 1
                return rs

        # elif respCode == RespRXFullError:

            # lastLine = payload[0]
            # self.gui.logComm("RespRXTFullEror:", lastLine)
            # if handleError:
                # return self.commandResend(lastLine)

        elif respCode == RespUnderrun:

            (p1, p2, p3, p4, p5) = struct.unpack("<IIIII", payload)
            self.gui.logError("ERROR: lastsize: %d, lastsize2: %d, minTimer: %d, swap: %d, sdreader: %d" % (p1, p2, p3, p4, p5))
            if handleError:
                raise FatalPrinterError(ResponseNames[respCode])

        elif respCode == RespUnsolicitedMsg:

            self.handleUnsolicitedMsg(respCode, payload)

        elif respCode < 128: # Range of errorcodes

            self.gui.logError("ERROR: unknown response code '0x%x'" % respCode)
            if handleError:
                raise FatalPrinterError(ResponseNames[respCode])

    # Check a printer response for an error
    def handleUnsolicitedMsg(self, respCode, payload):

        if respCode == RespUnsolicitedMsg:

            msgType = payload[0]
            self.gui.logComm("RespUnsolicitedMsg:", msgType)

            try:

                if msgType == ExtrusionLimitDbg:
                    (tempIndex, actSpeed, targetSpeed, grip) = struct.unpack("<hhhh", payload[1:])
                    self.gui.logComm("Limit extrusion index: %d, act: %d, target: %d, grip: %d" % (tempIndex, actSpeed, targetSpeed, grip))
                elif msgType == PidDebug or msgType == PidSwitch:
                    (pid_dt, pTerm, iTerm, dTerm, pwmOutput, e, esum) = struct.unpack("<iiiiBhi", payload[1:])
                    self.gui.logComm("%s: pid_dt: %d, pTerm: %d, iTerm: %d, dTerm: %d, pwmOutput: %d, e_16: %d, esum: %d" % (((msgType == PidDebug) and "PidDebug") or "PidSwitch", pid_dt, pTerm, iTerm, dTerm, pwmOutput, e, esum))
                elif msgType in (RespSDReadError, RespSDWriteError):
                    # payload: SD errorcode, SPI status byte
                    (errorCode, spiStatus) = struct.unpack("<BB", payload[1:])
                    self.gui.logError("%s: errCode: 0x%x (0x%x), spiStat: 0x%x" % (RespCodeNames[msgType], errorCode, errorCode, spiStatus))
                elif msgType == FilSensorDebug:
                    # payload: deltaStepperSteps(float) deltaSensorSteps(i32) filSensorCalibration(float) slip(float) s(float)
                    (deltaStepperSteps, deltaSensorSteps, filSensorCalibration, slip, s) = struct.unpack("<fifff", payload[1:])
                    self.gui.logError("FilSensorDebug: deltaStepperSteps: %f deltaSensorSteps: %d filSensorCalibration: %f slip: %f s: %f" % (deltaStepperSteps, deltaSensorSteps, filSensorCalibration, slip, s))
                elif msgType == GenericMessage:
                    # payload: generic message string
                    (slen,) = struct.unpack("<B", payload[1:2])
                    message = payload[2:2+slen]
                    self.gui.logComm("GenericMessage: '%s'" % message)
                elif msgType == GenericInt32:
                    # (p1, p2, p3, p4, p5) = struct.unpack("<iiiii", payload[1:])
                    (p1, ) = struct.unpack("<i", payload[1:])
                    # self.gui.logComm("GenericInt32: %d(0x%x), %d(0x%x),%d(0x%x), %d(0x%x), %d(0x%x)" % (p1, p1, p2, p2, p3, p3, p4, p4, p5, p5))
                    self.gui.logComm("GenericInt32: %d(0x%x)" % (p1, p1))
                elif msgType == BufDebug:
                    (swapdev, sdreader) = struct.unpack("<II", payload[1:])
                    self.gui.logComm("BufDebug: swapdev: %d, sdreader: %d" % (swapdev, sdreader))

            except struct.error:
                self.gui.logComm("handleUnsolicitedMsg: warning, error unpacking struct! Len data: %d" % len(payload))

            return True # consume message

        return False # continue message processing


    # Read usb-serial device, "ignore" exceptions
    def safeRead(self):

        try:
            return self.read(self.in_waiting)
        except SerialException as ex:
            self.gui.log("Readline() Exception raised:", ex)

        return ""

    def readWithTimeout(self, length):

        try:
            res = self.read(length)
        except SerialException as ex:
            self.gui.log("Readline(): disconnected? - exception raised:", ex)
            raise SERIALDISCON

        # Receive without error
        if not res:
            # print "timeout reading, data read: %d of %d bytes, '%s'" % (len(res), length, res)
            raise RxTimeout()

        l = len(res)
        if l < length:
            print(f"readWithTimeout(): short read {l}/{length}, retry read")
            return res + readWithTimeout(length - l)

        return res

    def readResponse(self):

        startByte = self.readWithTimeout(1)
        while (startByte != cobs.NullBytes):
            print("waiting for SOH, read garbage: 0x%x" % startByte[0])
            startByte = self.readWithTimeout(1)

        header = self.readWithTimeout(2) # SOH + RC + lenbyte

        # Response code
        cmd = header[0]

        # Length byte
        l = header[1]
        length = l - 1

        # if debugComm:
            # self.gui.logComm("Response 0x%x, reading %d b" % (cmd, length))

        # Checksum
        crc = crc_ccitt_kermit.crc16_kermit(header, 0xffff)

        payload = b""
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
            print("RxChecksumError our: 0x%x, fw: 0x%x, payload: %s (hex), cflags: 0x%x" % (crc, checkSum, (header + payload).hex(), cflags))
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

        if payload != "":
            # Decode COBS encoded payload
            payload = cobs.decodeCobs(payload)

        return (cmd, payload)

    def initSerial(self, device, bootloaderWait=False):

        if self.isOpen():
            print("\nWARNING: initSerial() already done.")
            self.resetLineNumber()
            return

        # Avoid reset of (arduino like) printer boards
        # Note: the first open() of the device will still do a reset
        #       because HUPCL not cleared, yet.
        # Note: some linux drivers (cp210x) still toggle DTR on open
        #       even with the HUPCL hack :-(
        f = open(device)
        attrs = termios.tcgetattr(f)
        attrs[2] = attrs[2] & ~termios.HUPCL
        termios.tcsetattr(f, termios.TCSAFLUSH, attrs)
        f.close()

        self.port = device
        # Py-Serial read timeout
        self.timeout = 3
        self.writeTimeout = 10
        self.open()

        if bootloaderWait:
            # print "Initial bootloader wait..."
            time.sleep(0.1)

        # Read left over garbage
        recvLine = self.safeRead()
        while recvLine:
            # self.gui.logComm("Initial read: '%s'" % recvLine)
            self.gui.logComm("Initail read: " + recvLine.encode("hex"), "\n")
            recvLine = self.safeRead()

        self.resetLineNumber()

    # Initialize serial interface and download printer settings.
    def commandInit(self):

        if not self.isOpen() and self.args.mode != "pre":
            self.initSerial(self.args.device, True)

        if not self.printerProfile:
            self.initPrinterProfile()

        if self.args.mode == "pre":
            self.commandInitDone = True
            return 

        settings = self.printerProfile.getSettings(self.args.pidset)

        # todo: move all settings into CmdSetHostSettings call

        # We want at least 50 FRS counts for a measurement, compute the neccessary
        # extruder stepper steps for this:
        fsrMinSteps = int(50 / settings["filSensorCalibration"])

        self.sendCommandParamV(
                CmdSetFilSensorConfig, (
                    intmath.fsCalibration(settings["filSensorCalibration"]),
                    packedvalue.uint16_t(fsrMinSteps)
                    )
                )

        (ki, maxEsum16H) = intmath.pidScaleKi(settings["Ki"])
        (kiC, maxEsum16HC) = intmath.pidScaleKi(settings["KiC"])

        self.sendCommandParamV(CmdSetPIDValues, (

            intmath.pidScaleKp(settings["Kp"]),
            ki,
            packedvalue.int32_t( maxEsum16H ),
            intmath.pidScaleKd(settings["Kd"]),

            intmath.pidScaleKp(settings["KpC"]),
            kiC,
            packedvalue.int32_t( maxEsum16HC ),
            intmath.pidScaleKd(settings["KdC"]),

            # Scaling factor to switch pid-set from cooling to heating
            intmath.pidSwitch(settings["KiC"], settings["Ki"]),

            # Scaling factor to switch pid-set for heating to cooling
            intmath.pidSwitch(settings["Ki"], settings["KiC"]),

            packedvalue.uint16_t(int(settings["Tu"] * 1000)), # xxx Tu < 65 s
            ))

        self.adjTemp(HeaterEx1, self.args.inctemp)

        self.sendCommandParamV(CmdSetStepsPerMM, (
            packedvalue.uint16_t(settings["stepsPerMMX"]),
            packedvalue.uint16_t(settings["stepsPerMMY"]),
            packedvalue.uint16_t(settings["stepsPerMMZ"])
            ))

        self.sendCommandParamV(CmdSetHostSettings, (
            packedvalue.uint32_t(settings["buildVolX"]),
            packedvalue.uint32_t(settings["buildVolY"]),
            packedvalue.uint32_t(settings["buildVolZ"]),
            packedvalue.uint8_t(settings["xHomeDir"]),
            packedvalue.uint8_t(settings["yHomeDir"]),
            packedvalue.uint8_t(settings["zHomeDir"]),
            ))

        if not self.commandInitDone:
            self.sendCommand(CmdPrinterInit)

        self.commandInitDone = True

    def setBaudRate(self, brIndex, setInFirmware=True):

        self.baudrateIndex = brIndex
        baudrate = self.baudrates[brIndex]

        if setInFirmware:

            lowbyte = (fCPU+baudrate*8) // (baudrate*16) - 1;
            print("Auto-Baudrate: set to %d (lowbyte: %d)" % (baudrate, lowbyte))

            payload = struct.pack("<I", lowbyte)
            self.sendCommand(CmdSetBaudRate, binPayload=payload)

            print("Baudrate command successful")

        self.baudrate = baudrate
        time.sleep(0.1)

    def resetLineNumber(self):

        # self.sendCommand(CmdResetLineNr)
        binary = self.buildBinaryCommand(CmdResetLineNr)

        # self.send2(cmd, binary)
   
        baudRatesToTry = list(range(len(self.baudrates)))
        del baudRatesToTry[self.baudrateIndex]
        timeouts = 0

        while True:

            print("Connecting to printer...")
            self.send(binary)

            try:
                (cmd, _) = self.readResponse()
            except RxTimeout:

                timeouts += 1
                if timeouts > 2:

                    # Try switching baudrate
                    if baudRatesToTry:

                        print("available baudrates:", baudRatesToTry)

                        brIndex = baudRatesToTry[0]
                        del baudRatesToTry[baudRatesToTry.index(brIndex)]

                        print("Set baudrate to: ", self.baudrates[brIndex])

                        self.setBaudRate(brIndex, False)
                        self.lastAutobaud = time.time()
                        timeouts = 0

                    else:

                        raise SerialException("unable to connect to printer")

                continue

            except RxChecksumError:

                self.gui.logComm("RxChecksumError, resending command...")
                continue

            if cmd == 0x6:
                print("ACK received, connection ok")
                break

        self.lineNr = 1

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

    # Fixme: mostly duplicates buildBinaryCommand(). and move.command()
    def buildBinaryCommand512(self, binPayload):

        binary  = struct.pack("<B", SOH)

        header = struct.pack("<BB", self.lineNr, CmdG1)

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

        # self.gui.log("checkSum: ", checksum, " 0x%x" % checksum)
        binary += struct.pack("<BH", cflags, checksum)

        # Store for later possible command resend
        self.lastCommands[self.lineNr] = binary

        self.incLineNumber()

        return binary

    # Use query() if you need the result of the command
    def sendCommand(self, cmd, binPayload=None):

        cobsBlock = binPayload and cobs.encodeCobs512(binPayload)
        
        binary = self.buildBinaryCommand(cmd, cobsBlock)
        self.send2(cmd, binary)

    # Send buffered command (512 bytes sector)
    def sendCommand512(self, cobsBlock):

        binary = self.buildBinaryCommand512(cobsBlock)

        # print "Dbg: sendCommand512 dbg: decoding command.."
        # self.dbgCommand512(binary)

        # print "sendCommand512.."
        if self.args.mode != "pre":
            self.send2(cmd, binary)

        # print "sendCommand512 done2"

    def dbgCommand512(self, binary):

        assert(binary[0] == SOH)
        pnum = binary[1]
        print("packet num:", pnum)
        assert(binary[2] == CmdG1)

        (payload, lenread) = cobs.decodeCobs512(binary[3:])

        assert(len(payload) == 512)

        flags = binary[3+lenread]
        print("flags:", flags)

        c1 = binary[4+lenread]
        print("c1: 0x%x" % c1)
        c2 = binary[5+lenread]
        print("c2: 0x%x" % c2)

    # Use query() if you need the result of the command
    def sendCommandParamV(self, cmd, params):

        assert(params[0] != None)
        payload = bytearray()
        for p in params:
            if p:
                payload += p.pack()
        self.sendCommand(cmd, binPayload=payload)

    # Query info from printer, use this to receive returned data, use [sendCommand, sendCommandParamV] for 
    # 'write only' commands.
    # Note: Commands must be 'restartable' without sideeffects (see commandResend(), ResendWasOK)
    def query(self, cmd, binPayload=None, expectedLen=None):

        cobsBlock = binPayload and cobs.encodeCobs512(binPayload)
        binary = self.buildBinaryCommand(cmd, cobsBlock)

        reply = self.send2(cmd, binary)

        if reply == ResendWasOK:
            # Command was ok, but no response due to reconnect, restart query:
            reply = self.query(cmd, binPayload)

        # Debug 
        if expectedLen and len(reply[1]) != expectedLen:
            print("WARNING: reply size wrong: %d/%d" % (len(reply[1]), expectedLen))
            print("Hex dump:")
            print(reply[1].encode("hex"))

        return reply

    def reconnect(self):

        # XXX add timeout, or otherwise prevent re-connection to power-cycled printer?!
        self.close()

        try:
            self.initSerial(self.port)
        except SerialException as ex:
            self.gui.log("reconnect() Exception raised:", ex)
            time.sleep(1)
            self.reconnect()

    # Send command or Query info from printer
    def send2(self, sendCmd, binary):

        # Send cmd/query
        startTime = time.time()
        lineNr = self.lineNr

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
        # if binary:
        self.send(binary)

        # print "Waiting for reply code 0x%x (or ack)" % sendCmd

        # Wait for response, xxx without timeout/retry for now
        while True:

            try:
                (cmd, payload) = self.readResponse()
            except SERIALDISCON:

                self.gui.logError("Line disconnected in send2(), reconnecting!")
                self.reconnect()

                startTime = time.time()
                # if binary:
                self.send(binary)
                continue

            except RxTimeout:

                self.gui.logComm("RxTimeout, resending command...")
                startTime = time.time()
                # if binary:
                self.send(binary)
                continue

            except RxChecksumError:

                self.gui.logComm("RxChecksumError, resending command...")
                startTime = time.time()
                # if binary:
                self.send(binary)
                continue

            # n = 0
            # if binary:
            n = len(binary)

            dt = time.time() - startTime
            if cmd == 0x6:

                if debugComm:
                    self.gui.logComm("ACK, Sent %d bytes in %.2f ms, %.2f Kb/s" % (n, dt*1000, n/(dt*1000)))

                # Do 'auto-baudrate'
                if time.time() > (self.lastAutobaud + self.throttleAutoBaud):

                    self.lastAutobaud = time.time()

                    if (self.sendErrors > 10*(self.throttleAutoBaud/60)) and self.baudrateIndex < len(self.baudrates)-1:

                            if self.throttleAutoBaud < 300:
                                self.throttleAutoBaud += 60

                            self.setBaudRate(self.baudrateIndex + 1)

                            # if self.lineNr != lineNr:
                                # print "resetting linenr from %d to %d'" % (self.lineNr, lineNr);
                                # self.lineNr = lineNr

                    else:
                        
                        if self.baudrateIndex > self.minBaudrateIndex:

                            self.setBaudRate(self.baudrateIndex - 1)

                    self.sendErrors = 0
                    
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

            print("unknown reply: 0x%x" % cmd, payload)
            assert(0)

        # Notreached

    def getStatus(self):

        valueNames = ["state", "t0", "t1", "Swap", "swapsize", "SDReader", "StepBuffer", "StepBufUnderRuns", "targetT0", "targetT1", "pwmOutput", "slippage", "slowdown", "ePos", "minBuffer", "underTemp", "underGrip"]

        (cmd, payload) = self.query(CmdGetStatus, expectedLen=39)

        tup = struct.unpack("<BhhIIHIhhhBhHiBHH", payload[:39])

        status = argparse.Namespace()

        for i in range(len(valueNames)):

            valueName = valueNames[i]

            if valueName in ["t0", "t1", "t2", "targetT0", "targetT1"]:
                # Temperatures in firmware are in 1/16th °C
                status.__setattr__(valueName, intmath.fromFWTemp(tup[i]))
            elif valueName == "slippage":
                # In firmware are in 1/32th 
                status.__setattr__(valueName, tup[i] / 32.0)
            elif valueName == "slowdown":
                # In firmware are in 1/1024th 
                status.__setattr__(valueName, (tup[i] / 1024.0) - 1.0)
            else:
                status.__setattr__(valueName, tup[i])

        # Update print duration
        if status.state == StateInit and self.printEndedAt == None:
            self.printEndedAt = time.time()

        self.gui.statusCb(status)

        return status

    def top(self):

        for (statusCmd, statusName, tasknames) in [(CmdGetTaskStatus, "*** TOP: ***:", ("idle", "tempcontrol", "tempheater", "filsensor", "ubscommand", "txbuffer", "swapdev", "fillbuffer", "tasksum")), (CmdGetIOStats, "*** IOStats: ***", ("read", "readSum", "write", "writeSum"))]:

            ## XXX debug iotimings
            (cmd, payload) = self.query(statusCmd)

            # print "cmd, len payload:", cmd, len(payload)

            structFmt = "<" + "I" * (len(payload) // 4)
            tup = struct.unpack(structFmt, payload)

            tupindex = 0
            print(statusName)
            print("%15s %10s %10s %10s" % ("Task", "#Calls", "TSum", "TLongest"))
            for taskname in tasknames:
                nCalls = tup[tupindex]
                tSum = tup[tupindex+1]
                print("%15s %10d %10d %10d" % (taskname, nCalls, tSum, tup[tupindex+2]))
                tupindex+=3

    def printStatus(self, ns):

        s = "*** STATUS: ***:"

        gripstr = "----"
        if ns.slippage:
            gripstr = "%4.2f" % (1.0/ns.slippage)

        s += "\n    State: %s" % StateNames[ns.state]
        s += "\n    Temp : T1: %.2f(%.2f pwm: %3d) bed: %.2f(%.2f)" % (ns.t1, ns.targetT1, ns.pwmOutput, ns.t0, ns.targetT0)
        s += "\n    Buf  : swapsize: %10s swap: %10s" % (util.sizeof_fmt(ns.swapsize), util.sizeof_fmt(ns.Swap))
        s += "\n    FRS  : grip: %.4s " % gripstr
        s += "\n    Slow : slowdown: %.2f " % (ns.slowdown)
        s += "\n    Misc : templow: %4d griplow: %4d epos: %5d stepbuffer: %4d bufferlow: %4d minbuffer: %4d" % (ns.underTemp, ns.underGrip, ns.ePos, ns.StepBuffer, ns.StepBufUnderRuns, ns.minBuffer)

        print(s)

    # Prettyprint printer status
    def ppStatus(self, status, msg=""):

        gripstr = "----"
        if status.slippage:
            gripstr = "%4.2f" % (1.0/status.slippage)
        if msg:
            print(msg)
        print("Bed: %5.1f, Hotend: %5.1f(%5.1f), Pwm: %3d, Swap: %10s, MinBuffer: %3d, underrun: %5d, Grip: %.4s, SlowDown: %4.2f, underTemp %5d, underGrip: %5d" % \
            (status.t0, status.t1, status.targetT1, 
             status.pwmOutput, util.sizeof_fmt(status.Swap),
             status.minBuffer, status.StepBufUnderRuns, gripstr, status.slowdown,
             status.underTemp, status.underGrip))

    # Get printer (-profile) name from printer eeprom
    def getPrinterName(self):

        if self.args.mode == "pre":
            return self.args.printer

        if not self.isOpen():
            self.initSerial(self.args.device, True)

        resp = self.query(CmdGetPrinterName)
        pn = util.getResponseString(resp[1], 1)
        return pn.decode()

    def setPrinterName(self):

        self.sendCommandParamV(CmdSetPrinterName,
            [packedvalue.pString_t(self.args.name)])

    def getPrinterVersion(self):

        resp = self.query(CmdGetVersion)
        pn = util.getResponseString(resp[1], 1)
        return pn.decode()

    #
    # The used printer profile is normally determined by the printername
    # read from the printer. Printername can be overridden by command
    # line argument, for example in preprocessing mode.
    #
    # To read from printer, initSerial() has to be called before.
    #
    def initPrinterProfile(self):

        self.printerProfile = PrinterProfile(self.getPrinterName())

        # See if we can extend the range of usable baudrates
        brLimit = self.printerProfile.getBautRateLimit()
        brIndex = self.baudrates.index(brLimit) # xxx todo: catch exception

        if self.minBaudrateIndex != brIndex:

            print("extend baudrate range to ", brLimit, brIndex)

            self.minBaudrateIndex = brIndex

            if brIndex > self.baudrateIndex:
                self.setBaudRate(brIndex)

    # Set a gpio port on printer
    def setGpio(self, pin, value):

        self.sendCommandParamV(CmdSetGpio, [packedvalue.uint8_t(pin), packedvalue.uint8_t(value)])

    def waitForState(self, destState, wait=1, log=False):

        status = self.getStatus()
        if log:
            pprint.pprint(status)

        while status.state != destState:
            time.sleep(wait)
            status = self.getStatus()
            if log:
                pprint.pprint(status)

    def stateMoving(self, status=None):

        if not status:
            status = self.getStatus()

        return status.state == StateStart

    def getTemp(self, doLog = False):

        #
        # CmdGetCurrentTemps returns (bedtemp, T1 [, T2])
        #
        (cmd, payload) = self.query(CmdGetCurrentTemps)
        if len(payload) == 4:
            temps = struct.unpack("<hh", payload)
        else:
            if len(payload) != 6:
                print("WARNING: getTemp(): payload size wrong: %d/12" % len(payload))

            temps = struct.unpack("<hhh", payload)

        return [intmath.fromFWTemp(t) for t in temps]

    def getTemps(self, doLog = False):

        temps = self.getTemp()
        (cmd, payload) = self.query(CmdGetTargetTemps)
        if len(payload) == 3:
            targetTemps = struct.unpack("<hh", payload)
        else:
            targetTemps = struct.unpack("<hhh", payload)

        targetTemps = [intmath.fromFWTemp(t) for t in targetTemps]

        self.gui.tempCb(temps[0], temps[1], targetTemps[1])
        return temps

    ####################################################################################################

    def heatUp(self, heater, temp, wait=None, log=False):

        assert(type(temp) == int)

        # payload = struct.pack("<BH", heater, temp) # Parameters: heater, temp
        # self.sendCommand(CmdSetTargetTemp, binPayload=payload)
        self.setTargetTemp(heater, temp)

        while wait !=  None:
            time.sleep(2)
            temps = self.getTemps()

            if log:
                print("\rTemp: %.2f (%.2f)" % (temps[heater], wait), end=' ')
                sys.stdout.flush()

            if temps[heater] >= wait:
                break

        print("\n")

    ####################################################################################################
    #
    # Slowly increase temperature (from off or a low value) to avoid big overshot,
    # this is some sort *setpoint ramping*.
    #
    # Note: Without ramping, the Ziegler Nichols (0.9) pid settings on the UM2 gives about 40%
    #       overshot when starting from room temperature, the Jennyprinter gives about 35% overshot.
    #
    def heatUpRamp(self, heater, tdest, log=False):

        assert(type(tdest) == int)

        # Ramp up time: tu+tg from the autotune step response.
        timeConstant = self.printerProfile.getTuI() + self.printerProfile.getTgI() 

        status = self.getStatus()
        startTemp = status.t1
        startTime = time.time()

        a = tdest / timeConstant

        print("Start tempramp...", tdest, timeConstant, a)

        if a <= 0:
            self.setTargetTemp(heater, tdest)
            return

        while True:

            status = self.getStatus()
            temp = status.t1

            if temp < tdest-2:

                t = min(round(startTemp + (time.time() - startTime) * a), tdest)

                print("temp is below dest", temp, t, tdest)

                self.setTargetTemp(heater, t)
                yield(temp)

            else:

                # print "temp reached"
                self.setTargetTemp(heater, tdest)
                yield(temp)
                break

    ####################################################################################################
    # Erase mass storage
    def erase(self, nBlocks):

        print("Erasing mass storage ...")

        startTS = time.time()
        if not nBlocks:
            # Entire erase, get nuber of blocks to erase from printer
            (cmd, payload) = self.query(CmdGetCardSize)
            nBlocks = struct.unpack("<i", payload)[0]

        self.sendCommandParamV(CmdErase, (packedvalue.uint32_t(nBlocks), ))

        self.waitForState(StateInit)

        print("Mass storage erase of %d blocks done, time used: %.2fs" % (nBlocks, time.time() - startTS))

    ####################################################################################################
    # Start printing process, and record print start time
    def startPrint(self):

        self.printStartedAt = time.time()
        self.printEndedAt = None
        self.sendCommandParamV(CmdMove, [MoveTypeNormal])

    ####################################################################################################

    def adjTemp(self, heater, adj):

        payload = struct.pack("<Bh", heater, intmath.toFWTemp(adj)) # Parameters: heater, adjtemp
        self.sendCommand(CmdSetIncTemp, binPayload=payload)

    ####################################################################################################

    def setTargetTemp(self, heater, temp):

        payload = struct.pack("<Bh", heater, intmath.toFWTemp(temp)) # Parameters: heater, temp
        self.sendCommand(CmdSetTargetTemp, binPayload=payload)

    ####################################################################################################

    def setTempTable(self, startTemp, nExtrusionLimit, tempTable):

        payload = struct.pack("<hB", startTemp, nExtrusionLimit)

        for timerValue in tempTable:
            # print "TempTable: %d°C, timerValue: %d(0x%x)" % (startTemp, timerValue, timerValue)
            payload += struct.pack("<H", timerValue)
            startTemp += 1

        resp = self.query(CmdSetTempTable, binPayload=payload)
        assert(util.handleGenericResponse(resp))

    ####################################################################################################

    def setTempPWM(self, heater, pwmvalue):

        payload = struct.pack("<Bh", heater, pwmvalue) # Parameters: heater, pwm
        self.sendCommand(CmdSetTempPWM, binPayload=payload)

    ####################################################################################################

    def coolDown(self, heater, temp=0, wait=None, log=False):

        assert(type(temp) == int)

        if log:
            print("coolDown(): cooling down hotend...")

        if heater > HeaterBed:
            # Switch on PID mode
            # payload = struct.pack("<BB", heater, 0)
            # self.sendCommand(CmdSetTempPWM, binPayload=payload)
            self.setTempPWM(heater, 0)

        # payload = struct.pack("<BH", heater, temp)
        # self.sendCommand(CmdSetTargetTemp, binPayload=payload)
        self.setTargetTemp(heater, temp)

        while wait !=  None:
            time.sleep(2)
            temps = self.getTemps()

            if log:
                print("\rTemp: %.2f (%.2f)" % (temps[heater], wait), end=' ')
                sys.stdout.flush()

            if temps[heater] <= wait:
                break

        if log:
            print("")

    ####################################################################################################

    def isHomed(self):

        (cmd, payload) = self.query(CmdGetHomed)
        return struct.unpack("<B", payload)[0] == 1

    ####################################################################################################

    def getPos(self):

        (cmd, payload) = self.query(CmdGetPos)
        tup = struct.unpack("<iiii", payload)
        return tup

    ####################################################################################################

    def getEndstops(self):

        # Check, if enstop was pressed
        (cmd, payload) = self.query(CmdGetEndstops)
        tup = struct.unpack("<BiBiBiBi", payload)
        return tup

    ####################################################################################################

    def endStopTriggered(self, dim, fakeHomingEndstops=False):

        # Check, if enstop was pressed
        tup = self.getEndstops()

        if tup[dim*2] or fakeHomingEndstops:
            print("Endstop %s hit at position: %d" % (dimNames[dim], tup[dim*2+1]))
            return True

        print("Endstop %s open at position: %d" % (dimNames[dim], tup[dim*2+1]))
        return False

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

        for i in range(len(payload) // 4):
            readings.append( struct.unpack("<Hh", payload[i*4:(i+1)*4]) )

        return readings

    ####################################################################################################

    # Get current stepper direction bits
    def getDirBits(self):

        (cmd, payload) = self.query(CmdGetDirBits)
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































































































































































































































































