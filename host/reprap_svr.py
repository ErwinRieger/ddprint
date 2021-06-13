
import os, pty, serial, time, sys



class ReprapServer:

    def __init__(self, gui):


        (self.master, slave) = pty.openpty()
        pty.tty.B115200
        self.s_name = os.ttyname(slave)
        self.ser = serial.Serial(self.s_name)

        self.gui = gui

        self.t0 = self.t1 = self.t0target = self.t1target = 0

        self.selectedFile = None

    def setTemps(self, t0, t0target, t1, t1target):

        self.t0 = t0
        self.t0target = t0target
        self.t1 = t1
        self.t1target = t1target

    def setStatus(self, status):

        self.printerState = status["state"]
        self.setTemps(status["t0"], status["targetT1"], status["t1"], self.t1target)

        # self.curPWM.set_value( "%8d" % status["pwmOutput"])
        # self.swapSize.set_value( "%8s" % util.sizeof_fmt(status["Swap"]))
        # self.sdrSize.set_value( "%8s" % util.sizeof_fmt(status["SDReader"]))
        # self.sbSisze.set_value( "%8s" % util.sizeof_fmt(status["StepBuffer"]))
        # if status["StepBufUnderRuns"] > 0:
            # self.underrun.set_value( "%8s" % str(status["StepBufUnderRuns"]))
        # else:
            # self.underrun.set_value( "       0" )

    def run(self):

        self.gui.log( "ReprapServer.run()" )
        self.gui.log( "Opened tty usb-serial at:", self.s_name )

        while True:

            req = os.read(self.master,1000)
            self.gui.log( "ReprapServer: read request:", req )

            resp = "ok"

            split = req.replace("*", " ").split()

            if split and split[0][0] == "N":
                del split[0]

            if not split:
                os.write(self.master, "ok\n")
                self.ser.flush()
                continue

            cmd = split[0]
            if cmd == "M110":
                pass
            elif cmd == "M111":
                pass
            elif cmd == "M104":
                # M104: Set Extruder Temperature
                # M104 S134
                self.gui.log( "Ignoring set temp command:", req )
                pass
            elif cmd == "M115":
                # M115: Get Firmware Version and Capabilities
                # resp = "ok FIRMWARE_NAME:ddPrint-Firmware"
                resp = "ok FIRMWARE_NAME:ddPrint-Firmware FIRMWARE_URL:https://github.com/ErwinRieger/ddprint PROTOCOL_VERSION:1.0 MACHINE_TYPE:ddPrinter EXTRUDER_COUNT:1"
            elif cmd == "M20":
                resp = "Begin file list:\n"
                resp += " SQUARE.G 1234 SQUARE.G\n"
                resp += " ZCARRI~2.GCO 234 ZCARRIAGE_V2.GCO\n"
                resp += " End file list \n"
                resp += "ok\n"
            elif cmd == "M21":
                # M21: Initialize SD card
                resp = "SD card ok\nok"
            elif cmd == "M23":
                # M23: Select SD file
                self.selectedFile = split[1]
                print "file %s selected for printing..." % self.selectedFile
                resp = "File opened:%s Size:%d" % (self.selectedFile, 1234)
                resp += "\nFile Selected\nok"
            elif cmd == "M27":
                # M27: Report SD print status
                resp = "SD printing byte 123/1234\nok"
                # resp = "Not SD printing."
            elif cmd == "M105":
                # M105: Get Extruder Temperature
                resp = "ok T:%.1f /%.1f B:%.1f /%.1f" % (self.t1, self.t1target, self.t0, self.t0target)
            elif cmd == "M140":
                # Set Bed Temperature (Fast)
                pass
            else:
                self.gui.log( "unknown request: ", req )
                sys.exit(0)

            self.gui.log( "ReprapServer: write resp:", resp )
            os.write(self.master, resp+"\n")
            self.ser.flush()
            time.sleep(0.1)


