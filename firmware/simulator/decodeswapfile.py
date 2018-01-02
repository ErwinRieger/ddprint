
import sys, struct

from ddprintcommands import *


# * Startbyte (SOH, 0x81)
# * Command counter 8 bit 0
# * Commandbyte         buffered (print) commands (G1...), direct commands (Temp, start, stop)
#    + commands < 128: buffered commands
#    + commands >= 128: direct commands
# * Length of command packet 16 bits, this includes the length of the variable command payload 
# * if length > 0: command specific parameters/data
# * checksum 16 bits
#
# Speicherung eines buffered commands:
#   - entferne startbye, commandcounter am anfang
#   - entferne checksum am ende
#   - speichere das kommando
#
#
# Move segment data:
#
#       + index lead axis, 8 bits
#       + array of absolute steps, 5 * 32 bits
#       + number of accel steps, naccel, 16 bits
#       + constant linear timer value, 16 bits
#       + number of deccel steps, ndeccel, 16 bits
#

readPos = 0
def decodeUInt8(inFile):
    global readPos
    readPos += 1
    return struct.unpack("<B", inFile.read(1))[0]

def decodeUInt16(inFile):
    global readPos
    readPos += 2
    return struct.unpack("<H", inFile.read(2))[0]

def decodeUInt32(inFile):
    global readPos
    readPos += 4
    return struct.unpack("<I", inFile.read(4))[0]

inFile = open(sys.argv[1])

while True:

    print "################# cmd: ##################\n"

    c = inFile.read(1)

    if c == "":
        print "eof reached ... exit"
        sys.exit(0)

    cmd = ord(c)
    print "Read commandbyte 0x%x at pos: %d (0x%x)" % (cmd, readPos, readPos)

    readPos += 1

    # l = decodeUInt32(inFile)
    # l = decodeUInt8(inFile)
    # print "Packet Len read: ", l

    ll = 0
    # if cmd == CmdDirG1:
    if cmd == CmdG1:
        # print "DirG1"
        print "CmdG1 at pos, ", readPos

        # dirbits = decodeUInt8(inFile)
        # print "dirbits: 0x%x" % dirbits
        # ll += 1
        # cmd = CmdG1

    # if cmd == CmdG1:

        print "G1"

        #       + flags, 16 bits
        #       + index lead axis, 8 bits
        #       + array of absolute steps, 5 * 32 bits
        #       + number of accel steps, naccel, 16 bits
        #       + E-Timer start move: 16 bits
        # [ #       + [lead factor] ]
        #       + constant linear timer value, 16 bits
        #       + number of deccel steps, ndeccel, 16 bits
        # HBIIIIIHHH

        flags = decodeUInt16(inFile)
        print "Flags: 0x%x" % flags
        ll += 2

        print "DirBitsBit: 0x%x" % (flags & DirBitsBit)
        print "dirbits: 0x%x" % (flags & 0x1f)
        print "AccelByteFlagBit: 0x%x" % (flags & AccelByteFlagBit)
        print "DecelByteFlagBit: 0x%x" % (flags & DecelByteFlagBit)
        print "MoveStartBit: 0x%x" % (flags & MoveStartBit)

        lead = decodeUInt8(inFile)
        print "lead axis:", lead
        assert(lead >= 0 and lead <= 4)
        ll += 1

        abssteps = []
        for i in range(5):
            nsteps = decodeUInt32(inFile)
            print "Abssteps: ", i, nsteps
            ll += 4
            abssteps.append(nsteps)

        na = decodeUInt16(inFile)
        print "n-accel:", na
        # ll += 2 + na*2
        ll += 2

        et = decodeUInt16(inFile)
        print "start E-Timer:", et
        ll += 2

        # lf = decodeUInt16(inFile)
        # print "lead-factor:", lf

        lt = decodeUInt16(inFile)
        print "timer lin: ", lt
        assert(lt >= 20);
        ll += 2

        # nl = decodeUInt8(inFile)
        # print "n-deccelloop:", nl
        # ll += 1

        nd = decodeUInt16(inFile)
        print "n-deccel:", nd
        ll += 2 + nd*2

        # print "ll: ", l, ll
        # assert(l == ll)
        # if l != ll:
            # print "Packet length doesn't match data length!", l, ll

        tn = None

        if flags & AccelByteFlagBit:
            assert(0)
        else:
            for i in range(na):
                tn1 = decodeUInt16(inFile)
            
                # print "accel timer:", tn1
                if tn1 < 50:
                    print " t < 50: ", tn1, "readpos: 0x%x" % readPos
                    assert(0)

                if tn != None and tn < tn1:
                    print " accel ramp error: t(%d) < t(n+1): 0x%x 0x%x" % (i, tn, tn1)
                    print " readpos: 0x%x" % readPos
                    assert(tn1 <= tn)
                tn = tn1

        if na:
            assert(lt <= tn1)

        tn = None

        if flags & DecelByteFlagBit:
            assert(0)
        else:
            for i in range(nd):
                tn1 = decodeUInt16(inFile)

                if tn1 < 50:
                    print " t < 50: ", tn1, "readpos: 0x%x" % readPos
                    assert(0)

                if tn != None and tn > tn1:
                    print " deccel ramp error: t(%d) < t(n+1): 0x%x 0x%x" % (i, tn, tn1)
                    print " readpos: 0x%x" % readPos
                    assert(tn1 >= tn)
                tn = tn1

    # elif cmd == CmdDirBits:
        # bits = ord(inFile.read(1))
        # readPos += 1
        # print "Dir 0x%x" % bits
        # assert(l == 1)

    elif cmd == CmdFanSpeed:
        speed = ord(inFile.read(1))
        readPos += 1
        print "Fan speed: 0x%x" % speed
        assert(l == 1)
    else:
        print "\nUNKNOWN command or end of data: 0x%x at pos: \n" % cmd, readPos
        assert(0)

sys.exit(0)

n = 0
for line in inFile.readlines():

    gcode = ultiprint.packGCode(line.strip(), n)

    if gcode:
        lines.append(gcode)

        if n < 0x10000:
            c = gcode[:-4] + "\n"
        else:
            c = gcode[:-6] + "\n"

        print "Packed: ", c.encode("hex")
        outFile.write(c)

    elif line.strip():
        lines.append(line)
        outFile.write(line)

    n += 1


outFile.close()
sys.exit(1)

outFile = open("/tmp/packedgcode_decode.g", "w")

n = 0
for line in lines:

    if line.strip() and ord(line[0]) < 59:
        gcode = decodeGCode(line, n)
        outFile.write(gcode + "\n")
    else:
        outFile.write(line + "\n")

    n += 1


