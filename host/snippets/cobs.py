# -*- coding: utf-8 -*-

import sys, struct, crc16

LenLen = 2

# initiale implementierung:
#
# Paketgrösse insgesamt max. 256 bytes
# [     B 0x1, SOH
#       B ..., Command counter [1...255]
#       B ..., Commandbyte [1...255]
#       H ..., länge payload "n" PLUS 1 -> N = n+1, n = N-1
LenHeader =          1+1+1+LenLen+1+2
nmax =               256 - LenHeader # = 256 - 8 = 248
#   n * B ..., Payload, aufgeteilt in COBS codeblocks
#       B ..., Checksum code
#       H ..., Checksum 16 bit
#
#
#
#
# nmax = 248

# kodierung checksum, checksum code
#
# 0x1: (x, y) -> (x, y)
# 0x2: (0, y) -> (1, y)
# 0x3: (x, 0) -> (x, 1)
# 0x4: (0, 0) -> (1, 1)
#


# Optimiert:
#
# Paketgrösse insgesamt max. 256 bytes
# [ B 0x0, SOH
#   B    , anz. lbytes länge [1..4]
#   
#

nullByte = chr(0)

def encodeCobs(data, maxlen):

    pos = 0
    cobsBody = ""
    cobsResult = ""
    datalen = maxlen
    while pos < maxlen:

        if data[pos] == nullByte:
            # print "found 0 at", pos, len(cobsBody)
            cobsResult += chr(len(cobsBody)+1)
            cobsResult += cobsBody
            cobsBody = ""

            # if pos == maxlen-1:
                # print "last byte is a nullbyte, good"
        else:
            if pos == maxlen-1:
                # Letzes byte, add ONE byte overhead
                # print "last byte is not a nullbyte, adding 0xff codeblock"
                cobsResult += chr(0xff)
                cobsResult += cobsBody
                if pos < (nmax-1):
                    cobsResult += data[pos]
                else:
                    datalen -= 1
            else:
                cobsBody += data[pos]

        pos += 1

    return (cobsResult, datalen)

def encodePacket(linenr, cmd, packetSize, payload):

    assert(linenr > 0)
    assert(cmd > 0)
    
    result = nullByte

    header = struct.pack("<BBH", linenr, cmd, packetSize+1)

    result += header
    checksum = crc16.crc16xmodem(header)

    result += payload
    checksum = crc16.crc16xmodem(payload, checksum)

    cflags = 0x1
    # print "cflags, c1: 0x%x, 0x%x" % (cflags, checksum)

    if checksum == 0:
        cflags = 0x4
        checksum = 0x0101
    elif (checksum & 0xFF00) == 0:
        cflags = 0x2
        checksum = 0x0100 | (checksum & 0xFf)
    elif (checksum & 0x00FF) == 0:
        cflags = 0x3
        checksum = 0x0001 | (checksum & 0xFF00)

    # print "c1 adjusted: 0x%x" % checksum

    result += struct.pack("<BH", cflags, checksum)

    return result

out = open("/tmp/cobs.out", "w")

def decodePacket(packet):

    ofs = 0

    assert(packet[ofs] == nullByte)
    ofs += 1

    line = ord(packet[ofs])
    ofs += 1

    cmd = ord(packet[ofs])
    ofs += 1

    # n = ord(packet[3]) - 1
    n = struct.unpack("<H", packet[ofs:ofs+LenLen])[0] - 1
    ofs += LenLen

    # print "line, cmd, n:", line, cmd, n

    result = packet[ofs:ofs+n]
    ofs += n

    cflags = ord(packet[ofs])
    ofs += 1

    # c1 = crc16.crc16xmodem(packet[1:n+LenHeader-4])
    c1 = crc16.crc16xmodem(packet[1:-3])

    (c2, ) = struct.unpack("<H", packet[ofs:ofs+2])

    assert (cflags in [0x1, 0x2, 0x3, 0x4])

    # print "cflags, c1, c2: 0x%x, 0x%x, 0x%x" % (cflags, c1, c2)

    # xxx
    if cflags == 0x2:
        c2 = c2 - 0x0100;
    elif cflags == 0x3:
        c2 = c2 - 0x1;
    elif cflags == 0x4:
        c2 = c2 - 0x0101;

    # print "c2 adjusted: 0x%x" % c2

    assert(c1 == c2)

    return (line, cmd, n, result)

def decodeCobs(data, l):

    result = ""

    pos = 0
    while pos < l:
        cobsCode = ord(data[pos])
        pos += 1
        # if cobsCode == 0xff:
            # cobsCode = (l - pos) + 1
        for i in range(cobsCode-1):
            if pos < l:
                result += data[pos]
                pos += 1
            else:
                break
        if cobsCode != 0xff:
            result += nullByte

    return result

blob = sys.stdin.read(nmax)

size = len(blob)
start = 0

linenr = 1
cmd = 1

while size:

    # print

    packetSize = min(size, nmax)

    # packet = encodeCobs(linenr, cmd, blob[start:start+packetSize], packetSize)
    (cobs, datalen) = encodeCobs(blob[start:start+packetSize], packetSize)
    cl = len(cobs)
    # print "len cobs block: ", cl, "datalen: ", datalen
    assert(cl <= nmax)

    packet = encodePacket(linenr, cmd, cl, cobs)
    cp = len(packet)
    # print "len cobs packet: ", cp
    assert(cp <= 256)

    # decode
    (line, c, l, data) = decodePacket(packet)
    assert(line == linenr)
    assert(c == cmd)

    # print "cobs len: ", cl, l, len(data)
    assert(len(data) == cl)

    encodedBlob = decodeCobs(data, l)

    # print "len blob, encodedBlob:", datalen, len(encodedBlob)
    assert(blob[:datalen] == encodedBlob)

    out.write(encodedBlob)
    # end decode

    cmd = 2
    linenr = ((linenr+1) % 256) + 1

    if datalen < packetSize:
        blob = blob[-1] + sys.stdin.read(nmax-1)
    else:
        blob = sys.stdin.read(nmax)

    size = len(blob)





