# -*- coding: utf-8 -*-
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

import sys, struct, crc16, cStringIO, zlib

LenLen    = 1
LenHeader = 1+1+1+LenLen+1+2
# Rxbuffer in firmare is 256 bytes, but for simpler index handling
# we use only 255 bytes of it.
LenCobs   = 255 - LenHeader

#
# Paketgrösse insgesamt max. 256 bytes
# [
#       B 0x1, SOH
#       B ..., Command counter [1...255]
#       B ..., Commandbyte [1...255]
#       B ..., länge payload "n" PLUS 0x01 -> N = n+0x101, n = N-0x101
#   n * B ..., Payload, aufgeteilt in COBS codeblocks
#       B ..., Checksum code
#       H ..., Checksum 16 bit
# ]
#
#
#

#
# Kodierung checksum, checksum code (bytes dürfen nicht 0 sein):
#
# 0x1: high und lowbyte sind != 0: (x, y) -> (x, y)
# 0x2: highbyte ist 0            : (0, y) -> (1, y)
# 0x3: lowbyte ist 0             : (x, 0) -> (x, 1)
# 0x4: high und lowbyte sind 0   : (0, 0) -> (1, 1)
#

nullByte = chr(0)

#
# Todo: check cobs-R for more optimization:
# https://pythonhosted.org/cobs/cobsr-intro.html, https://pypi.org/project/cobs/
#
def encodeCobs_cmd_packed(cmd, packedCmd, stream, blockLen=LenCobs):

    return (cmd, encodeCobsNoPack(stream, blockLen))

    cobsBody = ""
    cobsResult = ""

    fpos = stream.tell()

    rawdata = stream.read(blockLen)
    if not rawdata:
        return (cmd, None)

    compressor = zlib.compressobj(9, zlib.DEFLATED, -15)
    compressor.compress(rawdata)
    data = compressor.flush()
    # print "encodeCobs_cmd_packed compressed %d blocksize into %d bytes..." % (len(rawdata), len(data))

    if len(data) > len(rawdata)*0.95:
        # xxx ugly, do this compressible test in move.py...
        stream.seek(fpos)
        return (cmd, encodeCobsNoPack(stream, blockLen))

    # xxx waste one byte if lastbyte is not 0x0
    if data[-1] != nullByte:
        data += nullByte

    for c in data:
        if c == nullByte:
            cobsResult += chr(len(cobsBody)+1)
            cobsResult += cobsBody
            cobsBody = ""
        else:
            cobsBody += c

    assert(len(cobsResult) <= blockLen)
    assert(len(cobsResult) == len(data))
    return (packedCmd, cobsResult)

def encodeCobsNoPack(stream, blockLen=LenCobs):

    cobsBody = ""
    cobsResult = ""

    data = stream.read(blockLen-1)
    if not data:
        return None

    fpos = stream.tell()
    lastByte2 = stream.read(1)

    lastByte1 = data[-1]

    if lastByte2 == nullByte:

        # print "Cobs packet ends in 0 -> no overhead"

        data += lastByte2
        for c in data:
            if c == nullByte:
                cobsResult += chr(len(cobsBody)+1)
                cobsResult += cobsBody
                cobsBody = ""
            else:
                cobsBody += c

        assert(len(cobsResult) <= blockLen)
        assert(len(cobsResult) == len(data))
        return cobsResult

    # short block, push back lastByte2
    stream.seek(fpos)

    if lastByte1 == nullByte:

        # print "Cobs packet ends with 0 -> no overhead"

        for c in data:
            if c == nullByte:
                cobsResult += chr(len(cobsBody)+1)
                cobsResult += cobsBody
                cobsBody = ""
            else:
                cobsBody += c

        assert(len(cobsResult) <= blockLen-1)
        assert(len(cobsResult) == len(data))
        return cobsResult

    # print "Cobs packet does not end with 0 -> one byte overhead"

    size = len(data)+1
    for pos in range(size):

        if pos==size-1:
            cobsResult += chr(0xff)
            cobsResult += cobsBody
        elif data[pos] == nullByte:
            # print "found 0 at", pos, len(cobsBody)
            cobsResult += chr(len(cobsBody)+1)
            cobsResult += cobsBody
            cobsBody = ""
        else:
            cobsBody += data[pos]

    assert(len(cobsResult) <= blockLen)
    assert(len(cobsResult) == len(data)+1)
    return cobsResult

def encodeCobsString(s, blockLen=LenCobs):
    s = cStringIO.StringIO(s)
    return encodeCobsNoPack(s, blockLen)

def decodeCobs(data):

    result = ""

    pos = 0
    l = len(data)
    while pos < l:
        cobsCode = ord(data[pos])
        pos += 1
        for i in range(cobsCode-1):
            if pos < l:
                result += data[pos]
                pos += 1
            else:
                # break
                return result

        # if pos < l:
            # result += nullByte
        # if cobsCode != 0xff:
        result += nullByte

    return result


