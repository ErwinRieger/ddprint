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

import sys, struct, crc16, zlib

LenLen    = 1
LenHeader = 1+1+1+LenLen+1+2

MaxCobsBlock = 254

# Size of sectors of printer mass-storage
SectorSize = 512

#####################################################################

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
def encodeCobs_cmd_packed(cmd, packedCmd, stream):

# xxx debug
    print "XXXX no zip in encodeCobs_cmd_packed"
    return (cmd, encodeCobs512(stream))

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
        return (cmd, encodeCobs512(stream)) # , blockLen))

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

# Encode up to 512 bytes payload
def encodeCobs512(payload):

    assert(len(payload) <= SectorSize)

    cobsBody = ""
    cobsResult = ""
    # lastIsZero = payload[-1] == nullByte

    lCobsBody = 0
    for c in payload:
        if c == nullByte:
            cobsResult += chr(len(cobsBody)+1)
            cobsResult += cobsBody
            cobsBody = ""
            lCobsBody = 0
        else:
            cobsBody += c
            lCobsBody += 1

            if lCobsBody == MaxCobsBlock:
                cobsResult += chr(0xff) # Special max. length block
                cobsResult += cobsBody
                cobsBody = ""
                lCobsBody = 0

    # Handle "no null at end" case
    if cobsBody:
        cobsResult += chr(len(cobsBody)+1)
        cobsResult += cobsBody

    # print "cobs    :", len(cobsResult), cobsResult.encode("hex")

    # print "encodeCobs512(): todo remove decode-test!"
    # (deco, _) = decodeCobs512(cobsResult)
    # assert(deco == payload)

    return cobsResult

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

def decodeCobs512(data):

    assert(len(data) >= 512)

    result = ""
    pos = 0
    bytesRead = 0
    bytesWritten = 0

    while bytesWritten < SectorSize:

        cobsCode = ord(data[pos])
        bytesRead += 1
        pos += 1

        for i in range(cobsCode-1):

            assert(bytesWritten < SectorSize)# if bytesWritten < SectorSize:
            result += data[pos]
            bytesRead += 1
            bytesWritten +=1
            pos += 1

        # Length of last cobs subblock points
        # intentionally beyond the end
        if bytesWritten < SectorSize and cobsCode != 0xff:
            result += nullByte
            bytesWritten +=1

    return (result, bytesRead)


