# -*- coding: utf-8 -*-

import sys, struct, crc16, cStringIO

LenLen = 1
LenHeader =          1+1+1+LenLen+1+2
# Rxbuffer in firmare is 256 bytes, but for simpler index handling
# we use only 255 bytes of it.
LenCobs =               255 - LenHeader

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

def encodeCobs(stream, blockLen=LenCobs):

    cobsBody = ""
    cobsResult = ""

    data = stream.read(blockLen-1)
    if not data:
        return None

    fpos = stream.tell()

    lastByte1 = data[-1]

    lastByte2 = stream.read(1)

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
    return encodeCobs(s, blockLen)

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

















