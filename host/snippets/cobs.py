# -*- coding: utf-8 -*-

import sys, struct, crc16, cStringIO

LenLen = 2

# initiale implementierung:
#
# Paketgrösse insgesamt max. 256 bytes
# [     B 0x1, SOH
#       B ..., Command counter [1...255]
#       B ..., Commandbyte [1...255]
#       H ..., länge payload "n" PLUS 0x101 -> N = n+0x101, n = N-0x101
LenHeader =          1+1+1+LenLen+1+2
LenCobs =               256 - LenHeader # = 256 - 8 = 248
#   n * B ..., Payload, aufgeteilt in COBS codeblocks
#       B ..., Checksum code
#       H ..., Checksum 16 bit
#
#
#
#
# LenCobs = 248

# kodierung checksum, checksum code
#
# 0x1: (x, y) -> (x, y)
# 0x2: (0, y) -> (1, y)
# 0x3: (x, 0) -> (x, 1)
# 0x4: (0, 0) -> (1, 1)
#

nullByte = chr(0)

def xencodeCobs(stream, blockLen=LenCobs):

    fpos = stream.tell()
    stream.seek(fpos+blockLen-1)

    lastByte = stream.read(1)
    part = False

    stream.seek(fpos)

    if not lastByte:

        assert(0)

        data = stream.read()

        size = len(data)
        if size == 0:
            return None

        # lastByte = data[-1]
        # if lastByte != nullByte:
            # size = len(data)-1

        part = True

    else:

        size = blockLen

        if lastByte != nullByte:
            size = blockLen-1

        data = stream.read(size)

    cobsBody = ""
    cobsResult = ""
    for pos in range(size):

        if data[pos] == nullByte:
            # print "found 0 at", pos, len(cobsBody)
            cobsResult += chr(len(cobsBody)+1)
            cobsResult += cobsBody
            cobsBody = ""

            if pos == size-1:
                print "last byte is a nullbyte"
        else:
            cobsBody += data[pos]
            if pos == size-1:
                # Letzes byte, add ONE byte overhead
                print "last byte is not a nullbyte, adding 0xff codeblock"
                # cobsResult += chr(0xff)
                cobsResult += chr(len(cobsBody)+1)
                cobsResult += cobsBody

        # print "cobs: %s" % cobsResult.encode("hex")

    # xxx debug
    if not part and len(cobsResult) != blockLen:
        print "Error, cobs encoded block %d bytes instead of %d." % (len(cobsResult), blockLen)
        print "data: ", len(data), data.encode("hex")
        print "cobs: ", len(cobsResult), cobsResult.encode("hex")
        assert(0)

    return cobsResult

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
        print "long packet ending in 0 -> result is long, no overhead"

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
        print "short packet ends with 0 -> result is short, no overhead"

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

    print "short packet does not end with 0 -> result is long, one byte overhead"

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

    assert(0)

    if data[-1] != nullByte:
        size += 1


    for pos in range(size):

        if pos==size-1 or data[pos] == nullByte:
            # print "found 0 at", pos, len(cobsBody)
            cobsResult += chr(len(cobsBody)+1)
            cobsResult += cobsBody
            cobsBody = ""
        else:
            cobsBody += data[pos]

    # xxx debug
    # if len(cobsResult) != len(data)+1:
        # print "Error, cobs encoded block %d bytes instead of %d." % (len(cobsResult), blockLen)
        # print "data: ", len(data), data.encode("hex")
        # print "cobs: ", len(cobsResult), cobsResult.encode("hex")
        # assert(0)

    return cobsResult

def encodeCobsString(s, blockLen=LenCobs):
    s = cStringIO.StringIO(s)
    s = encodeCobs(s, blockLen)
    # print "encoded: %s" % s.encode("hex")
    return s

def encodePacket(linenr, cmd, payload):

    assert(linenr > 0)
    assert(cmd > 0)
    
    result = nullByte

    header = struct.pack("<BBH", linenr, cmd, len(payload)+0x101)

    result += header
    checksum = crc16.crc16xmodem(header)

    result += payload
    checksum = crc16.crc16xmodem(payload, checksum)

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

    # print "c1 adjusted: 0x%x" % checksum

    result += struct.pack("<BH", cflags, checksum)

    return result

def decodePacket(packet):

    ofs = 0

    assert(packet[ofs] == nullByte)
    ofs += 1

    line = ord(packet[ofs])
    ofs += 1

    cmd = ord(packet[ofs])
    ofs += 1

    # n = ord(packet[3]) - 1
    n = struct.unpack("<H", packet[ofs:ofs+LenLen])[0] - 0x101
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

def decodeCobs(data):

    result = ""

    pos = 0
    while pos < len(data):
        cobsCode = ord(data[pos])
        pos += 1
        for i in range(cobsCode-1):
            if pos < len(data):
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

# blob = sys.stdin.read(10000)

# size = len(blob)
# start = 0

if __name__ == "__main__":

    # Case 1, long packet, ends with 0
    s = (LenCobs-1)*" " + nullByte
    stream = cStringIO.StringIO(s)

    cobs = encodeCobs(stream, LenCobs)
    assert(len(cobs) == LenCobs)
    decodedBlob = decodeCobs(cobs)

    assert(s == decodedBlob)

    # Case 2, short packet, ends with 0
    s = (LenCobs-2)*" " + nullByte + " "
    stream = cStringIO.StringIO(s)

    cobs = encodeCobs(stream, LenCobs)
    assert(len(cobs) == LenCobs-1)
    decodedBlob = decodeCobs(cobs)

    assert(s[:-1] == decodedBlob)

    # Case 3, short packet, does not end with 0
    s = (LenCobs)*" "
    stream = cStringIO.StringIO(s)

    cobs = encodeCobs(stream, LenCobs)
    assert(len(cobs) == LenCobs)
    decodedBlob = decodeCobs(cobs)

    # print "s   : ", len(s), s.encode("hex")
    # print "deco: ", len(decodedBlob), decodedBlob.encode("hex")

    #######################
    # send:  000508f901029f03a80f010101010101010101010101010101010103aa04010253ffaa0476042304e003a9037a0352032e030f03f302da02c302af029c028a027a026b025d025002440239022e0223021a0211020802ff01f801f001e901e201db01d501cf01c901c301be01b801b301ae01aa01a501a1019c019801940190018c018901850181017e017b017701740171016e016b0168016601630160015e015b0159015601540151014f014d014b01480146014401420140013e013c013a013901370135013301310130012e012c012b0129012801260124012301220120011f011d011c011a01190118011601150114011301110110010f010e010d016357
    # send:  000606f80118010b010a010901080107010601050104010301020101010301ff02fe02fd02fc02fb02fa02f902f802f702f602f502f402f302f202f202f102f002ef02ee02ed02ed02ec02eb02ea02ea02e902e802e702e602e602e502e402e402e302e202e102e102e002df02df02de02dd02dd02dc02db02db02da02da02d902d802d802d702d702d602d502d502d402d402d302d202d202d102d102d002d002cf02cf02ce02ce02cd02cd02cc02cb02cb02ca02ca02c902c902c902c802c802c702c702c602c602c502c502c402c402c302c302c202c202c202c102c102c002c002bf02bf02bf02be02be02bd02bd02bc02bc02bc02bb02bb02bb015532
    # send:  000706f90102ba02ba02b902b902b902b802b802b702b702b702b602b602b602b502b502b502b402b402b302b302b302b202b202b202b102b102b102b002b002b002af02af02af02ae02ae02ae02ad02ad02ad02ad02ac02ac02ac02ab02ab02ab02aa02aa02aa02aa02a902a902a902a802a802a802a702a702a702a702a602a602a602a602a502a502a502a402a402a402a402a302a302a302a302a202a202a202a202a102a102a102a102a002a002a002a0029f029f029f029f029e029e029e029e029d029d029d029d029c029c029c029c029c029b029b029b029b029a029a029a029a0299029902990299029902980298029802980298029702970125be

    s="18010b010a010901080107010601050104010301020101010301ff02fe02fd02fc02fb02fa02f902f802f702f602f502f402f302f202f202f102f002ef02ee02ed02ed02ec02eb02ea02ea02e902e802e702e602e602e502e402e402e302e202e102e102e002df02df02de02dd02dd02dc02db02db02da02da02d902d802d802d702d702d602d502d502d402d402d302d202d202d102d102d002d002cf02cf02ce02ce02cd02cd02cc02cb02cb02ca02ca02c902c902c902c802c802c702c702c602c602c502c502c402c402c302c302c202c202c202c102c102c002c002bf02bf02bf02be02be02bd02bd02bc02bc02bc02bb02bb02bb"
    decodedBlob = decodeCobs(s.decode("hex"))

    print "s :", s
    print "sd:", decodedBlob.encode("hex")

    s="02ba02ba02b902b902b902b802b802b702b702b702b602b602b602b502b502b502b402b402b302b302b302b202b202b202b102b102b102b002b002b002af02af02af02ae02ae02ae02ad02ad02ad02ad02ac02ac02ac02ab02ab02ab02aa02aa02aa02aa02a902a902a902a802a802a802a702a702a702a702a602a602a602a602a502a502a502a402a402a402a402a302a302a302a302a202a202a202a202a102a102a102a102a002a002a002a0029f029f029f029f029e029e029e029e029d029d029d029d029c029c029c029c029c029b029b029b029b029a029a029a029a029902990299029902990298029802980298029802970297"
    decodedBlob = decodeCobs(s.decode("hex"))

    print "s :", s
    print "sd:", decodedBlob.encode("hex")

    #######################
    assert(s[:-1] == decodedBlob)

    linenr = 1
    cmd = 1
    inp = open(sys.argv[1])
    out = open("/tmp/cobs.out", "w")

    while True:

        cobs = encodeCobs(inp, LenCobs)

        if not cobs:
            # done
            break

        cl = len(cobs)
        # print "len cobs block: ", cl
        assert(cl <= LenCobs)

        packet = encodePacket(linenr, cmd, cobs)
        cp = len(packet)
        # print "len entire packet: ", cp
        assert(cp <= 256)

        # decode
        (line, c, l, data) = decodePacket(packet)
        assert(line == linenr)
        assert(c == cmd)

        # print "cobs len: ", cl, l, len(data)
        assert(len(data) == cl)

        decodedBlob = decodeCobs(data)

        assert(len(decodedBlob) <= LenCobs)

        # print "len decodedBlob:", len(decodedBlob)

        out.write(decodedBlob)
        # end decode

        cmd = 2
        linenr = ((linenr+1) % 256) + 1











