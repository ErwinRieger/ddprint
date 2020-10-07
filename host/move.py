# -*- coding: utf-8 -*-
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

import math, struct # , zlib

import ddprintcommands, cobs, cStringIO, ddprintutil

from ddprintconstants import maxTimerValue16, fTimer, _MAX_ACCELERATION, X_AXIS, Y_AXIS, A_AXIS
from ddprintconstants import AdvanceEThreshold, StepDataTypeBresenham, StepDataTypeRaw
from ddvector import Vector, VelocityVector32, VelocityVector5, vectorLength, vectorSub, vectorAbs
from ddprintcommands import CommandNames, DecelByteFlagBit, AccelByteFlagBit, DirBitsBit, DirBitsBitRaw, MoveStartBit
from ddprintcommands import MeasureStartBit, MeasureStartBitRaw, EndMoveBit, EndMoveBitRaw
from ddprintcommands import TimerByteFlagBit, MoveStartBitRaw
from ddprofile import PrinterProfile
from types import ListType

from ddconfig import *

import lz4.block, zlib


####################################################################################################
####################################################################################################
####################################################################################################
####################################################################################################
####################################################################################################
####################################################################################################

## #Julian Serra: jserra17@cmc.edu
## #LZ77

import struct
import math

def LZ77_search(search, look_ahead):

  ls = len(search)
  llh = len(look_ahead)
 
  if(ls==0):
   return (0,0, look_ahead[0])
  
  if(llh)==0:
   return (-1,-1,"")

  best_length=0
  best_offset=0 
  buf = search + look_ahead

  search_pointer = ls 
  #print( "search: " , search, " lookahead: ", look_ahead)
  for i in range(0,ls):
   length = 0
   while buf[i+length] == buf[search_pointer +length]:
    length = length + 1
    if search_pointer+length == len(buf):
     length = length - 1
     break
    if i+length >= search_pointer:
     break  
   if length > best_length:
    best_offset = i
    best_length = length

  return (best_offset, best_length, buf[search_pointer+best_length])



def encode(input):

    x = 20
    MAXSEARCH = 512
    MAXLH =  int(math.pow(2, (x  - (math.log(MAXSEARCH, 2))))) 

    searchiterator = 0;
    lhiterator = 0;

    output = ""


    while lhiterator<len(input):

        search = input[searchiterator:lhiterator]
        look_ahead = input[lhiterator:lhiterator+MAXLH]
        (offset, length, char) = LZ77_search(search, look_ahead)
        #print (offset, length, char)
  
        shifted_offset = offset << 6
        offset_and_length = shifted_offset+length 
        ol_bytes = struct.pack(">Hc",offset_and_length,char)  

        # file.write(ol_bytes) 
        output += ol_bytes

        lhiterator = lhiterator + length+1
        searchiterator = lhiterator - MAXSEARCH

        if searchiterator<0:
            searchiterator=0

    return output


# input = "HI fans!HI fans!HI fans!HI fans!HI fans!HI fans!HI fans!HI fans!HI fans!HI fans!HI fans!"
# output = encode(input)
# print "decoded:", len(input), len(output), output,

 
####################################################################################################
####################################################################################################
####################################################################################################
####################################################################################################
####################################################################################################
####################################################################################################

#!/usr/bin/env python3

# This script is my implementation of the LZ77 compression algorithm.  The script takes in a
# file as input and outputs a file with compressed data.   

__author__ = 'Michael Kasza'
__status__ = 'Development'

import argparse
import datetime

# from mk_compress_utils import *

 # This file contains utility functions to be used by the compress.py script

def isValidFilename(filename):
    if '.' in filename:
            return True
            
    return False

def appendToFilename(f_name,string):
    if '.' in f_name:
        i = f_name.index('.')
        f_name = f_name[:i]+string+f_name[i:]
    else:
        f_name += string
        
    return f_name
        
        
    
def copy(data,start,end):

    length = len(data)-start

    ret_data = []

    for i in range(0,end-start):
            ret_data.append(data[start:len(data)][i%length])
            
    return ret_data
    
def fileToList(filename):

    # Read data from input file, store it in a list and then close

    data = []

    f_input = open(filename,'rb')
            
    while True:
            byte = f_input.read(1)
            if not byte:
                    break
            data.append(byte)
            
    f_input.close()
    
    return data
    
def listToFile(filename,data):

    # Writes every element from a list of bytes to a file
    
    f_output = open(filename,'wb')
    
    for byte in data:
        f_output.write(byte)
    f_output.close()
    
def lengthOfCommonSubseq(seq1,seq2):
    
    i = 0
    while i < len(seq1) and i < len(seq2):
        if not seq1[i] == seq2[i]:
            break
        i += 1
        
    return i
        
 
def longestPrefix(data,pos,win_size,pv_size):
    
    start = 0+(pos > win_size)*(pos - win_size)
    
    if pos+pv_size < len(data):
        end = pos+pv_size
    else:
        end = len(data)
        
    longest_start = start
    longest_len = 0
    
    i = start
    while i < pos:
        lcs = lengthOfCommonSubseq(data[i:end],data[pos:end])
        
        if lcs > longest_len:
            longest_len = lcs
            longest_start = abs(pos-i)
        
        i += 1
        
    return longest_start,longest_len
                
        
        
        
    
def xcompress(data):
    
    comp_data = []
    literal_mode = {'status': False, 'index': 0}
    
    # The index of this loop indicates the position of the sliding window.  Everything before
    # the index is already compressed and everything after is uncompressed.
    
    i = 0
    
    while i < len(data):
            
        longest_start,longest_len = longestPrefix(data,i,127,10)
            
        # When the length of the longest sequence of data that can be compressed is less than
        # three bytes, it's not worth it to compress this sequence since compression adds two
        # bytes of overhead.  In this case, the better option is to turn on 'literal mode' and
        # keep adding the next byte to the compressed data until a sequence is found that is
        # worth compressing.
        
        if longest_len < 3:
            if literal_mode['status'] == True:
                
                # If literal mode is already on, get the current length of the literal sequence
                # and add one since one byte is about to be added
                
                literal_length = int.from_bytes(comp_data[literal_mode['index']],byteorder='big')
                literal_length += 1
                
                comp_data[literal_mode['index']] = literal_length.to_bytes(1,byteorder='big')
                comp_data.append(data[i])
                
                # Exit literal mode if the maximum length has been reached.  If the next byte
                # is still supposed to be part of a literal sequence, a new literal sequence
                # will have to be started
                
                if comp_data[literal_mode['index']] == 255:
                    literal_mode['status'] = False
                    
            
            else:
                # When literal mode is first turned on, one byte is added to the beginning of the
                # sequence.  The highest order bit indicates that literal mode is on, and the
                # remaining seven bits encode the length of the literal bytes to follow
                
                comp_data.append((129).to_bytes(1,byteorder='big'))
                comp_data.append(data[i])
                
                literal_mode['status'] = True
                literal_mode['index'] = len(comp_data)-2
                
            # When literal mode is on, only one byte will be added to the compressed data
            # on each iteration
            i += 1
        else:
            # Set the next_byte variable equal to the next byte in the data
            # list that comes after the data to be added to the dictionary.
            # If the index of this byte is past the end of the list, set
            # next_byte equal to the end character instead.  The end character
            # is '$' which has the decimal value 36 in UTF-8 encoding.
            
            if i+longest_len < len(data):
                next_byte = data[i+longest_len]
            else:
                next_byte = (36).to_bytes(1,byteorder='big')
                
            # Append the compressed data to the comp_data list
            
            comp_data.append(longest_start.to_bytes(1,byteorder='big'))
            comp_data.append(longest_len.to_bytes(1,byteorder='big'))
            comp_data.append(next_byte)
            
            # If literal mode was on, turn it off
            
            literal_mode['status'] = False
            
            # Since longest_len+1 bytes were just added to the compressed data,
            # advance the list index ahead by that many bytes
            
            i += longest_len+1
            
    return comp_data

####################################################################################################
####################################################################################################
####################################################################################################
####################################################################################################
####################################################################################################



import struct
import sys
import math

def LZ77_search(search, look_ahead):

  ls = len(search)
  llh = len(look_ahead)
 
  if(ls==0):
   return (0,0, look_ahead[0])
  
  if(llh)==0:
   return (-1,-1,"")

  best_length=0
  best_offset=0 
  buf = search + look_ahead

  search_pointer = ls 
  #print( "search: " , search, " lookahead: ", look_ahead)
  for i in range(0,ls):
   length = 0
   while buf[i+length] == buf[search_pointer +length]:
    length = length + 1
    if search_pointer+length == len(buf):
     length = length - 1
     break
    if i+length >= search_pointer:
     break  
   if length > best_length:
    best_offset = i
    best_length = length

  return (best_offset, best_length, buf[search_pointer+best_length])



def main():
 #extra credit
 x = 16
 MAXSEARCH = int(sys.argv[2])
 MAXLH =  int(math.pow(2, (x  - (math.log(MAXSEARCH, 2))))) 

 file_to_read = sys.argv[1] 
 input = parse(file_to_read)
 file = open("compressed.bin", "wb")
 searchiterator = 0;
 lhiterator = 0;

 while lhiterator<len(input):
  search = input[searchiterator:lhiterator]
  look_ahead = input[lhiterator:lhiterator+MAXLH]
  (offset, length, char) = LZ77_search(search, look_ahead)
  #print (offset, length, char)
  
  shifted_offset = offset << 6
  offset_and_length = shifted_offset+length 
  ol_bytes = struct.pack(">Hc",offset_and_length,char)  
  file.write(ol_bytes) 
   

  lhiterator = lhiterator + length+1
  searchiterator = lhiterator - MAXSEARCH

  if searchiterator<0:
   searchiterator=0
    

 file.close()
   

def parse(file):
 r=[]
 f = open(file, "rb" )
 text = f.read()
 return text


####################################################################################################
####################################################################################################
####################################################################################################
####################################################################################################
####################################################################################################
####################################################################################################

####################################################################################################










##################################################
# Move types.
# Printing move, X/Y move in combination with extrusion A/B
MoveTypePrint = 0
# Traveling move,
# * X/Y move without extrusion A/B
# * Z move
# * Extrusion A/B only move
MoveTypeTravel = 1

##################################################

class AccelData:

    def __init__(self):

        pass

        # Time for the three phases of this move
        # self.accelTime = None
        # self.linearTime = None
        # self.decelTime = None

    def setDuration(self, accelTime, linearTime, decelTime):

        assert(accelTime >= 0)
        assert(linearTime >= 0)
        assert(decelTime >= 0)

        self.accelTime = accelTime
        self.linearTime = linearTime
        self.decelTime = decelTime

    def getTime(self):

        return self.accelTime + self.linearTime + self.decelTime

    def sanityCheck(self):

        if hasattr(self, "accelTime"):
            assert(self.accelTime >= 0)
        if hasattr(self, "linearTime"):
            assert(self.linearTime >= 0)
        if hasattr(self, "decelTime"):
            assert(self.decelTime >= 0)

    def __repr__(self):
        return "\n  AccelTime: %f, LinearTime: %f, DecelTime: %f" % (self.accelTime, self.linearTime, self.decelTime)

class StepData:

    def __init__(self):
        self.accelPulses = []
        self.linearTimer = None
        self.decelPulses = []
        self.setDirBits = False
        self.dirBits = 0
        self.leadAxis = 0
        self.abs_vector_steps = None

    """
    def addAccelPulse(self, timer):

        self.checkTimerValue(timer, maxTimerValue16)
        self.accelPulses.append(timer)

    def addAccelPulsees(self, accelPulses):

        for (_, _, timer) in accelPulses:
            self.addAccelPulse(timer)
    """

    def setAccelPulses(self, accelPulses):
        self.accelPulses = accelPulses

    def setLinTimer(self, timer):
        self.linearTimer = timer
  
    def setDecelPulses(self, decelPulses):
        self.decelPulses = decelPulses

    """
    def addDecelPulse(self, timer):

        self.checkTimerValue(timer, maxTimerValue16)
        self.decelPulses.append(timer)

    def addDecelPulsees(self, decelPulses):

        for (_, _, timer) in decelPulses:
            self.addDecelPulse(timer)
    """

    def setBresenhamParameters(self, leadAxis, abs_vector_steps):
        self.leadAxis = leadAxis
        self.abs_vector_steps = abs_vector_steps

    def empty(self):
        # return not self.abs_vector_steps[self.leadAxis]
        return not self.abs_vector_steps

    def __repr__(self):
        return "StepData:" + \
           "\n  SetDirbits: %s, Direction bits: 0x%x" % (self.setDirBits, self.dirBits) + \
           "\n  # leadAxis: %d" % self.leadAxis + \
           "\n  # abs_vector_steps: %s" % `self.abs_vector_steps` + \
           "\n  # accelPulses: %d" % len(self.accelPulses) + \
           "\n  # linearPulses: %d" % (self.abs_vector_steps[self.leadAxis] - (len(self.accelPulses) + len(self.decelPulses))) + \
           "\n  # decelPulses: %d" % len(self.decelPulses)

    def checkLen(self, deltaLead):
        assert(self.abs_vector_steps[self.leadAxis] - (len(self.accelPulses) + len(self.decelPulses)) >= 0)

    # return a list of binary encoded commands, ready to be send over serial...
    def commands(self, move):

        flags = 0
        if self.setDirBits:
            flags = self.dirBits | DirBitsBit

        # Check if acceleration timer values can be sent as difference bytes
        accelByteFlag = AccelByteFlagBit
        if self.accelPulses:
            lastTimer = self.accelPulses[0]
            for tv in self.accelPulses[1:]:
                dtv = lastTimer - tv

                assert(dtv >= 0)

                if dtv > 255:
                    accelByteFlag = 0
                    break

                lastTimer = tv

        # Check if deceleration timer values can be sent as difference bytes
        decelByteFlag = DecelByteFlagBit
        if self.decelPulses:
            lastTimer = self.decelPulses[0]
            for tv in self.decelPulses[1:]:
                dtv = tv - lastTimer

                assert(dtv >= 0)

                if dtv > 255:
                    decelByteFlag = 0
                    break

                lastTimer = tv
       
        startMoveFlag = 0
        if move.isStartMove:
            startMoveFlag = MoveStartBit

        measureStartBit = 0
        if move.isMeasureMove:
            measureStartBit = MeasureStartBit

        endMoveBit = 0
        if move.isEndMove:
            endMoveBit = EndMoveBit

        # print "flags: 0x%x, dirbits: 0x%x" % (flags | accelByteFlag | decelByteFlag, flags)
        # print "ald:", len(self.accelPulses), self.abs_vector_steps[self.leadAxis]-(len(self.accelPulses)+len(self.decelPulses)), len(self.decelPulses)

        #
        # Payload in *LenCobs* blocksize blocks
        #
        payLoadBlocks = []

        payLoad = struct.pack("<HBiiiiiH",
                flags | accelByteFlag | decelByteFlag | startMoveFlag | measureStartBit | endMoveBit,
                self.leadAxis,
                self.abs_vector_steps[0],
                self.abs_vector_steps[1],
                self.abs_vector_steps[2],
                self.abs_vector_steps[3],
                self.abs_vector_steps[4],
                len(self.accelPulses))

        if move.isStartMove:

            # Store timervalue of max e-speed
            baseMove = move.getBaseMove()
            if baseMove.isPrintMove():

                # xxx move to own method  
                nominalSpeed = abs( baseMove.topSpeed.speed().eSpeed )
                e_steps_per_second_nominal = nominalSpeed * baseMove.e_steps_per_mm
                timerValue = fTimer / e_steps_per_second_nominal

                # print "StartMove, E-Timer:", nominalSpeed, timerValue
                payLoad += struct.pack("<H", min(timerValue, 0xffff))

            else:
                payLoad += struct.pack("<H", 0)

        if move.isMeasureMove:

            # Store window size of running average
            nAvg = min (255, PrinterProfile.getNShortInterval(move.getBaseMove().measureSpeed))
            payLoad += struct.pack("<B", nAvg)

        payLoad += struct.pack("<HH",
                self.linearTimer,
                len(self.decelPulses))

        if self.accelPulses and accelByteFlag:

            lastTimer = self.accelPulses[0]
            payLoad += struct.pack("<H", lastTimer)

            for tv in self.accelPulses[1:]:
                dtv = lastTimer - tv
                payLoad += struct.pack("<B", dtv)
                lastTimer = tv

        else:

            for timer in self.accelPulses:
                payLoad += struct.pack("<H", timer)

        if self.decelPulses and decelByteFlag:

            lastTimer = self.decelPulses[0]
            payLoad += struct.pack("<H", lastTimer)

            for tv in self.decelPulses[1:]:
                dtv = tv - lastTimer
                payLoad += struct.pack("<B", dtv)
                lastTimer = tv

        else:

            for timer in self.decelPulses:
                payLoad += struct.pack("<H", timer)

        stream = cStringIO.StringIO(payLoad)

        # print "start combs"
        # cobsBlock = cobs.encodeCobs_cmd_packed(stream)
        # cmds = [( ddprintcommands.CmdG1, cobsBlock )]

        cmds = [ cobs.encodeCobs_cmd_packed(ddprintcommands.CmdG1, ddprintcommands.CmdG1Packed, stream) ]

        while True:

            # print "follow combs"
            # cobsBlock = cobs.encodeCobs_cmd_packed(stream)
            cobsBlock = cobs.encodeCobs_cmd_packed(ddprintcommands.CmdBlock, ddprintcommands.CmdBlockPacked, stream)

            if not cobsBlock:
                break

            # cmds.append(( ddprintcommands.CmdBlock, cobsBlock ))
            cmds.append( cobsBlock )

        return cmds

    def debugPlot(self):

        d = {
                "stepType": "bresenham",
                "leadAxis": self.leadAxis,
                "accelPulses": self.accelPulses,
                "linearTimer": self.linearTimer,
                "linearSteps": self.abs_vector_steps[self.leadAxis] - (len(self.accelPulses)+len(self.decelPulses)),
                "decelPulses": self.decelPulses,
                }

        if self.setDirBits:
            d["dirbits"] = self.dirBits

        return d

class RawStepData:

    def __init__(self):
        self.pulses = []
        self.setDirBits = False
        self.dirBits = 0

    def addPulse(self, timerValue, pulse):

        assert(timerValue <= maxTimerValue16)
        self.pulses.append((timerValue, pulse))

    def empty(self):
        return not self.pulses

    def __repr__(self):
        return "RawStepData:" + \
           "\n  Direction bits: 0x%x" % self.dirBits + \
           "\n  # pulses: %d" % len(self.pulses)

    # For step bitmask see move.h::st_get_move_bit_mask()
    def stepBits(self, stepBits):

        bits = 0
        for i in range(5):
            if stepBits[i]:
                bits |= (0x1 << i)

        return bits

    # return a list of binary encoded commands, ready to be send over serial...
    def commands(self, move):

        flags = 0
        if self.setDirBits:
            flags = self.dirBits | DirBitsBitRaw

        # Check if timer values can be sent as difference bytes
        timerByteFlag = TimerByteFlagBit
        if self.pulses:
            lastTimer = self.pulses[0][0]
            for (tv, _) in self.pulses[1:]:
                dtv = lastTimer - tv

                if dtv > 127 or dtv < -128:
                    timerByteFlag = 0
                    break

                lastTimer = tv
       
        startMoveFlag = 0
        if move.isStartMove:
            startMoveFlag = MoveStartBitRaw

        measureStartBit = 0
        if move.isMeasureMove:
            measureStartBit = MeasureStartBitRaw

        endMoveBit = 0
        if move.isEndMove:
            endMoveBit = EndMoveBitRaw

        payLoad = struct.pack("<HH",
                flags | timerByteFlag | startMoveFlag | measureStartBit | endMoveBit,
                len(self.pulses))

        if move.isStartMove:

            # Store timervalue of max e-speed
            baseMove = move.getBaseMove()

            # xxx move to own method  
            nominalSpeed = abs( baseMove.topSpeed.speed().eSpeed )
            e_steps_per_second_nominal = nominalSpeed * baseMove.e_steps_per_mm
            timerValue = fTimer / e_steps_per_second_nominal

            # print "StartMove, E-Timer:", nominalSpeed, timerValue
            payLoad += struct.pack("<H", min(timerValue, 0xffff))

        if move.isMeasureMove:

            # Store window size of running average
            nAvg = min (255, PrinterProfile.getNShortInterval(move.getBaseMove().measureSpeed))
            payLoad += struct.pack("<B", nAvg)

        if timerByteFlag:

            (lastTimer, stepBits) = self.pulses[0]
            payLoad += struct.pack("<HB", lastTimer, self.stepBits(stepBits))

            for (tv, stepBits) in self.pulses[1:]:
                dtv = lastTimer - tv
                payLoad += struct.pack("<bB", dtv, self.stepBits(stepBits))
                lastTimer = tv

        else:

            for (timer, stepBits) in self.pulses:
                payLoad += struct.pack("<HB", timer, self.stepBits(stepBits))

        stream = cStringIO.StringIO(payLoad)

        # cobsBlock = cobs.encodeCobs_cmd_packed(stream)
        # cmds = [( ddprintcommands.CmdG1Raw, cobsBlock )]

        cmds = [ cobs.encodeCobs_cmd_packed(ddprintcommands.CmdG1Raw, ddprintcommands.CmdG1RawPacked, stream) ]

        while True:

            # cobsBlock = cobs.encodeCobs_cmd_packed(stream)
            cobsBlock = cobs.encodeCobs_cmd_packed(ddprintcommands.CmdBlock, ddprintcommands.CmdBlockPacked, stream)

            if not cobsBlock:
                break

            # cmds.append(( ddprintcommands.CmdBlock, cobsBlock ))
            cmds.append( cobsBlock )

        return cmds

    def debugPlot(self):

        d = {
                "stepType": "raw",
                "pulses": self.pulses,
                }

        if self.setDirBits:
            d["dirbits"] = self.dirBits

        return d

class AdvanceData:

    def __init__(self, move):

        self.move = move

        # Additional start E-Feedrate if advance applied or 0
        self.startFeedrateIncrease = 0
        # Additional end E-Feedrate if advance applied or 0
        self.endFeedrateIncrease = 0

        self.startSplits = 0
        self.endSplits = 0

        self.startESteps = None
        self.linESteps = None
        self.endESteps = None
        self.endEStepsC = None
        self.endEStepsD = None

        # self.accelGroup = []
        self.sAccel = 0.0
        # self.sAccelSum = 0.0

        # self.decelGroup = []
        self.sDecel = 0.0
        # self.sDecelSum = 0.0

        # Debug, prüfung ob alle in planAdvance() berechneten e-steps in planSteps() 
        # verwendet werden. Summe ist im idealfall 0, kann aber aufgrund von rundungsfehlern
        # auch ungleich null sein.
        self.advStepSum = 0

        # xxx
        # self.hasAccelAdvance = False
        # self.hasDecelAdvance = False

    def hasStartAdvance(self):
        return self.startFeedrateIncrease != 0

    def startEFeedrate(self):
        return self.move.startSpeed.speed().eSpeed + self.startFeedrateIncrease

    # xxx rename to startETopFeedrate
    def startEReachedFeedrate(self):
        return self.move.topSpeed.speed().eSpeed + self.startFeedrateIncrease

    def hasEndAdvance(self):
        return self.endFeedrateIncrease != 0

    # xxx rename to endETopFeedrate
    def endEReachedFeedrate(self):

        return self.move.topSpeed.speed().eSpeed + self.endFeedrateIncrease

    def endEFeedrate(self):

        return self.move.endSpeed.speed().eSpeed + self.endFeedrateIncrease

    # Check if sign changes at accel/decel
    def startSignChange(self):
        return ddprintutil.sign(self.startEFeedrate()) != ddprintutil.sign(self.startEReachedFeedrate())

    def endSignChange(self):

        v0 = self.endEReachedFeedrate()
        v1 = self.endEFeedrate()

        if v0 == 0 or v1 == 0:
            return False

        if v0 >= 0 and v1 >= 0:
            return False

        if v0 < 0 and v1 < 0:
            return False

        return True

    def estepSum(self):

        esteps = 0
        if self.startESteps:
            esteps += self.startESteps
        if self.linESteps:
            esteps += self.linESteps
        if self.endESteps:
            esteps += self.endESteps
        if self.endEStepsC:
            esteps += self.endEStepsC
        if self.endEStepsD:
            esteps += self.endEStepsD

        return esteps

    def __repr__(self):

        s = ""
        if self.hasStartAdvance():
            s += "\n  EStartAdvance: %.3f, Start %.3f, Top: %.3f" % (self.startFeedrateIncrease, self.startEFeedrate(), self.startEReachedFeedrate())
        if self.hasEndAdvance():
            s += "\n    EEndAdvance: %.3f, Top %.3f, End: %.3f" % (self.endFeedrateIncrease, self.endEReachedFeedrate(), self.endEFeedrate())

        if self.startESteps:
            s += "\n startESteps: %.3f" % self.startESteps
        if self.linESteps:
            s += "\n linESteps: %.3f" % self.linESteps
        if self.endESteps:
            s += "\n endESteps: %.3f" % self.endESteps
        if self.endEStepsC:
            s += "\n endEStepsC: %.3f" % self.endEStepsC
        if self.endEStepsD:
            s += "\n endEStepsD: %.3f" % self.endEStepsD

        esteps = self.estepSum()
        if esteps:
            s += "\n estep sum: %.3f" % esteps

        # s += "\n Group data:"
        # s += "\n Accel group:" + str(map(lambda m: m.moveNumber, self.accelGroup))
        s += "\n sAccel: %.3f" % self.sAccel
        # s += "\n sAccelSum: %.3f" % self.sAccelSum
        # s += "\n Decel group:" + str(map(lambda m: m.moveNumber, self.decelGroup))
        s += "\n sDecel: %.3f" % self.sDecel
        # s += "\n sDecelSum: %.3f" % self.sDecelSum
        return s

    def sanityCheck(self):

        # XXX assuming no retraction moves with advance
        if self.hasStartAdvance():
            assert(self.move.startSpeed.speed()[A_AXIS] >= 0)
            assert(self.move.topSpeed.speed()[A_AXIS] > 0)
            assert(self.startFeedrateIncrease >= 0)

        if self.hasEndAdvance():
            assert(self.move.topSpeed.speed()[A_AXIS] > 0)
            assert(self.move.endSpeed.speed()[A_AXIS] >= 0)
            assert(self.endFeedrateIncrease >= 0)

##################################################
#
#
#
class VelocityOverride(object):

    def __init__(self, nominalSpeed):
        self.speeds = [(nominalSpeed, "initial")]

    def speed(self):
        return self.speeds[-1][0].copy()

    def setSpeed(self, speed, comment):

        # debug
        if speed == self.speeds[-1]:
            print "duplicate speed: ", speed, comment
            assert(0)

        if speed != self.speeds[-1]:
            self.speeds.append((speed, comment))

    def nu_nominalSpeed(self, speed=None):

        if speed:
            self.setSpeed(speed)
            return

        return self.speed()

    def nu_plannedSpeed(self, speed=None):
        return self.nominalSpeed(speed)

    def nu_trueSpeed(self, speed=None):
        return self.nominalSpeed(speed)

    def __repr__(self):

        s = ""

        for (speed, comment) in self.speeds:
            s += "\n\tSpeed: " + str(speed) + " " + comment

        return s

##################################################

class AccelOverride(object):

    def __init__(self, accel, xyzDirection):
        self.accels = [accel]
        self.xyzDirection = xyzDirection

    def xyAccel(self):
        return vectorLength(self.accels[-1][0])

    def eAccel(self):
        return self.accels[-1][1]

    def setAccel(self, xyAccel, eAccel):

        """
        # debug
        if xyAccel == self.xyAccel() and eAccel == self.eAccel():
            print "duplicate accel: ", xyAccel, eAccel
            assert(0)
        """

        self.accels.append([self.xyzDirection.scale(xyAccel), eAccel])

    def accel(self, dim):

        if dim == A_AXIS:
            return self.eAccel()

        return self.accels[-1][0][dim]

    def __repr__(self):

        s = ""

        for (accel3, eAccel) in self.accels:
            s += "\n\tXYZAccel: " + str(accel3) + (" EAccel: %f" % eAccel)

        return s

##################################################

class MoveBase(object):

    def __init__(self):

        self.accelData = AccelData()

        # debug
        self.state = 0 # 1: joined, 2: accel planned, 3: steps planned

        self.moveNumber = None

        self.stepData = None

        self.isStartMove = False

        self.isMeasureMove = False

        # Last move of a path, used to implement
        # soft stop.
        self.isEndMove = False

    def isMove(self):
        return True

    def isSubMove(self):
        return False

    # Returns base move, eg our self
    def getBaseMove(self):
        return self

    def setDuration(self, accelTime, linearTime, decelTime):

        self.accelData.setDuration(accelTime, linearTime, decelTime)

    def accelTime(self):

        return self.accelData.accelTime

    def linearTime(self):

        return self.accelData.linearTime

    def decelTime(self):

        return self.accelData.decelTime

    def getTime(self):

        return self.accelData.getTime()

    # Get vector of absolute steps.
    def absStepsVector(self, disp=None):

        if disp != None:
            return vectorAbs(disp)

        asv = []
        for dim in range(3):
            asv.append(abs(self.displacement_vector_steps_raw3[dim]))

        return asv + [abs(self.eSteps), 0]

    def leadAxis(self, nAxes=5, disp=None):

        asv = self.absStepsVector(disp)
        maxstep = 0
        maxdim = 0
        for dim in range(nAxes):
            if asv[dim] > maxstep:
                maxdim = dim
                maxstep = asv[dim]

        # Use top speed to determine leadAxis if x- and y-axis have the same step amount
        if maxdim == X_AXIS and asv[X_AXIS] == asv[Y_AXIS] and abs(self.topSpeed.speed()[Y_AXIS]) > abs(self.topSpeed.speed()[X_AXIS]):
            return Y_AXIS

        return maxdim

    def initStepData(self, stepDataType):

        if stepDataType == StepDataTypeBresenham:
            self.stepData = StepData()
        elif stepDataType == StepDataTypeRaw:
            self.stepData = RawStepData()
        else:
            assert(0)

    def empty(self):
        return self.stepData.empty()

    # return a list of binary encoded commands, ready to be send over serial...
    def commands(self):

        if not self.empty():
            return self.stepData.commands(self)

        return []

    def sanityCheck(self, checkDirection=True):

        ss = self.startSpeed.speed()
        ts = self.topSpeed.speed()
        es = self.endSpeed.speed()

        # All velocities should have reasonable feedrates
        assert(ss.feedrateGEZ())
        assert(ts.feedrateGZ())
        assert(es.feedrateGEZ())

        if checkDirection:

            # All velocities should point into the same direction
            assert(vectorLength(vectorSub(ss.direction, ts.direction)) < 0.001)
            assert(vectorLength(vectorSub(es.direction, ts.direction)) < 0.001)

        self.accelData.sanityCheck()

    def pprint(self, title):

        print "\n------ Move %s, #: %d, '%s' ------" % (title, self.moveNumber, self.comment)

        if self.isPrintMove():
            print "Print-move, distance: %s" % self.distanceStr()
        else:
            print "Travel-move, distance: %s" % self.distanceStr()

        print "displacement_vector:", self.rawDisplacementStr(), "_steps:", self.rawDisplacementStepsStr()

        print "Startspeed: ",
        print self.startSpeed
        print "Top  speed: ",
        print self.topSpeed
        print "End  speed: ",
        print self.endSpeed

        if self.state > 1:
            print ""
            print self.accelData

        if self.state > 2:
            print ""
            print self.stepData

        print "---------------------"

# Base class for TravelMove and PrintMove
class RealMove(MoveBase):

    def __init__(self, comment, layerPart):

        MoveBase.__init__(self)

        self.prevMove = None
        self.nextMove = None

        self.comment = comment

        self.accelData = AccelData()

        self.layerPart = layerPart

        # assert(self.layerPart != "infill")

        # debug
        self.state = 0 # 1: joined, 2: accel planned, 3: steps planned

    def getJerkSpeed(self, jerk):

        return self.topSpeed.speed().constrain(jerk)

    def setPlannedJerkStartSpeed(self, jerk, comment):

        v = self.getJerkSpeed(jerk)
        if v != None:
            self.startSpeed.setSpeed(v, "setPlannedJerkStartSpeed " + comment)

    def setPlannedJerkEndSpeed(self, jerk, comment):

        v = self.getJerkSpeed(jerk)
        if v != None:
            self.endSpeed.setSpeed(v, "setPlannedJerkEndSpeed " + comment)

    def sanityCheck(self):

        MoveBase.sanityCheck(self)

    def isInfill(self):
        return self.layerPart == "infill"

    def hasAdvance(self):
        return not self.isInfill()

class TravelMove(RealMove):

    def __init__(self,
                 comment,
                 displacement_vector,
                 displacement_vector_steps,
                 feedrate, # mm/s
                 layerPart,
                 ):

        # assert(layerPart != "infill")
        RealMove.__init__(self, comment, layerPart)

        # self.displacement_vector_raw = displacement_vector

        # self.displacement_vector3=displacement_vector[:3]
        # self.displacement_vector_steps3=displacement_vector_steps[:3]
        # self.extrusion_displacement_raw = displacement_vector[3:]
        # self.extrusion_displacement_steps_raw = displacement_vector_steps[3:]

        #
        # Move distance in XYZAB plane
        #
        self.distance5 = displacement_vector.length()

        self.displacement_vector_raw5 = displacement_vector
        self.displacement_vector_steps_raw5 = displacement_vector_steps
        self.direction5 = displacement_vector.normalized()

        v = VelocityVector5(feedrate = feedrate, direction = self.direction5)

        self.startSpeed = VelocityOverride(v)
        self.topSpeed = VelocityOverride(v)
        self.endSpeed = VelocityOverride(v)

    def isPrintMove(self):
        return False

    def distanceStr(self):
        d = self.displacement_vector_raw5.length()
        return "%.2f mm (XYZAB)" % d

    def rawDisplacementStr(self):
        return str(self.displacement_vector_raw5)

    def rawDisplacementStepsStr(self):
       return str(self.displacement_vector_steps_raw5)

    # Note: returns positive values 
    def getMaxAllowedAccelVectorNoAdv5(self):

        accelVector = self.direction5.scale(_MAX_ACCELERATION)
        return abs(accelVector.constrain(PrinterProfile.getMaxAxisAcceleration()) or accelVector)

    # Note: always positive
    def getMaxAllowedAccelNoAdv5(self):

        accelVector = self.getMaxAllowedAccelVectorNoAdv5()
        return accelVector.length() # always positive

    def sanityCheck(self, jerk):

        RealMove.sanityCheck(self)

        # Check start ramp
        assert(self.startSpeed.speed().feedrate5() <= self.topSpeed.speed().feedrate5());

        # Check end ramp
        assert(self.topSpeed.speed().feedrate5() >= self.endSpeed.speed().feedrate5());

        nullV = self.topSpeed.speed()
        # xxx use same start speed as addMove() here!
        # nullV.setSpeed(0.1)
        nullV.setSpeed(0.0)

        nextMove = self.nextMove

        if nextMove:

            endSpeed1 = self.endSpeed.speed()
            startSpeed2 = nextMove.startSpeed.speed()
            endSpeed1.checkJerk(startSpeed2, jerk, "#: %d" % self.moveNumber, "#: %d" % nextMove.moveNumber)

        else:

            # Last move
            self.endSpeed.speed().checkJerk(nullV, jerk)

        if not self.prevMove:

            # First move
            self.startSpeed.speed().checkJerk(nullV, jerk, "start 0", "#: %d" % self.moveNumber)

##################################################

class PrintMove(RealMove):

    def __init__(self,
                 comment,
                 displacement_vector,
                 displacement_vector_steps,
                 feedrate, # mm/s
                 layerPart,
                 maxAccelV,
                 ):

        # assert(layerPart != "infill")
        RealMove.__init__(self, comment, layerPart)

        #
        # Move distance in XYZ plane
        #
        self.distance3 = displacement_vector.length(3)

        self.advanceData = AdvanceData(self)

        self.e_steps_per_mm = PrinterProfile.getStepsPerMM(A_AXIS)

        self.displacement_vector_raw3 = Vector(displacement_vector[:3])
        self.displacement_vector_steps_raw3 = displacement_vector_steps[:3]

        self.direction3 = self.displacement_vector_raw3.normalized()

        ### Apply extrusion adjust
        ### if UseExtrusionAdjust:

        direction5 = displacement_vector.normalized()

        self.eDistance = displacement_vector[A_AXIS]

        # xxx todo: add override
        self.eSteps = displacement_vector_steps[A_AXIS]

        self.eDirection = self.eDistance / self.distance3

        # Compute nominal eSpeed

        v = VelocityVector32(feedrate*self.eDirection, feedrate = feedrate, direction = self.direction3)

        self.startSpeed = VelocityOverride(v)
        self.topSpeed = VelocityOverride(v)
        self.endSpeed = VelocityOverride(v)

        # self.__direction5 = displacement_vector.normalized()
        # av = self.__getMaxAllowedAccelVector5(maxAccelV)

        accelVector = direction5.scale(_MAX_ACCELERATION)
        av = accelVector.constrain(maxAccelV) or accelVector

        # xxx rework accel, store default xyz and eaccel, make start- and eaccel overridable
        self.startAccel = AccelOverride([av[:3], av[A_AXIS]], self.direction3)
        self.endAccel = AccelOverride([av[:3], av[A_AXIS]], self.direction3)

    def isPrintMove(self):
        return True

    def distanceStr(self):
        d = self.displacement_vector_raw3.length()
        return "%.2f mm (XYZ)" % d

    def rawDisplacementStr(self):
        return str(self.displacement_vector_raw3) + (" [%.3f]" % self.eDistance)

    def rawDisplacementStepsStr(self):
        return str(self.displacement_vector_steps_raw3) + (" [%.3f]" % self.eSteps)

    # Alle achsen werden in der gleichen zeit beschleunigt.
    # Dadurch teilen sich die zulässigen einzelbeschleunigungen
    # im entsprechenden verhältnis auf.
    # def __getMaxAllowedAccelVector5(self, maxAccelV):
        # accelVector = self.__direction5.scale(_MAX_ACCELERATION)
        # return accelVector.constrain(maxAccelV) or accelVector

    ####### always positive
    ######def __getMaxAllowedAccel5(self, maxAccelV):
        ######accelVector = self.getMaxAllowedAccelVector5(maxAccelV)
        ######return accelVector.length()

    ################################################################################
    # Area (e-distance) of advance start ramp
    # Berechnet die trapezfläche und damit die durch advance zusätzlich
    # extruderstrecke.
    # Annahme: startFeedrateIncrease immer positiv (keine retraction), dadurch
    # return value auch immer positiv.
    def startAdvDistance(self, ta, startFeedrateIncrease=None):

        if startFeedrateIncrease == None:
            startFeedrateIncrease = self.advanceData.startFeedrateIncrease

        # Trapezberechnung
        sadv = startFeedrateIncrease * ta
        assert(sadv > 0)
        return sadv

    # Area (e-distance) of advance end ramp
    # Berechnet die trapezfläche und damit die durch advance verringerte
    # extruderstrecke.
    # Annahme: endFeedrateIncrease immer negativ (keine retraction), dadurch
    # return value auch immer negativ.
    def endAdvDistance(self, td, endFeedrateIncrease=None):

        if endFeedrateIncrease == None:
            endFeedrateIncrease = self.advanceData.endFeedrateIncrease

        # Trapezberechnung, resultat negativ, da endFeedrateIncrease negativ ist
        sadv = endFeedrateIncrease * td
        assert(sadv < 0)
        return sadv
    ################################################################################

    ################################################################################
    def startAdvSteps(self, startFeedrateIncrease=None):

        ta = self.accelTime()

        if not ta:
            return 0.0

        sa = self.startAdvDistance(ta, startFeedrateIncrease)
        # esteps = sa * self.e_steps_per_mm

        return sa

    def endAdvSteps(self, endFeedrateIncrease=None):

        td = self.decelTime()

        if not td:
            return 0.0

        sd = self.endAdvDistance(td, endFeedrateIncrease)
        # esteps = sd * self.e_steps_per_mm

        return sd
    ################################################################################

    ################################################################################
    # Berechnet die fläche des dreieckigen anteils (und damit die strecke) der start-rampe.
    # Vorzeichen:
    #   v0, v1 positiv: resultat positiv
    #   v0, v1 negativ: resultat negativ
    def startRampTriangle(self, v0, v1, dt):

        # print "v0:", v0, "v1:", v1, "dt:", dt

        if not dt:
            return 0.0

        if v1 > 0: 
            assert(v0 >= 0)
            assert(v1 >= v0)
        elif v1 < 0:
            assert(v0 <= 0)
            assert(v1 <= v0)

        return ((v1 - v0) * dt) / 2.0

    def endRampTriangle(self, v0, v1, dt):

        # print "v0:", v0, "v1:", v1, "dt:", dt

        if not dt:
            return 0.0

        if v0 > 0:
            assert(v1 >= 0)
            assert(v0 >= v1)
        if v0 < 0:
            assert(v1 <= 0)
            assert(v0 <= v1)

        return ((v0 - v1) * dt) / 2.0
    ################################################################################

    ################################################################################
    # Berechnet die fläche (und damit die strecke) der start-rampe.
    # Diese besteht aus zwei teilen:
    # * Der rechteckige teil der aus v0*dt besteht
    # * Und dem dreieckigen teil der aus dv*dt besteht.
    # Vorzeichen:
    #   v0, v1 positiv: resultat positiv
    #   v0, v1 negativ: resultat negativ
    def startRampDistance(self, v0, v1, dt):

        return self.startRampTriangle(v0, v1, dt) + v0 * dt

    def endRampDistance(self, v0, v1, dt):

        return self.endRampTriangle(v0, v1, dt) + v1 * dt
    ################################################################################

    ################################################################################
    def startERampDistance(self, ta=None, startFeedrateIncrease=None):

        if ta == None:
            ta = self.accelTime()

        s = self.startRampDistance(
                self.startSpeed.speed().eSpeed,
                self.topSpeed.speed().eSpeed,
                ta)
        return s + self.startAdvDistance(ta, startFeedrateIncrease)

    def endERampDistance(self, td=None, endFeedrateIncrease=None, v0=None, v1=None):

        if td == None:
            td = self.decelTime()

        if v0 == None:
            v0 = self.topSpeed.speed().eSpeed

        if v1 == None:
            v1 = self.endSpeed.speed().eSpeed

        s = self.endRampDistance(v0, v1, td)
        return s + self.endAdvDistance(td, endFeedrateIncrease)
    ################################################################################

    ################################################################################
    def startERampSteps(self, startFeedrateIncrease=None):

        ta = self.accelTime()

        sTri = self.startRampDistance(
                self.startSpeed.speed().eSpeed,
                self.topSpeed.speed().eSpeed,
                ta)

        sPara = self.startAdvDistance(ta, startFeedrateIncrease)

        assert(sPara >= 0)

        sa = sTri + sPara
        esteps = sa * self.e_steps_per_mm

        return esteps

    def endERampSteps(self, td=None, endFeedrateIncrease=None, v0=None, v1=None, roundError=0):

        if td == None:
            td = self.decelTime()

        if endFeedrateIncrease == None:
            endFeedrateIncrease = self.advanceData.endFeedrateIncrease

        if v0 == None:
            v0 = self.topSpeed.speed().eSpeed

        if v1 == None:
            v1 = self.endSpeed.speed().eSpeed

        sTri = self.endRampDistance(v0, v1, td)

        sPara = self.endAdvDistance(td, endFeedrateIncrease)

        # print "sPara: %f" % sPara

        assert(sPara <= 0)

        sd = sTri + sPara
        esteps = sd * self.e_steps_per_mm

        return esteps
    ################################################################################

    def crossedDecelStep(self):
        return False

    def getExtrusionVolume(self, matProfile):
        return self.eDistance * matProfile.getMatArea()

    def sanityCheck(self, jerk):

        RealMove.sanityCheck(self)

        # Check start ramp
        assert(self.startSpeed.speed().feedrate3() <= self.topSpeed.speed().feedrate3());

        # Check end ramp
        assert(self.topSpeed.speed().feedrate3() >= self.endSpeed.speed().feedrate3());

        nullV = self.topSpeed.speed()
        # xxx use same start speed as addMove() here!
        # nullV.setSpeed(0.1)
        nullV.setSpeed(0.0)

        nextMove = self.nextMove

        if nextMove:

            endSpeed1 = self.endSpeed.speed()
            startSpeed2 = nextMove.startSpeed.speed()
            endSpeed1.checkJerk(startSpeed2, jerk, "#: %d" % self.moveNumber, "#: %d" % nextMove.moveNumber)

        else:

            # Last move
            self.endSpeed.speed().checkJerk(nullV, jerk)

        if not self.prevMove:

            # First move
            self.startSpeed.speed().checkJerk(nullV, jerk, "start 0", "#: %d" % self.moveNumber)

        self.advanceData.sanityCheck()

    # not used
    def checkAdvance(self):

        # Check direction of start advance increase

        # Check direction of end advance decrease
        pass

    def pprint(self, title):

        RealMove.pprint(self, title)

        print "Start ESpeed: " + self.startSpeed.speed().eSpeedStr()
        print "  End ESpeed: " + self.endSpeed.speed().eSpeedStr()

        print "Allowed Start Acceleration: ",
        print self.startAccel
        print "Allowed End Acceleration: ",
        print self.endAccel

        print self.advanceData
        print "---------------------"

##################################################

class SubMove(MoveBase):

    def __init__(self,
                 parentMove,
                 moveNumber,
                 displacement_vector_steps):

        # MoveBase.__init__(self, Vector(displacement_vector_steps))
        MoveBase.__init__(self)

        self.parentMove = parentMove

        self.moveNumber = moveNumber

        self.startSpeed = VelocityOverride(None)
        self.topSpeed = VelocityOverride(None)
        self.endSpeed = VelocityOverride(None)

        self.prevMove = None
        self.nextMove = None

        # self.topSpeed = parentMove.topSpeed

        self.displacement_vector_steps_raw3 = displacement_vector_steps[:3]
        self.eSteps = displacement_vector_steps[A_AXIS]

        self.startAccel = parentMove.startAccel
        self.endAccel = parentMove.endAccel

        self.state = 2

    def isSubMove(self):
        return True

    # Returns base move, eg our parent move
    def getBaseMove(self):
        return self.parentMove

    def setSpeeds(self, sv, tv, ev):

        self.startSpeed.setSpeed(sv, "SubMove.setSpeeds")

        self.topSpeed.setSpeed(tv, "SubMove.setSpeeds")

        self.endSpeed.setSpeed(ev, "SubMove.setSpeeds")

    def crossedDecelStep(self):

        if self.endSpeed.speed().eSpeed < 0:
            return True

        # xxx  wofür test auf topspeed < 0 ?
        if self.topSpeed.speed().eSpeed < 0:
            print "crossedDecelStep", self.topSpeed.speed().eSpeed
            assert(0)
            return True

        return False

    def pprint(self, title):

        print "\n------ SubMove # %d: %s, Parent #: %d ------" % (self.moveNumber, title, self.parentMove.moveNumber)

        # print "Print-move, distance: %.2f" % self.distance

        print "displacement_vector_steps:", self.displacement_vector_steps_raw3, self.eSteps

        print "Startspeed: ",
        print self.startSpeed
        print "Top  speed: ",
        print self.topSpeed
        print "End  speed: ",
        print self.endSpeed

        if self.state > 1:
            print ""
            print self.accelData

        if self.state > 2:
            print ""
            print self.stepData

        print "---------------------"

    def sanityCheck(self):

        # MoveBase.sanityCheck(self, checkDirection=False) # directionCheck not true for advanced moves

        if self.displacement_vector_steps_raw3 == [0, 0, 0] and self.eSteps == 0:
            print "ERROR: null move:"
            self.pprint("Nullmove")
            assert(0)


















