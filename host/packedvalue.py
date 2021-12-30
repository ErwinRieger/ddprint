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

import struct, types
from dddebug import assertType

####################################################################################################

class PackedValue:
    def __init__(self, value, fmt):
        self.value = value
        self.fmt = fmt

    def pack(self):
        return struct.pack("<%s" % self.fmt, self.value)

    def __repr__(self):
        return "PackedValue: typ: %s, value: %s" % (self.fmt, str(self.value))

class uint8_t(PackedValue):
    def __init__(self, value):
        assertType(value, int)
        PackedValue.__init__(self, value, "B")

class int16_t(PackedValue):
    def __init__(self, value):
        assertType(value, int)
        PackedValue.__init__(self, value, "h")

class uint16_t(PackedValue):
    def __init__(self, value):
        assertType(value, int)
        PackedValue.__init__(self, value, "H")

class int32_t(PackedValue):
    def __init__(self, value):
        assertType(value, int)
        PackedValue.__init__(self, value, "i")

class uint32_t(PackedValue):
    def __init__(self, value):
        assertType(value, int)
        PackedValue.__init__(self, value, "I")

class float_t(PackedValue):
    def __init__(self, value):
        assertType(value, float)
        PackedValue.__init__(self, value, "f")

class pString_t(PackedValue):

    def __init__(self, value):
        self.value = value.encode()

    def pack(self):
        return struct.pack("<%dp" % (len(self.value)+1), self.value)

# scaledint32_t
class scaledint_t(PackedValue):

    def __init__(self, value, bits):
        self.value = value
        self.bits = bits

    def pack(self):
        return struct.pack("<IB", self.value, self.bits)

class scaledint16_t(PackedValue):

    def __init__(self, value, bits):
        self.value = value
        self.bits = bits

    def pack(self):
        return struct.pack("<HB", self.value, self.bits)

####################################################################################################

