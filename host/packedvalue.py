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

import struct

####################################################################################################

class PackedValue:
    def __init__(self, value, fmt):
        self.value = value
        self.fmt = fmt

    def pack(self):
        return struct.pack("<%s" % self.fmt, self.value)

class uint8_t(PackedValue):
    def __init__(self, value):
        PackedValue.__init__(self, value, "B")

class int16_t(PackedValue):
    def __init__(self, value):
        PackedValue.__init__(self, value, "h")

class uint16_t(PackedValue):
    def __init__(self, value):
        PackedValue.__init__(self, value, "H")

class float_t(PackedValue):
    def __init__(self, value):
        PackedValue.__init__(self, value, "f")

class pString_t(PackedValue):

    def __init__(self, value):
        self.value = value

    def pack(self):
        return struct.pack("<%dp" % (len(self.value)+1), self.value)

####################################################################################################

