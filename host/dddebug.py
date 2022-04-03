#/*
# This file is part of ddprint - a 3D printer firmware.
# 
# Copyright 2020 erwin.rieger@ibrieger.de
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

import json

from ddconfig import debugTypes

#
# Some type checking helpers, can be turned on/off with ddconfig.py:debugTypes
#
def assertType(value, typ):

    if debugTypes:
        assert(type(value) == typ)

#
# Log data in json format
#
class JsonLogger:

    def __init__(self, args):

        self.f = open(args.logat, "w")

        argd = vars(args)

        self.f.write("{\n")
        self.f.write('  "args": ')
        self.f.write(json.dumps(argd, indent=4))

    def __del__(self):
        self.f.write('}\n')
        self.f.close()

    def log(self, s):
        self.f.write(s)

    def dumps(self, d, indent=0):
        self.f.write(json.dumps(d, indent=indent))



