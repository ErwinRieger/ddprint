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

import ddprintutil as util, logging

# from __future__ import print_function

#
# Dumb console tui interface
#
class DumbGui:

    def log(self, *args):
        logging.info(util.stringFromArgs(*args))

    def logComm(self, *args):
        logging.info("CommLog: %s", util.stringFromArgs(*args))

    def logError(self, *args):
        logging.error(util.stringFromArgs(*args))

    def tempCb(self, t0=None, t1=None, targetT1=None):
        pass

    def statusCb(self, status):
        pass

    def logPrintLog(self, s):
        logging.info("PrintLog: %s" % s)



