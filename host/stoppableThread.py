# encoding: utf-8 
#
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


import threading, ctypes
import sys

class StopThread(Exception): pass

class StoppableThread(threading.Thread):

    def __init__(self, target):
        threading.Thread.__init__(self, target=target)
        self.threadStopCount = 0

    def xrun(self):

        print("ident: ", self.ident)

        while True:
            try:

                i=0
                while True:
                    print((time.time() % 1))
                    time.sleep(0.1)
                    i += 1

                    if i>100:
                        return

            except StopThread:
                print("caught StopThread, continue....")
                self.threadStopCount += 1

    def stop(self):

        stopCount = self.threadStopCount

        ret = ctypes.pythonapi.PyThreadState_SetAsyncExc(ctypes.c_long(self.ident), ctypes.py_object(StopThread))
        # ref: http://docs.python.org/c-api/init.html#PyThreadState_SetAsyncExc
        if ret == 0:
            raise ValueError("Invalid thread ID")
        elif ret > 1:
            # Huh? Why would we notify more than one threads?
            # Because we punch a hole into C level interpreter.
            # So it is better to clean up the mess.
            # ctypes.pythonapi.PyThreadState_SetAsyncExc(target_tid, NULL)
            ctypes.pythonapi.PyThreadState_SetAsyncExc(ctypes.c_long(self.ident), NULL)
            raise SystemError("PyThreadState_SetAsyncExc failed")

        print("StoppableThread.stop(): Successfully set asynchronized exception for", self.ident)

        while self.threadStopCount == stopCount:
            print("StoppableThread.stop(): wait for stop...")
            time.sleep(1)

    def incStopCount(self):
        self.threadStopCount += 1

################################################################################

import time

def main():
    test = StoppableThread()
    test.start()

    for i in range(10):
        time.sleep(1)
        test.stop()

if __name__ == '__main__':
    main()






