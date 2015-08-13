#!/usr/bin/env python 
# encoding: utf-8 
#
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

#
# Compare the s3d settings of a gcode file with a s3d profile (*.fff).
#


import sys
from lxml import etree

skipTokens = ["processName", "applyToModels", "profileName",
        "profileVersion", ]

def skipLine(key):

    for t in skipTokens:
        if t in key:
            # print "skipping key:", key
            return True
    return False

def checkProfileKV(root, key, values=None):

    el = root.find(".//%s" % key)

    # print "found: ", key, el, el.tag, el.text, values

    if values == None:
        return True

    if el.text == values:
        return True

    print "\nValue '%s': '%s' has a different value in the profile: '%s'" % (key, values, el.text)
    return True

def findNodeByAttr(nodes, attname, attvalue, attname2=None, attvalue2=None):

    for node in nodes:
        if node.get(attname) == attvalue:
            if attname2:
                if node.get(attname2) == attvalue2:
                    return node
                else:
                    print "\nValue \"%s %s='%s'\": attribute \"%s='%s'\" has a different value in the profile: '%s'." % (key, attname, attvalue, attname2, node.get(attname2), attvalue2)
                    return True
            return node

    assert(0) # print "\nElement '%s': '%s' has a different value in the profile: '%s'" % (key, values, el.text)
    return None

f = open(sys.argv[1])

tree = etree.parse(open(sys.argv[2]))
root = tree.getroot()
# print "root: ", root

extruderNodes = root.findall("extruder[@name]")
extruderNode = None

tempNodes = root.findall("temperatureController[@name]")
tempControllers = []
tempSetPoints = []
tempMapping = {
        "temperatureNumber": "temperatureNumber",
        "temperatureStabilizeAtStartup": "stabilizeAtStartup",
        "temperatureHeatedBed": "isHeatedBed",
        "temperatureRelayBetweenLayers": "relayBetweenLayers",
        "temperatureRelayBetweenLoops": "relayBetweenLoops",
        }

fanSpeedNode = root.find("fanSpeed")
fanLayers = []

print "\n*****"
print "Comparing %s with %s:" % (sys.argv[1], sys.argv[2])
print "*****"
skip = True
for line in f.readlines():

    if line.startswith(";"):

        # print "comment line:", line

        kv = line[1:].strip().split(",")
        key = kv[0]

        if key == "Settings Summary":
            skip = False
            continue

        if skip:
            continue

        if skipLine(key):
            continue

        if len(kv) > 1 and kv[1] != "":
            values = ",".join(kv[1:])
            # print "kv '%s' '%s'" % (key, values)

            if key == "extruderName":
                extruderName = values
                extruderNode = findNodeByAttr(extruderNodes, "name", extruderName)
                continue

            if key == "temperatureName":
                for controllerName in values.split(","):
                    controller = findNodeByAttr(tempNodes, "name", controllerName)
                    tempControllers.append(controller)
                    tempSetPoints.append({"count":0, "layers": []})
                continue

            if key.startswith("extruder"):
                key = key[8].lower() + key[9:]
                if checkProfileKV(extruderNode, key, values):
                    continue

            if key in tempMapping:
                for (controller, value) in zip(tempControllers, values.split(",")):
                    assert(checkProfileKV(controller, tempMapping[key], value))
                continue

            if key == "temperatureSetpointCount":
                i = 0
                for spc in values.split(","):
                    tempSetPoints[i]["count"] = int(spc)
                    i += 1
                continue

            if key == "temperatureSetpointLayers":
                i = 0
                count = tempSetPoints[i]["count"]
                for l in values.split(","):
                    tempSetPoints[i]["layers"].append(l)
                    count -= 1
                    if count == 0:
                        i += 1
                continue

            if key == "temperatureSetpointTemperatures":

                i = 0
                l = 0

                count = tempSetPoints[i]["count"]
                for temp in values.split(","):

                    assert(findNodeByAttr(
                        tempControllers[i], 
                        "layer", tempSetPoints[i]["layers"][l],
                        "temperature", temp) != None)

                    count -= 1
                    l += 1
                    if count == 0:
                        i += 1
                        l = 0
                continue

            if key == "fanLayers":
                for l in values.split(","):
                    fanLayers.append(l)
                continue

            if key == "fanSpeeds":

                for (l, s) in zip(fanLayers, values.split(",")):
                    assert(findNodeByAttr(
                        fanSpeedNode, 
                        "layer", l,
                        "speed", s) != None)

                continue

            if key == "extrusionMultiplier":
                if checkProfileKV(extruderNode, key, values):
                    continue

            if checkProfileKV(root, key, values):
                continue
        else:
            # print "key '%s'" % key
            if checkProfileKV(root, key):
                continue

        print "\nkey not processed:", key
        assert(0)

    else:

        # stop on first non-comment line:
        print "done..."
        break

