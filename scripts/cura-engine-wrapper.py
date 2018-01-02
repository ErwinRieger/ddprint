#!/usr/bin/env python3
# encoding: utf-8 
#
#/*
# This file is part of ddprint - a direct drive 3D printer firmware.
# 
# Copyright 2016 erwin.rieger@ibrieger.de
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
#
# This program needs a CURA (github.com/Ultimaker/Cura) installation (2.3.x) 
# and uses its python modules.
#
#*/

#
# Collect CuraEngine settings from Cura Machine- and Qualityprofiles and call
# CuraEngine to produce a gcode file from a stl input.
#
# See also: http://ibrieger.de/cura-engine-wrapperpy-helper-to-call-curaengine-from-command-line.html
#

import os, copy, sys, subprocess
import urllib.parse

debug = False

# XXX todo: use proper python argparse module here, add "-d" debug option
if len(sys.argv) < 3 or sys.argv[1] == "-h":
    print("Usage:")
    print("  %s -h: print help" % os.path.basename(sys.argv[0]))
    print("  %s <curapath> -l: list all container names" % os.path.basename(sys.argv[0]))
    print("  %s <curapath> -pm: <machine name>: print values of machine container" % os.path.basename(sys.argv[0]))
    print("  %s <curapath> -pp: <profile name>: print values of profile" % os.path.basename(sys.argv[0]))
    print("  %s <curapath> <machine name> <profile name> <input-stl> <output-gcode>: Collect settings from profiles and call curaEngine to produce gcode ouput from stl." % os.path.basename(sys.argv[0]))
    sys.exit(0)

from UM.Application import Application
from UM.Resources import Resources
from UM.Settings.SettingDefinition import SettingDefinition, DefinitionPropertyType
from UM.Settings.Validator import Validator
from UM.Settings.SettingFunction import SettingFunction
from UM.PluginRegistry import PluginRegistry
from UM.Logger import Logger

import cura.Settings

curaDir = sys.argv[1]

# Send log messages of cura to stderr
class DummyLogger:

    def log(self, log_type, message):
        print("Log:", log_type, message, file=sys.stderr)

# Dummy cura application
class DummyApp(Application):

    class ResourceTypes:
        QmlFiles = Resources.UserType + 1
        Firmware = Resources.UserType + 2
        QualityInstanceContainer = Resources.UserType + 3
        MaterialInstanceContainer = Resources.UserType + 4
        VariantInstanceContainer = Resources.UserType + 5
        UserInstanceContainer = Resources.UserType + 6
        MachineStack = Resources.UserType + 7
        ExtruderStack = Resources.UserType + 8

    def __init__(self):
        Application.__init__(self, "cura", "2.3.1")

    def _addProfileReader(self, profile_reader):
        pass

    def parseCommandLine(self):
        pass

def skipCurrentUserSettings(cr):

    # Remove current user settings by cleaning them out. XXX Better way to do this? Some sort of filter?
    containers = cr.findInstanceContainers(type = "user") # , machine = stack.getId())
    for container in containers:

        if debug:
            print("Skipping user settings container:", container.getName())
        container.clear()

dummyApp = DummyApp()

Logger.addLogger(DummyLogger())

plugin_registry = PluginRegistry.getInstance()
plugin_registry.setApplication(dummyApp)
plugin_registry.addPluginLocation(os.path.join(sys.argv[1], "lib", "cura", "plugins"))
plugin_registry.addType("profile_reader", dummyApp._addProfileReader)
# plugin_registry.loadPlugins()
plugin_registry.loadPlugin("XmlMaterialProfile")
plugin_registry.loadPlugin("CuraProfileReader")

Resources.addSearchPath(os.path.join(sys.argv[1], "share", "cura", "resources"))
Resources.addSearchPath(os.path.join(os.environ["HOME"], ".local", "share", "cura"))

Resources.addStorageType(dummyApp.ResourceTypes.QualityInstanceContainer, "quality")
Resources.addStorageType(dummyApp.ResourceTypes.VariantInstanceContainer, "variants")
Resources.addStorageType(dummyApp.ResourceTypes.MaterialInstanceContainer, "materials")
Resources.addStorageType(dummyApp.ResourceTypes.UserInstanceContainer, "user")
Resources.addStorageType(dummyApp.ResourceTypes.ExtruderStack, "extruders")
Resources.addStorageType(dummyApp.ResourceTypes.MachineStack, "machine_instances")

SettingDefinition.addSupportedProperty("settable_per_mesh", DefinitionPropertyType.Any, default = True, read_only = True)
SettingDefinition.addSupportedProperty("settable_per_extruder", DefinitionPropertyType.Any, default = True, read_only = True)
SettingDefinition.addSupportedProperty("settable_per_meshgroup", DefinitionPropertyType.Any, default = True, read_only = True)
SettingDefinition.addSupportedProperty("settable_globally", DefinitionPropertyType.Any, default = True, read_only = True)
SettingDefinition.addSupportedProperty("limit_to_extruder", DefinitionPropertyType.Function, default = "-1")
SettingDefinition.addSupportedProperty("resolve", DefinitionPropertyType.Function, default = None)
SettingDefinition.addSettingType("extruder", None, str, Validator)

SettingFunction.registerOperator("extruderValues", cura.Settings.ExtruderManager.getExtruderValues)
SettingFunction.registerOperator("extruderValue", cura.Settings.ExtruderManager.getExtruderValue)
SettingFunction.registerOperator("resolveOrValue", cura.Settings.ExtruderManager.getResolveOrValue)

cr = cura.Settings.CuraContainerRegistry.getInstance()
cr.setApplication(dummyApp)

cr.addResourceType(dummyApp.ResourceTypes.QualityInstanceContainer)
cr.addResourceType(dummyApp.ResourceTypes.VariantInstanceContainer)
cr.addResourceType(dummyApp.ResourceTypes.MaterialInstanceContainer)
cr.addResourceType(dummyApp.ResourceTypes.UserInstanceContainer)
cr.addResourceType(dummyApp.ResourceTypes.ExtruderStack)
cr.addResourceType(dummyApp.ResourceTypes.MachineStack)

empty_container = cr.getInstance().getEmptyInstanceContainer()

empty_variant_container = copy.deepcopy(empty_container)
empty_variant_container._id = "empty_variant"
empty_variant_container.addMetaDataEntry("type", "variant")
cr.addContainer(empty_variant_container)

empty_material_container = copy.deepcopy(empty_container)
empty_material_container._id = "empty_material"
empty_material_container.addMetaDataEntry("type", "material")
cr.addContainer(empty_material_container)

empty_quality_changes_container = copy.deepcopy(empty_container)
empty_quality_changes_container._id = "empty_quality_changes"
empty_quality_changes_container.addMetaDataEntry("type", "quality_changes")
cr.addContainer(empty_quality_changes_container)

cr.load()

if sys.argv[2] == "-l":

    print("# containers:", len(cr.findContainers()))
    for container in cr.findContainers():
        print("container: ", container.getName(), container.getMetaDataEntry("type"))

    """
    print("# container stacks:", len(cr.findContainerStacks()), cr.findContainerStacks())
    """

    sys.exit(0)

if sys.argv[2] == "-pm":

    machProfile = sys.argv[3]

    skipCurrentUserSettings(cr)

    machine = cr.findContainerStacks(type = "machine", name = machProfile)[0]
    dummyApp.setGlobalContainerStack(machine)

    print("\nMachine container:\n")
    for key in machine.getAllKeys():
        print(key, "=", machine.getProperty(key, "value"))

    sys.exit(0)

if sys.argv[2] == "-pp":

    qualityProfile = sys.argv[3]

    profiles = cr.findInstanceContainers(type = "quality_changes", name = qualityProfile)
    if not profiles:
        profiles = cr.findInstanceContainers(type = "quality", name = qualityProfile)
    profile = profiles[0]

    print("\nProfile container:\n")
    for key in profile.getAllKeys():
        print(key, "=", profile.getProperty(key, "value"))

    sys.exit(0)

machProfile = sys.argv[2]
qualityProfile = sys.argv[3]
inputFile = sys.argv[4]
outputFile = sys.argv[5]
additionalSettings = {}

if len(sys.argv) > 6 and sys.argv[6]:
    additionalSettings = eval(sys.argv[6])

allValues = {}

skipCurrentUserSettings(cr)

machine = cr.findContainerStacks(type = "machine", name = machProfile)[0]
dummyApp.setGlobalContainerStack(machine)

for key in machine.getAllKeys():

    value = machine.getProperty(key, "value")

    # XXX todo: use resolved values here
    # The "resolved" value of a setting is the value that should be used when two extruders have a conflicting value.
    resolved = machine.getProperty(key, "resolve")

    if debug and resolved != None and resolved != value:
        print(key, "resolved:", resolved, value)
        assert(0)

    allValues[key] = value

profile = cr.findInstanceContainers(type = "quality_changes", name = qualityProfile)[0]

for key in profile.getAllKeys():

    # XXX todo: use resolved values here
    # The "resolved" value of a setting is the value that should be used when two extruders have a conflicting value.
    if debug:
        assert(profile.getProperty(key, "resolve") == None)

    allValues[key] = profile.getProperty(key, "value")

for key in additionalSettings:

    allValues[key] = additionalSettings[key]

# fdmprinter = cr.findDefinitionContainers(type = "machine", name = "FDM Printer Base Description")[0]

engineArgs = [ 
        os.path.join(curaDir, "bin", "CuraEngine"),
        "slice",
        "-j", os.path.join(curaDir, "share", "cura", "resources", "definitions", "fdmprinter.def.json"),
        ]

for key in allValues:

    value = allValues[key]

    """
    # Output changed values only
    if fdmprinter.hasProperty(key, "value"):

        fdmprinterValue = fdmprinter.getProperty(key, "value")

        if isinstance(fdmprinterValue, SettingFunction):
            fdmprinterValue = fdmprinterValue(machine)

        if fdmprinterValue != value:
            engineArgs.append("-s")
            engineArgs.append("%s=%s" % (key, value))
    else:
        engineArgs.append("-s")
        engineArgs.append("%s=%s" % (key, value))
    """

    engineArgs.append("-s")
    engineArgs.append("%s=%s" % (key, value))


engineArgs += [
        "-s", "center_object=true",
        "-o", outputFile,
        "-l", inputFile
        ]

if debug:
    for key in sorted(allValues.keys()):
        print(key, allValues[key])
    print("engineArgs:", engineArgs)

# subprocess.call(engineArgs)








