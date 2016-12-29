#!/usr/bin/env python3
#
# export PYTHONPATH="/opt/cura/lib/cura/plugins:/opt/cura/lib/python3/dist-packages"
# export LD_LIBRARY_PATH=/opt/cura/lib
# CURA_ENGINE_SEARCH_PATH
#
# Drei verschiedene container typen (ContainerRegistry.getContainerTypes()):
# * stack?
# * definition (.json)
# * InstanceContainer: instantiierung einer definition?, profile (.cfg)
#
# ~/.config/cura/cura.cfg:
#   active_machine = Ultimaker 2        -> ...home...local...cura/machine_instances/Ultimaker+2.stack.cfg
#
import os, copy, sys
import urllib.parse

from UM.Application import Application
from UM.Resources import Resources
from UM.Settings.SettingDefinition import SettingDefinition, DefinitionPropertyType
from UM.Settings.ContainerRegistry import ContainerRegistry
from UM.Settings.Validator import Validator
from UM.Settings.SettingFunction import SettingFunction
from UM.PluginRegistry import PluginRegistry
from UM.Logger import Logger

import cura.Settings

if len(sys.argv) < 3 or sys.argv[1] == "-h":
    print("Usage:")
    print("  %s -h: print help" % sys.argv[0])
    print("  %s -l: <curapath> list all container names" % sys.argv[0])
    print("  %s -pm <curapath> <machine name>: print values of machine container" % sys.argv[0])
    print("  %s -pp <curapath> <profile name>: print values of profile" % sys.argv[0])
    print("  %s <curapath> <machine name> <profile name>: print settings string for curaEngine" % sys.argv[0])
    sys.exit(0)

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

cr = ContainerRegistry.getInstance()
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
ContainerRegistry.getInstance().addContainer(empty_variant_container)

empty_material_container = copy.deepcopy(empty_container)
empty_material_container._id = "empty_material"
empty_material_container.addMetaDataEntry("type", "material")
ContainerRegistry.getInstance().addContainer(empty_material_container)

empty_quality_changes_container = copy.deepcopy(empty_container)
empty_quality_changes_container._id = "empty_quality_changes"
empty_quality_changes_container.addMetaDataEntry("type", "quality_changes")
ContainerRegistry.getInstance().addContainer(empty_quality_changes_container)

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

    print("\nMachine container:\n")

    machine = cr.findContainerStacks(type = "machine", name = sys.argv[3])[0]
    dummyApp.setGlobalContainerStack(machine)

    for key in machine.getAllKeys():
        print(key, "=", machine.getProperty(key, "value"))

    sys.exit(0)

if sys.argv[2] == "-pp":

    print("\nProfile container:\n")

    profile = cr.findInstanceContainers(type = "quality_changes", name = sys.argv[3])[0]

    for key in profile.getAllKeys():
        print(key, "=", profile.getProperty(key, "value"))

    sys.exit(0)

allValues = {}

machine = cr.findContainerStacks(type = "machine", name = "Ultimaker 2 0.80")[0]
dummyApp.setGlobalContainerStack(machine)

for key in machine.getAllKeys():

    # assert(machine.getProperty(key, "resolve") == None)
    allValues[key] = machine.getProperty(key, "value")

profile = cr.findInstanceContainers(type = "quality_changes", name = "advance_08")[0]

for key in profile.getAllKeys():

    # assert(profile.getProperty(key, "resolve") == None)
    allValues[key] = profile.getProperty(key, "value")


fdmprinter = cr.findDefinitionContainers(type = "machine", name = "FDM Printer Base Description")[0]

def listStr(value):

    if isinstance(value, list):
        s = "["
        s += ",".join(map(lambda v: str(listStr(v)), value))
        s += "]"
        return s
    else:
        return value


for key in allValues:

    value = allValues[key]

    # print("-s %s=\"%s\"" % (key, str(value).replace(" ", "\ ")), end=" ")
    # print("-s %s='%s'" % (key, listStr(value)), end=" ")
    print("-s\0%s=%s" % (key, value), end="\0")

    """
    # Output changed values only
    if fdmprinter.hasProperty(key, "value"):

        fdmprinterValue = fdmprinter.getProperty(key, "value")

        if isinstance(fdmprinterValue, SettingFunction):
            fdmprinterValue = fdmprinterValue(machine)

        if fdmprinterValue != value:
            print("-s %s=\"%s\"" % (key, value), end=" ")
    else:
        print("-s %s=\"%s\"" % (key, value), end=" ")
    """

# print()








