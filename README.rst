
.. raw:: html

   <link rel="stylesheet" href="/images/js_css/asciinema-player.css"" type="text/css"/>

ddPrint - Adding Process control to FDM printers
=================================================

:tags: FDM, 3DDruck, 3dprinting, python, ddprint
:slug: ddprint-3d-printer-firmware

.. image:: /images/um2-1/20200421_212447.jpg
   :width: 75px
   :target: /images/um2-1/20200421_212447.jpg

.. image:: /images/flowsensor_red.jpg
   :width: 150px
   :target: /images/flowsensor_red.jpg

**Note: Experimental and work in progress, documentation is incomplete and outdated.**


Overview
++++++++++

..
   XXX LEVEL 1 XXX

DDPrint adds process control to FDM printer.

The process that is to be controlled is the process of extruding plastic.

This includes the feeder, a bowden tube (*) and the hotend with its heater, lets call
this the *extruder*.

Most FDM printers have no feedback on the extruder part of the printer - they are running *open loop*.

That means to find the balance of printing speed (better: volumetric flow) and hotend temperature
is difficult. Another problem of this *open loop* extrusion system is that the printer cannot react when the
hotend has difficulties to deliver the current demand of flow (for example when printing the first layer or when
the hotend is not hot enough yet).

To close the control loop we add a `sensor <#flowratesensor-label>`__ to measure the performance of the extruder
und use this data to control the temperature of the hotend and the speed of the printer.

The control loop is closed in two ways, an acvitve way where the speed of the printer is lowered if the hotend cannot 
deliver the demanded flowrate and the feeder begins to slip.
The other is a *feed forward* way: The flowrate sensor is used to measure the characteristics of a given filament - extruder
combination. The result of this measurement is a so called `*material profile* <#matprofile-label>`__.

Control of hotend temperature is called `atutotemp <#autotemp-label>`__.

Control of printer speed is done with the `temperature limiter <#templimiter-label>`__ and the `flowrate limiter <#flowratelimiter-label>`__
.






(*) For the moment `bowden style printers only. <#bowdenstyle-label>`__

.. contents::

..
   XXX LEVEL 2 XXX

.. _flowratesensor-label:

Flowratesensor
++++++++++++++

.. image:: /images/flowsensor_red.jpg
   :width: 100px
   :target: /images/flowsensor_red.jpg

.. image:: /images/flowratesensor/flowsensor1.jpg
   :width: 50
   :target: /images/flowratesensor/flowsensor1.jpg

.. image:: /images/ender5/flowrate_sensor_ender5.jpg
   :width: 80px
   :target: /images/ender5/flowrate_sensor_ender5.jpg

.. _bowdenstyle-label:

Bowden style printers only?
+++++++++++++++++++++++++++

For best results, the flowrate sensor has to be placed *after* the feeder. This is easy for a bowden style printer.

For direct driver printers it should be possible but it is more difficult to add the flowrate sensor into the print head (because of
space requirements, heat, added weight and so on).

.. _matprofile-label:

Material Profiles
++++++++++++++++++

TBD

.. _autotemp-label:

Auto Temp
+++++++++++++

TBD

.. _templimiter-label:

Temperature limiter
++++++++++++++++++++

TBD

.. _flowratelimiter-label:

Flowrate limiter
+++++++++++++++++

TBD

..
   XXX LEVEL 3 XXX

---------------------------------------------------------------------------------------------

** Following stuff has to be reviewed **


FDM 3d printer firmware with host preprocessing and *closed loop E*
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

See last chapter in this document for a log/history.

.. image:: /images/flowsensor_red.jpg
   :width: 200px
   :target: /images/flowsensor_red.jpg

3d printer firmware for cartesian FDM printers (ultimaker clones, atmega and stm32, like um2, ramps or jennyprinter).

This firmware is part of the *ddprint printing system*, which immproves a common FDM printer by a closed loop E-Axis: http://www.ibrieger.de/close_the_loop_for_e.html.

Reference printer is a Ultimaker 2 with ddprint installed and some hardware
modifications: http://www.ibrieger.de/pimped-ultimaker-2.html, and a jennyprinter X340 now too.

Github Mirror, project Homepage
++++++++++++++++++++++++++++++++

DDPrint FDM firmware: `github.com/ErwinRieger/ddprint <http://github.com/ErwinRieger/ddprint>`_, mirrored here: http://www.ibrieger.de/ddprint-3d-printer-firmware.html.

DDprint system project homepage: `ibrieger.de/close_the_loop_for_e.html <http://www.ibrieger.de/close_the_loop_for_e.html>`_.

Video: https://youtu.be/1Kbl9AZd10Y, ddprint playlist: https://www.youtube.com/playlist?list=PLzn7lnnZpS7XP-JhLw_o7p27ayv5bJ29o.

HackadayIO project: https://hackaday.io/project/170695-ddprint.

Current state
+++++++++++++++++++++++++++++++

Works for me, experimental.

Update Mar 31 2021:

Merged *next branch* into master with the following changes:

* Feed forward temp control (*PID hinting*, *guided PID*) :-o
  AutoTemp algorithm: set new temperature some time before the new flowrate demand, 
  using the information from the material profile.
* Added a experimental gcode (M901) for part strength (along with new *workingpoint* command line parameter) :-o
* Asymmetric PID temperature control: switch between different PID control sets for
  heating (fast) and cooling (a slower PID set to avoid temperature underruns).
* Deflate/zip data downloaded to printer to increase download speed over usb-serial.
  Using python's zlib on the host and uzlib (https://github.com/pfalcon/uzlib) on
  the firmware side.
* Added a second step to material profile measurement (measuretempflowratecurve2):
  Measure hotend performance while doing a real print, in addition to the *into-the-air* extrusion measurement.
* Some smaller improvements:
   + Moved printer-, nozzle- and material-profiles into their own repository (https://github.com/ErwinRieger/ddprint-profiles).
   + Show print time in CLI and terminal UI (TUI).
   + Show the number of *under-temperature* and *under-grip* warnings in TUI.
   + Added a *printlog* to ddprintui.py: log information about the current printjob (gcode info, timestamp,
     printing time, slicersettings).
   + Added some tools and scripts (plot_mat_profile, ddtool, wrapperscripts).
   + Removed usage of eeprom, configuration now stored on mass storage device (sdcard, usb).
   + And many more...

Key features
+++++++++++++

* Uses a incremental sensor to measure extruder flowrate at realtime
  to "close the loop for E". This limits the speed of the printer
  if feeder tends to slip.
* Automatic measurement of feeder system caracteristics for calibration.
* Automatic measurement of filament caracteristics to create filament
  profiles .
* "Auto temperature algorithm": hotend temperature depends on the
  gcode-requested flowrate. 
* "Temperature-flowrate limiter": speed of printer is limited if hotend
  has not (yet) the right temperature for the requested flowrate.
* Extruder pressure advance, of course ;-)

Main part is the Flowrate Sensor realized with a Bourns EMS22AFS incremental
encoder: http://www.ibrieger.de/pimped-ultimaker-2.html#feeder-flowratesensor.

Installation
+++++++++++++

Firmware part
-------------

Requirements
************

* Arduino code/libraries and avr compiler
* Arduino-Makefile
* SdCard library
* Protothreads header

Install them along the checked out ddprint sources so that the
directory structure looks like this:

.. code-block:: sh

    .
    ├── ddprint
    │   ├── LICENSE
    │   ...
    ├── arduino-1.6.13
    │   ├── arduino
    │   ...
    ├── Arduino-Makefile
    │   ├── Arduino.mk
    │   ...
    ├── SdFat-1.0.5
    │   ...
    │   └── src
    ├── protothreads-cpp
    │   ├── LICENSE.txt
    │   ...

The following versions are used at the moment (maybe newer versions will also work):

.. code-block:: sh

    Arduino IDE 1.6.13, installed from downloaded archive.

    Arduino-Makefile from https://github.com/sudar/Arduino-Makefile.git
        commit c3fe5dcc2fbd5c895b032ca5a5a1f60af163b744
        Merge: 7a26a86 6d3d973
        Author: Simon John <git@the-jedi.co.uk>
        Date:   Thu Dec 28 18:05:18 2017 +0000

    SdFat-1.0.5, installed from downloaded zip archive (downloads.arduino.cc/libraries/github.com/greiman/SdFat-1.0.5.zip),
    apply ddprint/patches/SdFat-1.0.5.patch.

    Protothreads from https://github.com/benhoyt/protothreads-cpp.git
        commit 984aa540dd4325b7e23dc76135ca28a36526f0c6
        Author: Ben Hoyt <benhoyt@gmail.com>
        Date:   Tue Dec 4 16:48:52 2018 -0500

        Apply ddprint/patches/protothreads-cpp.patch


Build and upload firmware
***************************

:Note: keep a backup of your previous firmware in case you want to go back.
:Note: ddPrint does not use EEProm.

For a ultimaker UM2 do:

.. code-block:: sh

    make -f Makefile.fw
    make -f Makefile.fw do_upload

For a ramps based printer do:

.. code-block:: sh

    make -f Makefile.ramps
    make -f Makefile.ramps do_upload

:Todo: Add info about configuration.

Host part
-------------

Requirements:

* apt-get install python python-serial
* pip install npyscreen vor the TUI (ddprintui.py)

No installation procedure yet, checkout the repository and run *ddprint.py* or *ddprintui.py* from the
*ddprint/host* subdirectory.

Configuration
+++++++++++++

Parts of printer configuration hardcoded in firmware, parts come from printer profile at runtime.

:Todo: describe configuration.

Only one setting stored in printer: the printer name.

:Todo: describe printer name setting.


Gcode input
+++++++++++++

Easier slicing, simple gcode
-----------------------------

:Note: Simplify3d is used as of this writing.

Use mostly plain gcode with ddprint, many of the *advanced features* of the slicers (i call it *slicer hacks*) are not
needed, see http://www.ibrieger.de/close_the_loop_for_e.html#simpler-gcode.

The (automatically measured) material profile gives a picture of the hotend melting capacity for a given machine/filament combination.
This eases the determination of a good printing speed. 

Simplify3d example slicer settings in https://github.com/ErwinRieger/ddprint/tree/master/examples/s3d_profiles.

Supported gcodes
-----------------

*Todo*

Usage, Commandline Interface
++++++++++++++++++++++++++++++++++

Machine setup and calibration
------------------------------

*setPrinterName*
**************************************

Store printer name in printer's runtime config (on mass storage device):

.. code-block:: sh

    ./ddprint.py setPrinterName UM2-1

See also: `getprintername command <#getprintername-label>`__.

.. _calibrateesteps-label:

*getEndstops*
**************************************

Get current endstop state.

.. code-block:: sh

    ./ddprint.py getEndstops

*calibrateESteps*
**************************************

Machine setup and calibration: Automatically determine extruder *e-steps* value for printer profile:

.. code-block:: sh

    ./ddprint.py calibrateesteps

Example screencast:

.. raw:: html

    <asciinema-player src="/images/video/calestep.asc" font-size="8"></asciinema-player>  
    <noscript>
       <a href="http://www.ibrieger.de/ddprint-3d-printer-firmware.html#calibrateesteps-label">Asciinema not available on github, see video here.</a>
    </noscript>

.. _calflowratesensor-label:

*calibrateFilSensor*
**************************************

Machine setup and calibration: Automatically determine flowrate sensor calibration value for printer profile:

.. code-block:: sh

    ./ddprint.py calibratefilsensor

Example screencast:

.. raw:: html

    <asciinema-player src="/images/video/calflowratesensor.asc" font-size="8"></asciinema-player>  
    <noscript>
       <a href="http://www.ibrieger.de/ddprint-3d-printer-firmware.html#calflowratesensor-label">Asciinema not available on github, see video here.</a>
    </noscript>

*testFilSensor*
**************************************

Test *e-steps* and flowrate sensor calibration:

.. code-block:: sh

    ./ddprint.py testFilSensor UM2-2 100

Autotune hotend PID, *autoTune*
**************************************

Run PID autotune to determine the hotend PID parameters:

.. code-block:: sh

    ./ddprint.py autoTune petg_1.75mm
    cd pid_tune
    PYTHONPATH=.. ./pidAutoTune.py ../autotune.raw.json

*bedLeveling*
**************************************

:Todo: describe command

Material measurment
------------------------------

Measure material profile, *measureTempFlowrateCurve*
*********************************************************

Extrude some filament into air and measure the material properties (melting capacity, temperatures)
of this machine/filament combination.

.. code-block:: sh

    ./ddprint.py measureTempFlowrateCurve nozzle80 petg_1.75mm 2.5

Printing
------------------------------

*home*
**************************************

.. code-block:: sh

    ./ddprint.py home

*insertFilament*
**************************************

Heat hotend and start filament insertion process.

.. code-block:: sh

    ./ddprint.py removeFilament petg_1.75mm

*removeFilament*
**************************************

Heat hotend and pull back/remove filament.

.. code-block:: sh

    ./ddprint.py removeFilament petg_1.75mm

Preprocess gcode file, *pre*
**************************************

Preprocess a gcode file, this parses the given gcode file and runs all processing steps without actually 
sending anything to the printer. Used for development, debugging and to check if a given gcode file can be 
processed by ddprint.

.. code-block:: sh

    ./ddprint.py  -smat esun_petg_transparent-orange-6-922572-263079 pre UM2-1 nozzle80 petg_1.75mm quader_10x20.gcode

Print gcode file, *print*
**************************************

Print a gcode file with the commandline tool:

.. code-block:: sh

    ./ddprint.py  -smat esun_petg_transparent-orange-6-922572-263079 print nozzle80 petg_1.75mm quader_10x20.gcode

Misc commands
------------------------------

*disableSteppers*
**************************************

Switch off stepper current, printer no longer homed after that.

.. code-block:: sh

    ./ddprint.py disableSteppers

Manual movement, *moverel*
**************************************

Move axis relative to current position.

.. code-block:: sh

    ./ddprint.py moverel X 100

Manual movement, *moveabs*
**************************************

Move axis to absolute position.

.. code-block:: sh

    ./ddprint.py moveabs X 0

*heatHotend*
**************************************

:Todo: describe command

*getFilSensor*
**************************************

Get current position of filament sensor

.. code-block:: sh

    ./ddprint.py getFilSensor

*getFreeMem*
**************************************

Get current printer free memory.

.. code-block:: sh

    ./ddprint.py getFreeMem

*getpos*
**************************************

Get current printer positions.

.. code-block:: sh

    ./ddprint.py getpos

.. _getprintername-label:

Read printer name from printer, *getPrinterName*
**************************************************

Read printer name from printer, stored on mass storage device (sdcard, usbmemory).

.. code-block:: sh

    ./ddprint.py getprintername

*getTemps*
**************************************

Get bed- and hotend temperatures from printer.

.. code-block:: sh

    ./ddprint.py getTemps

*getStatus*
**************************************

.. code-block:: sh

    ./ddprint.py getStatus



Usage, Userinterface (TUI)
+++++++++++++++++++++++++++++++++++++++++++++++++

Print a gcode file with the TUI:

.. code-block:: sh

    ./ddprintui.py  -smat esun_petg_transparent-orange-6-922572-263079 nozzle80 petg quader_10x20.gcode

Some implementation notes
++++++++++++++++++++++++++++

Host side preprocessing and stepgeneration
----------------------------------------------

The software is split into two main parts:

* The host part where the cpu intensive work (gcode preprocessing, path planning, lookahead, acceleration, advance...) is done.
  The host part is written in Python.
* And the firmware part that runs on the ATMega Controller in the printer. This part executes the move commands from
  the host and does other things like the temperature control of the printer.
* Host software and printer firmware are connected through the usual atmega rs232 USB emulation.

Use printers SD card as a *swap device*
----------------------------------------------

To overcome the limited memory of the atmega, the unused SD card is converted to something like a *swap device*: It buffers the received data. This decouples the USB transfer
and actual use of the received data, too.
The SD card is used in 'raw/blockwise mode' without a filesystem on it.

Working SD cards
*******************

Not all cards are working in SPI mode, some fail to initialize, some freeze after some time. See https://github.com/greiman/SdFat/issues/160, also.

Some working ones:

* The ones that come with your printer should work.
* SandDisk, 2Gb, SD
* SandDisk, 2Gb, Micro-SD
* SandDisk, 4Gb, SDHC, Class 2
* SandDisk, 4Gb, SDHC, Class 4

Not working ones:

* MediaRange, 4Gb, SDHC, Class 10

AutoTemp
-----------

AutoTemp algorithm: the hotend temperature is increased for parts of the model where high printing speeds are reached and vice-versa.

Protothreads
------------

The firmware part is implemented using the great *protothreads* library: http://github.com/benhoyt/protothreads-cpp, thanks for this work.

Other 
------

* Look ahead path planning with linear acceleration ramps.
* Hardened USB communication using COBS encoding and CCITT checksums in BOTH directions.
* Simulator mode for testing/development: Firmware runs as a host-program with serial communication over a ptty device.
* Debugging: plot/display generated acceleration ramps

Things todo, nice to have
++++++++++++++++++++++++++++

* Improve documentation, examples, videos.
* Cleanup and stabilisation, make binary releases.
* Python3 port (currently python 2.7).
* Other convenient things like automatic bedleveling and so on.

Thanks
+++++++++++++

Thanks to all open/free software people that make this all possible.  


Log
++++++++++++++++++++++++++++

::

   Fri Jun 25 09:49:51 CEST 2021

   Merged *next* branch into master, changes are:

   * Flowrate/grip measurement:
   * Added a experimental reprap usbserial interface using a pseudo-tty to use OctoPrint 
     as a frontend for ddPrint. Not much functionality yet: display temperatures and some
     SD card commands (stubs).
   * Flowrate limiter: max. slowdown now four times instead of 16.

   Wed Jun 16 23:47:01 CEST 2021

   Current development (cleanup, minor fixes, documentation) is done on *next* branch.
   Adding asciinema screencast to show how to use ddPrint.

   Wed Jun  9 21:17:22 CEST 2021

   Merged *fix-avr* branch into master.

   Tue Jun  8 14:41:41 CEST 2021

   Pushed fix-avr branch to github. The JennyPrinter port made the avr/atmega side to slow.
   Changes are:

   * Integer math instead of floating point.
   * Reworked usb-serial interface: store 512byte blocks.
   * Removed compression with zlib, the avr has not enough cpu cycles.
   * Experiment: auto-baudrate. Switch between 1000000, 500000 and 250000 baud.
   * SDReader: double-buffering.
   * Many other improvements and cleanup.

   So for now, fix-avr is the branch to use for avr/atmega based printers and master is for
   the stm32 JennyPrinter. Branch fix-avr has will be merged into master.


.. raw:: html

    <script type='text/javascript' src='/images/Widget_2.js'></script>
    <script type='text/javascript' src='/images/kofiButton.js'></script>
    <script src="/images/js_css/asciinema-player.js"></script>

