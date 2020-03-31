
FDM 3d printer firmware with host preprocessing.
=================================================

:tags: FDM, 3DDruck, 3dprinting, python
:slug: ddprint-3d-printer-firmware

3d printer firmware for atmega based cartesian printers (ultimaker 2 and ramps based only at this time).

Improve your FDM printer with a closed loop E-Axis: http://www.ibrieger.de/close_the_loop_for_e.html.

**Note: Experimental and work in progress.**

.. contents::

Github Mirror, project Homepage
++++++++++++++++++++++++++++++++

DDPrint FDM firmware: `http://github.com/ErwinRieger/ddprint <http://github.com/ErwinRieger/ddprint>`_.

Project homepage: `http://www.ibrieger.de/close_the_loop_for_e.html <http://www.ibrieger.de/close_the_loop_for_e.html>`_.

Video: https://youtu.be/1Kbl9AZd10Y

Features
+++++++++++++

* Closed loop E-Axis using a PMW3360 optical sensor.
* auto tempearture control
* extruder pressure advance

Installation
+++++++++++++

Requirements:

* apt-get install python python-serial
* pip install npyscreen vor the TUI (ddprintui.py)

No installation procedure yet, checkout the repository and run *ddprint.py* or *ddprintui.py* from there.

Usage
+++++++++++++

Commandline Interface, CLI
-----------------------------

:Note: todo...

Terminal Userinterface, TUI
-----------------------------

:Note: todo...

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

AutoTemp
-----------

AutoTemp algorithm: the hotend temperature is increased for parts of the model where high printing speeds are reached and vice-versa.

Other 
------

* Look ahead path planning with linear acceleration ramps.
* Hardened USB communication using COBS encoding and CCITT checksums in BOTH directions.
* Simulator mode for testing/development: Firmware runs as a host-program with serial communication over a ptty device.
* Debugging: plot/display generated acceleration ramps



Usage examples
++++++++++++++++++++++++++++

Store printer name in printer's eeprom:

.. code-block:: sh

    ./ddprint.py setPrinterName UM2-1

Run PID autotune to determine the hotend PID parameters:

.. code-block:: sh

    ./ddprint.py autoTune petg_1.75mm
    cd pid_tune
    PYTHONPATH=.. ./pidAutoTune.py ../autotune.raw.json

Print a gcode file with the TUI:

.. code-block:: sh

    ./ddprintui.py  -smat esun_petg_transparent-orange-6-922572-263079 nozzle80 petg_1.75mm /3dmodels/tests/s3d/quader_10x20.gcode

