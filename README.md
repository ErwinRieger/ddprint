ddprint
=======


Ultimaker direct drive firmware with host preprocessing.

This is a experimental firmware for Arduino Mega based 3D printers. I use it to print on a Ultimaker2 over USB, 
the slicer used is Cura.

The software is split into two main parts:

* The host part where the cpu intensive work (gcode preprocessing, path planning, lookahead, acceleration...) is done.
  The host part is written in Python.
* And the firmware part that runs on the ATMega Controller. This part executes the move commands from
  the host and does the temperature control of the printer.

The preprocessed move commands are sent through the USB connection between host and printer. The SD card is used to
buffer the received data in the firmware (i call it 'SwapDevice'). The SD card is used in 'raw mode' without a
filesystem on it - an existing filesystem and its contents will be erased!

A simple form of a "AutoTemp" algorithm is implemented, the hotend temperature is increased for parts of the
model where high printing speeds are reached.


Links
-----

Here are some links where i got ideas and inspiration from:

* https://github.com/Ultimaker/Ultimaker2Marlin, https://github.com/MarlinFirmware/Marlin
* https://github.com/benhoyt/protothreads-cpp
* https://github.com/makerbot/s3g
* https://github.com/Traumflug/Teacup_Firmware
* https://github.com/ambrop72/aprinter
* http://www.repetier.com/documentation/repetier-firmware/
* https://github.com/triffid/SoupCup_Firmware
* https://github.com/JustAnother1/Pacemaker
* https://github.com/JustAnother1/pmc
* http://elm-chan.org/docs/mmc/mmc_e.html
* http://www.hwml.com/leibramp.pdf
* http://www.embedded.com/design/mcus-processors-and-socs/4006438/Generate-stepper-motor-speed-profiles-in-real-time

