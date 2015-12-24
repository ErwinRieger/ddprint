ddprint
=======


Ultimaker direct drive firmware with host side preprocessing.

This is a experimental firmware for Arduino Mega based 3D printers. I use it to print on a Ultimaker2 over USB, 
the slicers used are Cura, Slic3r and Simplify3D.

The main goal for this firmware is to do fast and highly detailed (circles with many segmets) prints
over USB. This is not possible with the original Marlin firmware because of the limited CPU power of the
2560 Mega board.

The software is split into two main parts:

* The host part where the cpu intensive work (gcode preprocessing, path planning, lookahead, acceleration...) is done.
  The host part is written in Python.
* And the firmware part that runs on the ATMega Controller. This part executes the move commands from
  the host and does the temperature control of the printer.

The preprocessed move commands are sent through the USB connection between host and printer. The SD card is used to
buffer the received data in the firmware (i call it 'SwapDevice'). The SD card is used in 'raw mode' without a
filesystem on it - an existing filesystem and its contents will be erased!

Some Features:

*   Interrupt based movement with linear acceleration.
*   Look ahead path planning (Keep the speed high when possible. High cornering speed).
*   Hardware and software endstop support.
*   EEPROM storage of parameters.
*   Temperature oversampling.
*   A simple form of an "AutoTemp" algorithm, the hotend temperature is increased for parts of the model where high printing speeds are reached.
*   PID auto-tuning. Measuring and plotting (gnuplot) of the hotend temperature stepresponse.
*   Simulator mode for testing/development: Firmware runs as a host-program with serial communication over a ptty device.


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
* http://www.extrudable.me/2013/04/18/exploring-extrusion-variability-and-limits/
