ddprint 3D printer firmware
===========================


Ultimaker direct drive firmware with host side preprocessing.

This is a firmware for Arduino Mega based (cartesian) 3D printers. I use it to print on a Ultimaker2 over USB, 
the slicers used are Cura, Slic3r and Simplify3D. Todo: link to pimped um2 here...

The original reason for this firmware was to circumvent problems with the (Ultimaker-) Marlin firmware together with the
underpowered Mega 2650 printer board. If you want to know more about these problems, just google for *marlin firmware stutter* or
*marlin usb stutter*.

In the meantime, the firmware evolved and has features like *extruder pressure advance*, *auto tempearture control*, *flowrate limiter* or *feeder slip correction*.

This allows to print fast, reliable and repeatable with high flowrates.

The software is split into two main parts:

* The host part where the cpu intensive work (gcode preprocessing, path planning, lookahead, acceleration, advance...) is done.
  The host part is written in Python.
* And the firmware part that runs on the ATMega Controller. This part executes the move commands from
  the host and does the temperature control of the printer.

A positive side effect of this architecture is the rapid development of the python based host part and
powerful debugging and testing posibilities.

The preprocessed move commands are sent through the USB connection between host and printer. The SD card is used to
buffer the received data in the firmware (like a *Swap Device*). The SD card is used in 'raw mode' without a
filesystem on it - an existing filesystem and its contents will be erased!

Some features:

*   Extrusion advance algorithm (Bernhard Kubicek, http://bernhardkubicek.soup.io/post/168776124/Another-acceleration-extrusion-compensation-for-repraps)
*   Flowrate limiter: speed is limited if the nozzle is not hot enough for the targeted flowrate to prevent underextrusion/filament grinding.
*   A "AutoTemp" algorithm, the hotend temperature is increased for parts of the model where high printing speeds are reached.
*   "Extrusion rate adjust" algorithm, to compensate for the feeder slipping on higher extrusion rates using a quadratic function. Thanks to
    illuminarti for his work on http://www.extrudable.me/2013/04/18/exploring-extrusion-variability-and-limits.
*   Support for a ADNS9800 based flowrate sensor.
*   Look ahead path planning with linear acceleration ramps.
*   Hardened USB communication using COBS encoding and CCITT checksums in both directions.
*   Hardware and software endstop support.
*   EEPROM storage of parameters.
*   PID auto-tuning. Measuring and plotting (gnuplot) of the hotend temperature stepresponse.
*   Simulator mode for testing/development: Firmware runs as a host-program with serial communication over a ptty device.
*   Preprocessor keeps track of the used extrusion rates and emits warnings if they get to high.


Similar Firmware Projects
--------------------------

Other firmware projects that split the work into two parts, a firmware part running on the printer and a host part running
on a more powerful computer (PC, Raspi etc.):

* Pacemaker: https://github.com/JustAnother1/Pacemaker
* https://github.com/mtu-most/franklin
* Klipper: https://github.com/KevinOConnor/klipper


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
* http://bernhardkubicek.soup.io/post/168776124/Another-acceleration-extrusion-compensation-for-repraps


