
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

:Note: Work in progress, documentation is incomplete and partly outdated.


Overview
++++++++++

..
   XXX LEVEL 1 XXX

DDPrint adds process control to FDM printer.

The process that is to be controlled is the process of extruding plastic.

The extrusion system includes the feeder, a bowden tube (*) and the hotend with its heater, lets call
this system the *extruder*.

Most FDM printers have no feedback on the extruder part of the printer - they are running *open loop*.

That means to find the balance of printing speed (better: volumetric flow) and hotend temperature
is difficult. Another problem of this *open loop* extrusion system is that the printer cannot react when the
hotend has difficulties to deliver the current demand of flow (for example when printing the first layer or when
the hotend is not hot enough yet).

To close the control loop we add a `sensor <#flowratesensor>`__ to measure the performance of the extruder
und use this data to control the temperature of the hotend and the speed of the printer.

The control loop is closed in two ways, an acvitve way where the speed of the printer is lowered if the hotend cannot 
deliver the demanded flowrate and the feeder begins to slip.
The other is a *feed forward* way: The flowrate sensor is used to measure the characteristics of a given filament - extruder
combination. The result of this measurement is a so called `material profile <#material-profiles>`__.

Control of hotend temperature is called `autotemp <#auto-temp>`__.

Control of printer speed is done with the `temperature-limiter <#temperature-limiter>`__ and the `flowrate-limiter <#flowrate-limiter>`__
.






(*) For the moment `bowden style printers only. <#bowden-style-printers-only>`__

.. contents::

..
   XXX LEVEL 2 XXX

Key features
+++++++++++++

* One of the main features of ddprint: using a flowratesensor 
* Automatic material profile measurement, determine *into-air* and *printing* volumetric flowrate data.
* material profiles
* auto-temp
* temperatuer-limiter.
* flowrate-limiter.
* ddprint uses Integer math instead of floating point because of the weak atmega cpu

Installation
+++++++++++++

Firmware part
-------------

Requirements, dependencies
**************************

* `Arduino-Makefile <https://github.com/sudar/Arduino-Makefile>`__
* avrdude for arduino based firmware upload 
* stm32flash for stm32 based (jennyprinter) firmware upload 

Host part
-------------

The host software is written in Python 3.

Requirements:

* python3
* python3-serial
* python3-numpy
* npyscreen (for the TUI)

Use the `scripts/ddinstall.sh` script to install the dependencies.

Usage
+++++

The host software provides a command line interface (`ddprint`) and a text based user interface (`ddprintui`).

Use the `scripts/dd-fw-upload.sh` script to upload the firmware to the printer.

There are explanatory ansciinema screencasts for the following commands:

* `ddprint getstatus` (shortform: `ddprint stat`)
* `ddprint top`
* `ddprint mon`

Architecture
++++++++++++

The software is split into two main parts:

* The host part where the cpu intensive work (gcode preprocessing, path planning, lookahead, acceleration, advance...) is done.
  The host part is written in Python 3.
* And the firmware part that runs on the ATMega Controller in the printer. This part executes the move commands from
  the host and does other things like the temperature control of the printer.

The firmware part is implemented using the `protothreads-cpp <https://github.com/benhoyt/protothreads-cpp>`__ library.

To overcome the limited memory of the atmega, the unused SD card is converted to something like a *swap device*: It buffers the received data. This decouples the USB transfer
and actual use of the received data, too.

Experimental Features
+++++++++++++++++++++

* RepRap/OctoPrint interface: There is an experimental reprap usbserial interface using a pseudo-tty to use OctoPrint as a frontend for ddPrint. Not much functionality yet: display temperatures and some SD card commands (stubs).
* `reconnect` command: To reconnect to a running printer, works after download is complete, but not if disconnected while downloading stepper data.

Flowratesensor
++++++++++++++

.. image:: /images/ender5/flowrate_sensor_ender5_assembled.jpg
   :width: 100px
   :target: /images/ender5/flowrate_sensor_ender5_assembled.jpg

.. image:: /images/ender5/20210814_123443.jpg
   :width: 100px
   :target: /images/ender5/20210814_123443.jpg

.. image:: /images/flowratesensor/flowsensor1.jpg
   :width: 65
   :target: /images/flowratesensor/flowsensor1.jpg

.. image:: /images/ender5/flowrate_sensor_ender5.jpg
   :width: 100px
   :target: /images/ender5/flowrate_sensor_ender5.jpg

The flowrate sensor (FRS) is used to measure the movement of the filament. It consists of a incremental rotary encoder (Bourns EMS22) and a 3d printed housing.
The axle of the encoder is pressed onto the moving filament using a spring that is part of the FRS housing.

The distance-information from the FRS is continuously read by the firmware and is used for several tasks:

* Compute the speed of the filament and the resulting volumetric flowrate (taking filament diameter into account).
* Compare the actual volumetric flow with the nominal volumetric flow to implement the `flowrate-limiter <#flowrate-limiter>`__.
* Automatically record `material profiles <#material-profiles>`__.
* Automatically `calibrate the feeder "esteps" <#calibrateesteps>`__ (for the machine profile).
* Automatically `calibrate the FRS <#calibratefilsensor>`__ (for the machine profile, too).

More details `are here <#flowratesensor-1>`__.

Bowden style printers only?
+++++++++++++++++++++++++++

For best results, the flowrate sensor has to be placed *after* the feeder. This is easy for a bowden style printer.

For direct driver printers it should be possible but it is more difficult to add the flowrate sensor into the print head (because of
space requirements, heat, added weight and so on).

Material Profiles
+++++++++++++++++

Material (filament) profiles are used for two things in ddprint:

* They define the hotend temperature necessary to melt a given volumetric flow of filament, see `autotemp feature <#auto-temp>`__.
* The `temperature-limiter <#temperature-limiter>`__ uses the information in the material profile to slow down the print in cases
  where the hotend is not hot enough (yet) to melt the requestet amount of filament.

With other words: the material profile gives a picture of the hotend melting capacity for a given machine/filament combination - "*the printer knows its filament*".

A material profile for a given filament is created automatically by ddPrint and stored in JSON format for later use.

This filament-measurement is done in two steps:

* A best-case scenario where filament is extruded *into air*.
* And a worst-case scenario where a small testpart is printed under difficult circumstances (high backpressure because of 100% infill and small layerheight).

ddPrint comes with a python script to plot material profiles (`plot_mat_profile utility <#profile-plotting-utility>`__). Here are two examples of material profiles, one
for a PLA filament and one for a PETg filament:

.. image:: /images/mat-profile/Mat._Profile_esun_pla_glass-purple.json.png
   :width: 250px
   :target: /images/mat-profile/Mat._Profile_esun_pla_glass-purple.json.png

.. image:: /images/mat-profile/Mat._Profile_herz_petg-black-5800070.json.png
   :width: 250px
   :target: /images/mat-profile/Mat._Profile_herz_petg-black-5800070.json.png


For more details `see here <#material-profiles-1>`__.

Auto Temp
+++++++++++++

While parsing/pathplanning the gcode input, the needed volumetric flowrate is computed. Then the required (minimum) temperature
for this flowrate is determined using a (automatically measured) `material profile <#material-profiles>`__ of the used filament.

So when printing, the temperature of the hotend is dynamically changed in respect to the currently requested flowrate.
This is done in a feed-forward manner because there is a delay between controlling the hotend heater and the change of
temperature in the melting zone/nozzle, of course.

:Note: because of this automatic temperature control, ddPrint ignores bed- and hotend-temperature related commands (M104, M140...) in the gcode input file. When slicing
       your models to be printed with ddPrint you can forget about all the temperature settings there.

The hotend temperature follows the volumentric flow demand given in the input gcode file, is increased for parts of the model where high
flow rates are required and vice-versa.

Temperature limiter
++++++++++++++++++++

The firmware part of ddPrint running on the printer maintains a *temperature-flowrate* table. This table is downloaded from the host to the firmware before
a print is done.

The *temperature-flowrate table* maps hotend temperatures to the max. volumetric flowrates (extruder speed) allowed at a given temperature (for the used
filament). This table is generated from the information found in the material-profile of the used filament (see `material profile <#material-profiles>`__) by the host
part of ddPrint.

If the current hotend temperature is too low for the requested extrusion speed, the speed of the printer (feedrate) is scaled down so that it matches
the achievable flowrate value in the table. This avoids underextrusion (thogether with filament-grinding) and the many problemns related to it.

This is called *temperature limiting*. The temperature-limiter works on a per-printing-move basis, that means this check and a possible slowdown is done
for every straight line of the printhead (essentially for every printing gcode line).

With other words: The *temperature-limiter* together with the `autotemp <#auto-temp>`__ feature ensures that the hotend is always hot enough to
be able to melt the requeste amount of filament.

Flowrate limiter
++++++++++++++++

Similar to the `temperature-limiter <#temperature-limiter>`__ the *flowrate-limiter* slows down the print to avoid underextrusion and grinding of the filament.

It starts to limit the feedrate when the feeder slip is above some threshold, so it uses the *grip value* measured by the `volumetric flowrate sensor <#flowratesensor>`__ (FRS).

Up to 10% of feeder slippage (90% grip) is allowed before the firmware begins to slow down the print. Below 90% grip the feedrate is decreased linearly until reaches
one quater (25%) of the nominal speed, see following plot.

.. image:: /images/ddprint/flowrate-limiter-plot.png
   :width: 150px
   :target: /images/ddprint/flowrate-limiter-plot.png


..
   XXX LEVEL 3 XXX

More in detail
++++++++++++++

Flowratesensor
--------------

.. image:: /images/ender5/flowrate_sensor_ender5_assembled.jpg
   :width: 100px
   :target: /images/ender5/flowrate_sensor_ender5_assembled.jpg

.. image:: /images/ender5/20210814_123443.jpg
   :width: 100px
   :target: /images/ender5/20210814_123443.jpg

.. image:: /stl/feedsensor_v2_preview_cutopen.png
   :width: 185px
   :target: /stl/feedsensor_v2_preview_cutopen.png

The FRS consists of the following components:

* A 3d printed housing (PETg).
* The incremental rotary encoder (EMS22).
* A ptfe inliner with a small cutout to allow the encoder axle to touch the filament.
* A pneumatic coupler to connect the extruder bowden tube to the FRS.
* Depending on the type of the feeder a short piece of a M6 heatbreak to mount the FRS
  at the feeder outlet.
* A cable to connect the FRS to the mainboard of the printer (SPI bus).

The EMS22 rotary encoder has a resolution of 1024 counts per revolution. The diameter of the axle is 3.17mm, this equates
to a overall resolution of about 10µm (0.0097mm) of filament movement per count.

The nominal accuracy of the sensor is 0.7° (about 0.2%), worst case accuracy is 1.4° (about 0.4%). You can find a copy of the EMS22
datasheet (PDF) `here </doc/datasheets/EMS22A.pdf>`__.

The firmware reads the rotary encoder every 10mS, meaning a sample rate of 100Hz.

.. image:: images/flowratesensor/um2-feeder-adapter.jpg
   :width: 65px
   :target: images/flowratesensor/um2-feeder-adapter.jpg

.. image:: /images/ender5/flowrate_sensor_ender5_with_feeder.jpg
   :width: 100px
   :target: /images/ender5/flowrate_sensor_ender5_with_feeder.jpg

The FRS is mounted at the feeder outlet with an adapter that is part of the FRS housing (BMG or UM2 feeder) or with
a short piece of a M6 heatbreak (Anycubic or Ender feeder) (todo: add bmg style picture).

`Here are some STL files <http://github.com/ErwinRieger/ddprint/tree/master/stl>`__ of the FRS housing.

Material Profiles
-----------------

.. image:: /images/mat-profile/Mat._Profile_esun_pla_glass-purple.json-cut.png
   :width: 350px
   :target: /images/mat-profile/Mat._Profile_esun_pla_glass-purple.json-cut.png

A material profile defines the "melting-capabilites" of a given machine/nozzle/filament combination. It is used by the ddPrint
software to implement the `autotemp <#auto-temp>`__ and the `temperature-limiter <#temperature-limiter>`__ features. The material profile
determines the volumetric flowrate at a given temperature and vice-versa.

Material profiles are generated automatically by ddPrint using the `flowrate sensor <#flowratesensor>`__. This measurement is done
in two steps. This gives us a range of achievable volumetric flowrates for a given filament ("working field", the grey area in the profile plot).

In a first measurement the maximum volumetric flowrate for a given temperature is determined (blue line in material profile plot). This is done by extruding some plastic `into-air <#into-air-filament-measurement>`__, 
without any flow restriction of a printed part. This gives us a best-case flowrate.

The the other end of the flowrate range is measured doing a `real print <#real-print-filament-measurement>`__. In this case the flow through the nozzle is restricted by the part
thats printed. The testpart is sliced using 100% infill and a very low layerheight. This gives us the worst-case flowrate (green line in material profile plot).

:Note: To display a material profile graphically, use the `plot_mat_profile utility <#profile-plotting-utility>`__.

Into-air filament measurement
*****************************

.. image:: /images/mat-profile/Mat._Profile_esun_pla_glass-purple.json-cut.png
   :width: 350px
   :target: /images/mat-profile/Mat._Profile_esun_pla_glass-purple.json-cut.png

Into-air flowrate (blue line in plot above) is determined using the `measureTempFlowrateCurve <#measure-material-profile-measuretempflowratecurve>`__ command
of the ddPrint program.

To save filament, only two temperature/flowrate points are measured (P1 and P2). One at the lower end of the usable temperature range and one at the top
end (for example, at 200° and 260°). The temperature-flowrate graph is then aproximated using this two points, they define a straight line equation y = M*x + C1.

The steps to measure *into-air flowrate data* are are:

* User loads the filament to test and starts measurement.
* ddPrint heats the hotend to the lower temperature.
* When start-temperature is reached, the PWM value of the hotend heater is fixed (PID temp control is disabled) and
  ddPrint starts to extrude filament at a low flowrate.
* Then in a loop, the flowrate is increased while watching the feeder grip.
* While increasing the flowrate, the feeder grip will decrease because of the rising
  backpressure from the filament pressed through the nozzle.
* At some point, feeder grip drops below a pre-defined value called *minGrip*, 90% for example.
* Flowrate is no longer increased, the first measurement point P1 is determined.
* PID temperature control is re-enabled and hotend is heated to the upper temp.
* When this upper temperature is reached, the measurement loop starts again. PWM is fixed and
  flowrate is increased while watching feeder grip.
* When feeder grip falls below *minGrip* the second time, we have determined the second
  measurement point P2.
* Hotend is switched off and the speed of the extruder is gracefully decreased until stop.
* The final step is to compute the values of M and C1 and write the *into-air flowrate data* to a material profile template file.


Real print filament measurement
********************************

.. image:: /images/mat-profile/Mat._Profile_esun_pla_glass-purple.json-cut.png
   :width: 350px
   :target: /images/mat-profile/Mat._Profile_esun_pla_glass-purple.json-cut.png

This is similar to the *into-air* measurement, but this time doing a real print. The result is the *worst case flowrate data* (the green line in the plot above).

The ddPrint command for this measurement is `measureTempFlowrateCurve2 <#measure-material-profile-measuretempflowratecurve2>`__.

Again a straight line equation y = M*x + C2 is determined. To save filament, only one temperature/flowrate point is measured (P3). As a simplification,
we assume the same slope as in the *into-air* measurement.

.. image:: /images/mat-profile/measure2.png
   :width: 100px
   :target: /images/mat-profile/measure2.png

The test-part printed is designed and sliced as follows:

* The part is tall to quickly gain height, this is to minimize the heating influence of the heated bed.
* It is long in one direction to help the flowrate measurement (long straight lines with constant extrusion, minimize acceleration effects).
* It is sliced with 100% infill for high nozzle backpressure.
* It is sliced with a small layer-height for high nozzle backpressure.
* It is sliced with a high feedrate, again for high nozzle backpressure.

:Note: The test-part is not printed to its full height, measurement will stop before.

The steps to measure *printing flowrate data* are are:

* Filament to test is loaded, printer is prepared to do a print and user starts measurement.
* ddPrint heats the hotend to a temperature in the middle of the usable temperature range.
* When start-temperature is reached, the PWM value of the hotend heater is fixed (PID temp control is disabled) and
  ddPrint starts to print the testpart using a low speed/flowrate.
* Print until some height is reached to minimize heating bed effects.
* Then in a loop, the flowrate is increased on every layer while watching the feeder grip.
* While increasing the flowrate, the feeder grip will decrease because of the rising
  backpressure from the filament pressed through the nozzle (additionally restricted by the part thats printed).
* At some point, feeder grip drops below a pre-defined value called *minGrip*, 90% for example.
* Measurement ends, datapoint P3 is determined.
* Print is gracefully stopped.
* The final step is to write the *printing flowrate data* (together with the *into-air data*) to a material profile template file.

After editing the newly measured material-profile (.json file) it can be renamed and stored somewhere in a profiles folder for
later view (`plot_mat_profile utility <#profile-plotting-utility>`__) or use (printing).

Workingpoint (strength) parameter
**********************************

The `autotemp <#auto-temp>`__ feature splits the print into segements and determines the necessary temperature for theese
segments using the given material profile. It then adds the corresponding temperature control commands into the datastream
sent to the printer. This is done in a feed-forward fashion, the temperature is set some time before the segment is
printed.

When looking up the needed temperature in the material profile, the following question arises: Which
temperature should be used, the one belonging to the into-air measurement, the one from the real-print measurement or
a temperature somewhere in between?

The next picture illustrates this. In this example, we want to know the necessary temperature for a volumetric
flowrate of 9 mm³/s (pink horizontal line).

.. image:: /images/mat-profile/Mat._Profile_esun_pla_glass-purple.json-wp1.png
   :width: 350
   :target: /images/mat-profile/Mat._Profile_esun_pla_glass-purple.json-wp1.png

As we can see, the into-air temperature for this flowrate would be 202 °C (T1) and the real-print temp is 231 °C (T0).

This is where the *workingpoint* parameter comes into play. The workingpoint (WP) parameter tells ddPrint which temperature
to choose. WP is a value in the range 0...1 an can be specified as a commandline parameter or through a experimental
G-Code command (M901). When WP is 0 then the temperature belonging to the real-print graph is used and when WP is 1 then
the into-air temperature is used. Other values of WP use the according temperature between T0 and T1.

The default value of WP is 0.5, the corresponding graph looks like this:

.. image:: /images/mat-profile/Mat._Profile_esun_pla_glass-purple.json-wp05-cut.png
   :width: 350
   :target: /images/mat-profile/Mat._Profile_esun_pla_glass-purple.json-wp05-cut.png

With WP=0.5, the autotemp feature of ddPrint uses temperatures in the middle of the two graphs, the corresponding function
is shown as the brown linegraph. Hotend temperature for flowrate 9 mm³/s in this case is 216 °C.

With other words, with the workingpoint parameter we can:

* Control the temperature level the autotemp feature is using througout the print.
* Control the quality of the print, *look VS strength tradeof*.

WP values towards zero are increasing the temperature level, resulting in better layerbonding and stronger
parts (good for functional parts). WP values towards one are lowering the resulting temperatures, resulting in
a better look of the printed parts (better for figurines and the like).

Two examples with a volumetric flowrate of 9 mm³/s and working points 0.3 (hotter than default WP 0.5) and
0.8 (cooler than default WP 0.5):

.. image:: /images/mat-profile/Mat._Profile_esun_pla_glass-purple.json-wp03-cut.png
   :width: 350
   :target: /images/mat-profile/Mat._Profile_esun_pla_glass-purple.json-wp03-cut.png

.. image:: /images/mat-profile/Mat._Profile_esun_pla_glass-purple.json-wp08-cut.png
   :width: 350
   :target: /images/mat-profile/Mat._Profile_esun_pla_glass-purple.json-wp08-cut.png

When workingpoint value is set to zero, then the real-print measurement graph is used (brown linegraph laying on the
green graph). Accordingly when WP is one, the into-air measurement grap is used (brown linegraph laying on the blue graph).

TBD: note about PWM values/graph, link to think in terms of energy.

Think in terms of energy
********************************

TBD: think more in terms of energy (hotend PWM) instead of temperature.

Profile plotting utility
********************************

TBD: describe *plot_mat_profile* utility.

.. code-block:: sh

      usage: plot_mat_profile.py [-h] printer nozzle mat smat [smat ...]

      plot_mat_profile.py - plot ddPrint material profiles.

      positional arguments:
        printer     Name of printer to select profiles.
        nozzle      Name of nozzle profile to use [nozzle40, nozzle80...].
        mat         Name of generic material profile to use [pla, petg...].
        smat        Material profile(s) to plot.

      optional arguments:
        -h, --help  show this help message and exit

---------------------------------------------------------------------------------------------


.. raw:: html

    <script type='text/javascript' src='/images/Widget_2.js'></script>
    <script type='text/javascript' src='/images/kofiButton.js'></script>
    <script src="/images/js_css/asciinema-player.js"></script>

