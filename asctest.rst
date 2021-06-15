
FDM 3d printer firmware with host preprocessing and *closed loop E*
=====================================================================


Usage
+++++++++++++

Commandline Interface, CLI
-----------------------------

*setPrinterName*
**************************************

Store printer name in printer's eeprom:

.. code-block:: sh

    ./ddprint.py setPrinterName UM2-1

.. raw:: html

    <iframe width="100%" height="475" src="https://dotnetfiddle.net/Widget/CsCons" frameborder="0"></iframe>

*calibrateESteps*
**************************************

Automatically determine extruder *e-steps* value for printer profile:

.. code-block:: sh

    ./ddprint.py calibrateESteps

.. raw:: html

    <asciinema-player src="/images/video/calesteps.asc"></asciinema-player>


*calibrateFilSensor*
**************************************

Automatically determine flowrate sensor calibration value for printer profile:

.. code-block:: sh

    ./ddprint.py calibrateFilSensor UM2-2


.. raw:: html

    <script type='text/javascript' src='/images/Widget_2.js'></script>
    <script type='text/javascript' src='/images/kofiButton.js'></script>
    <script src="/images/js_css/asciinema-player.js"></script>

