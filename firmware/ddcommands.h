/*
* This file is part of ddprint - a direct drive 3D printer firmware.
* 
* Copyright 2015 erwin.rieger@ibrieger.de
* 
* ddprint is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
* 
* ddprint is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
* 
* You should have received a copy of the GNU General Public License
* along with ddprint.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

//
// USB/Seraial communication
//
// Startbyte of datablock for serial communication
// #define SOH  0x81
#define SOH  0x0

// 
// Move flags
//
#define AccelByteFlag (1 << 6)
#define RawByteFlag AccelByteFlag
#define DecelByteFlag (1 << 5)

// MeasureStartBit: move is suatable for FRS measurement
// #define MeasureStartBit (1 << 9)    // 0x200
// #define MeasureStartBitRaw (1 << 8) // 0x100

//
// USB commands
//
//
// Buffered commands:
// ----------------------
//
// #define CmdNull            0x0
// #define CmdDirBits         0x2
#define CmdSyncFanSpeed    0x3
#define CmdRaw             0x4
// #define CmdBlock           0x6

#define CmdG1              0x7
// #define CmdDirG1           0x8


#define CmdSyncTargetTemp  0xb
#define CmdDwellMS         0xc
#define CmdG1Raw           0xd
// #define CmdDirG1Raw        0xe
#define CmdSuggestPwm      0x11

#define CmdNop             0x77

//
// Direct commands:
// ----------------------
//

#define CmdPrinterInit          128
#define CmdMove                 129
#define CmdResetLineNr          131

#define CmdSetPos               133
#define CmdSetTargetTemp        134
#define CmdGetCardSize          135 // Get size of mass storage, number of 512b blocks
#define CmdErase                136 // Erase mass storage, parameter: number of 512b blocks
#define CmdFanSpeed             137
#define CmdStopMove             138

// #define CmdGetDirBits           150
#define CmdGetHomed             151
#define CmdGetEndstops          152

#define CmdDisableStepperIsr    155
#define CmdDisableSteppers      156

#define CmdGetCurrentTemps      157
#define CmdGetTargetTemps       158
#define CmdGetPos               159

#if defined(DDSim)
    #define CmdExit             160
#endif
#define CmdGetStatus            161

// Get raw value of filament sensor pos 
#define CmdGetFilSensor         162 
// #define CmdGetTempTable         163 // ExtrusionLimit: get tempTable
#define CmdSetTempTable         164 // ExtrusionLimit: set tempTable
#define CmdEnableFRLimit        165 // Enable/disable flowrate limit

#define CmdSetContTimer         166 // Timer value for CmdContinuousE move (set speed), set to 0 for stop.
#define CmdContinuous           167 // Start/Stop continuous stepper move for debugging/measurement
#define CmdSetFilSensorConfig   168 // Flowrate sensor: Set steps per mm value and calibration value.
#define CmdSetFilSensorCal      169 // Flowrate sensor: set calibration value.

#define CmdSetPrinterName       170 // Write printer (-profile) name to printer eeprom
#define CmdGetPrinterName       171 // Read printer (-profile) name from printer eeprom

#define CmdSetPIDValues         172
// #define CmdSetBedlevelOffset    173
#define CmdSetIncTemp           174 // Adjust temperature niveau 
#define CmdGetFreeMem           175 // Get number of free memory bytes [uint16]
#define CmdGetFSReadings        176 // Get last n filsensor readings 
#define CmdSetTempPWM           177 // Set PWM value of hotend heater (for filament profile measurement)
#define CmdSoftStop             180 // Stop printer softly after current path is finished.  
#define CmdBootBootloader       181 // Reboot into bootloader (stm32)
#define CmdReadGpio             182 // Read a gpio port, used to determine pinmap.
#define CmdSetGpio              183 // Set a gpio port, dangerous, used to determine pinmap.
#define CmdReadAnalogGpio       184 // Read a analog value from gpio port, used to determine pinmap.
#define CmdSetStepsPerMM        185 // Initial printer settings: steps per mm for X/Y/Z
#define CmdSetHostSettings      186 // Initial printer settings
#define CmdSystemReset          187 // Emergency hard reset system
#define CmdGetTaskStatus        188 //
#define CmdGetIOStats           189 //
#define CmdDumpMassStorage      190 // Dump a 512 bytes sector from sdcard/usb.
#define CmdSetBaudRate          191 // Autobaudrate
#define CmdSetSlowDown          192 // Slowdown print, for filament measurement
#define CmdGetVersion           193 // Get git version of build.

//
// Response types 
//
#define RespUnknownCommand      1 // Unknown Direct Command, Payload: the unknown command (1 byte)
// #define RespGenericString    2 // Payload: 'pascal string'
#define RespKilled              3 // Payload: reason, optional parameters
#define RespRXError             4 // Payload: serialNumber (last line), errorflags
#define RespRXCRCError          5 // Payload: serialNumber (last line)
#define RespACK                 6 // 
#define RespSerNumberError      7 // Payload: serialNumber (last line)
#define RespRXTimeoutError      8 // Payload: serialNumber (last line)
#define RespUnsolicitedMsg      9 // Payload: message type, params
// #define RespRXFullError        10 // Payload: serialNumber (last line)

//
// Rsponse error codes (payload)
//
#define RespOK                  1 // Command successful
#define RespInvalidArgument     2 // Payload: none
#define RespHardwareEndstop     3 // Payload: none
#define RespSoftwareEndstop     4 // Payload: none
#define RespUnknownBCommand     5 // Unknown Buffered Command, payload: the command
#define RespAssertion           6 // payload: line, file
#define RespFilsensorInit       7 // 
#define RespSDInit              8 // 
#define RespSDReadError         9 // 
#define RespMinTemp            10 // 
#define RespMaxTemp            11 // 
#define RespSDWriteError       12 // 
#define RespUnderrun           13 // 

//
// Unsolicited message types
//
// #define ExtrusionLimitDbg       0x0
#define PidDebug                0x1
// #define FilSensorDebugMsg       0x2
#define GenericMessage          0x3
#define BufDebug                0x4
#define PidSwitch               0x5
#define GenericInt32            0x6










