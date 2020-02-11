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
//
//
#define AccelByteFlag (1 << 6)
#define RawByteFlag AccelByteFlag
#define DecelByteFlag (1 << 5)

//
// USB commands
//
//
// Buffered commands:
// ----------------------
//
#define CmdNull            0x0
// #define CmdDirBits         0x2
#define CmdSyncFanSpeed    0x3
#define CmdRaw             0x4
#define CmdBlock           0x6

#define CmdG1              0x7
// #define CmdDirG1           0x8


#define CmdSyncTargetTemp  0xb
#define CmdDwellMS         0xc
#define CmdG1Raw           0xd
// #define CmdDirG1Raw        0xe
#define CmdSyncHotendPWM   0xf
#define CmdSyncHotendPulse 0x10

//
// Direct commands:
// ----------------------
//

#define CmdPrinterInit          128
#define CmdMove                 129
#define CmdEOT                  130
#define CmdResetLineNr          131

#define CmdSetHomePos           133
#define CmdSetTargetTemp        134
// #define CmdWriteEepromFloat     135 // Name max. 63 chars!
// #define CmdEepromFactory        136
#define CmdFanSpeed             137
#define CmdStopMove             138

#if defined(PIDAutoTune)
    #define CmdSetHeaterY       139
#endif

#define CmdGetDirBits           150
#define CmdGetHomed             151
#define CmdGetEndstops          152
// #define CmdGetEepromVersion     153
// #define CmdGetEepromSettings    154

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
#define CmdGetTempTable         163 // ExtrusionLimit: get tempTable
#define CmdSetTempTable         164 // ExtrusionLimit: set tempTable
#define CmdEnableFRLimit        165 // Enable/disable flowrate limit

#define CmdSetContTimer         166 // Timer value for CmdContinuousE -> E-Speed
#define CmdContinuousE          167 // Start/Stop continuous e-move for filament measurement
#define CmdSetFilSensorCal      168 // Set filament sensor calibration vaue
// #define CmdSetStepsPerMME       169 // 

#define CmdSetPrinterName       170 // Write printer (-profile) name to printer eeprom
#define CmdGetPrinterName       171 // Read printer (-profile) name from printer eeprom

#define CmdSetPIDValues         172
// #define CmdSetBedlevelOffset    173
#define CmdSetIncTemp           174 // Adjust temperature niveau 
#define CmdGetFreeMem           175 // Get number of free memory bytes [uint16]
#define CmdGetFSReadings        176 // Get last n filsensor readings 
#define CmdSetTempPWM           177 // Set PWM value of hotend heater (for filament profile measurement)
#define CmdSetP0pwm             178 // Set P0 PMW value

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

//
// Unsolicited message types
//
#define ExtrusionLimitDbg       0x0
#define PidDebug                0x1
#define FilSensorDebugMsg       0x2










