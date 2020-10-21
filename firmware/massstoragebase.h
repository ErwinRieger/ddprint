/*
* This file is part of ddprint - a direct drive 3D printer firmware.
* 
* Copyright 2020 erwin.rieger@ibrieger.de
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

// Buffer and sector size reading and writing to the storage device
#define SwapSectorSize 512
// Mask to implement ring-buffer
#define SwapSectorMask (SwapSectorSize - 1) 

//
// First sector of mass storage is configuration/eeprom emulation
//
union MSConfigBlock {
    struct {
        char printerName[64];
    } config;
    uint8_t sector[SwapSectorSize];
};

//
// Common base class for MassStorage classes
//
class MassStorageBase { };

