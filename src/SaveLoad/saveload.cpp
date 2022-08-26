/******************************************************************************
This file is part of Flexible Flight Controller, a flight controller to everyone 
with a target on flexibility.
*******************************************************************************
LICENCE:

Copyright 2022 Schwartz Germán Andrés <anyeos@gmail.com>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal 
in the Software without restriction, including without limitation the rights 
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell 
copies of the Software, and to permit persons to whom the Software is 
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE 
SOFTWARE.
******************************************************************************/

#include "../../Global.h"
#include <Arduino.h>
#include <EEPROM.h>
#include <EEPROMTyped.h>
#include "saveload.h"
#include "../indicators/indicators.h"

void saveload_setup() {
    indicator_do();
    #ifdef __STM32F1__
        EEPROM.PageBase0 = 0x801F000;
        EEPROM.PageBase1 = 0x801F800;
        EEPROM.PageSize  = 0x400;
        uint16_t E_status = EEPROM.init();
        INFOPRINT("EEPROM Init status = ");
        INFOPRINT(E_status);
        INFOPRINTLN();
    #endif
    #if defined(ARDUINO_ARCH_RP2040)
        EEPROM.begin(256);
    #endif

    uint8_t first_time_value;
    EEPROMTyped.read(0, first_time_value);
    /*if (first_time_value != FIRST_TIME_VALUE) {
        first_time = true;
    }*/

    indicator_done();
}

void saveload_firsttime_done() {
    if (first_time) {
        first_time = false;
        EEPROMTyped.write(0, FIRST_TIME_VALUE);
    }
}


void save_gyro_offset( gyro_offset_t gyro_offset ) {
    EEPROMTyped.write(GYRO_OFFSET_ADDRESS_XGYRO,
        gyro_offset.Xgyro);
    EEPROMTyped.write(GYRO_OFFSET_ADDRESS_YGYRO,
        gyro_offset.Ygyro);
    EEPROMTyped.write(GYRO_OFFSET_ADDRESS_ZGYRO,
        gyro_offset.Zgyro);

    EEPROMTyped.write(GYRO_OFFSET_ADDRESS_XACCEL,
        gyro_offset.Xaccel);
    EEPROMTyped.write(GYRO_OFFSET_ADDRESS_YACCEL,
        gyro_offset.Yaccel);
    EEPROMTyped.write(GYRO_OFFSET_ADDRESS_ZACCEL,
        gyro_offset.Zaccel);
}
bool load_gyro_offset( gyro_offset_t *gyro_offset ) {
    EEPROMTyped.read(GYRO_OFFSET_ADDRESS_XGYRO, 
        gyro_offset->Xgyro);
    EEPROMTyped.read(GYRO_OFFSET_ADDRESS_YGYRO, 
        gyro_offset->Ygyro);
    EEPROMTyped.read(GYRO_OFFSET_ADDRESS_ZGYRO, 
        gyro_offset->Zgyro);

    EEPROMTyped.read(GYRO_OFFSET_ADDRESS_XACCEL, 
        gyro_offset->Xaccel);
    EEPROMTyped.read(GYRO_OFFSET_ADDRESS_YACCEL, 
        gyro_offset->Yaccel);
    EEPROMTyped.read(GYRO_OFFSET_ADDRESS_ZACCEL, 
        gyro_offset->Zaccel);
    return true;
}

void save_attitude_offset( attitude_offset_t offset ) {
    EEPROMTyped.write(ATTITUDE_OFFSET_ADDRESS_PITCH,
        offset.pitch);
    EEPROMTyped.write(ATTITUDE_OFFSET_ADDRESS_ROLL,
        offset.roll);
}

bool load_attitude_offset( attitude_offset_t *offset ) {
    EEPROMTyped.read(ATTITUDE_OFFSET_ADDRESS_PITCH, 
        offset->pitch);
    EEPROMTyped.read(ATTITUDE_OFFSET_ADDRESS_ROLL, 
        offset->roll);
    return true;
}