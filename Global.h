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

#ifndef GLOBAL_H
#define GLOBAL_H

#define VERSION "0.5b"

#include <Wire.h>
#include "config.h"
#include "src/Debugino/Debug.h"
#include "src/indicators/indicators.h"
#include "src/third_party/FixedPoints/FixedPoints.h"
#include "src/third_party/FixedPoints/FixedPointsCommon.h"
#define fixed SFixed<23U, 8U>

//TwoWire HWire (2, I2C_FAST_MODE);          //Initiate I2C port 2 at 400kHz.
//TwoWire HWire (PIN_I2C_SDA, PIN_I2C_SCL);
//extern TwoWire HWire;

struct waypoint_t {
  // Barometer based
  int32_t altitude; // millimeters
  // GPS location
  fixed altitude_amsl; // meters
  fixed longitude;
  fixed latitude;
};


#define STATE_INIT          0
#define STATE_LANDED        1
#define STATE_CALIBRATING   3
#define STATE_ARMED         4
#define STATE_RETURN_HOME   5
#define STATE_FLYING        6
#define STATE_LANDING       7
#define STATE_SETUP         255

extern uint8_t state;
extern bool first_time;
extern uint8_t flight_mode;


#endif