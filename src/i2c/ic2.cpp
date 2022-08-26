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

/*
 I²C helper functions.
 It is to avoid the duplication of code where there are more than one sensor using the
 same bahaviour to request or send data on the bus.
*/

#include "i2c.h"

uint8_t i2cWriteRegister(TwoWire &TWire, uint8_t address, uint8_t reg, uint8_t val )
{
    TWire.beginTransmission(address);
    TWire.write(reg);
    TWire.write(val);
    return TWire.endTransmission();
}

uint8_t i2cReadRegister(TwoWire &TWire, uint8_t address, uint8_t reg )
{
    TWire.beginTransmission(address);
    TWire.write(reg);
    TWire.endTransmission(false);
    TWire.requestFrom(address, (uint8_t)1);
    if (TWire.available() > 0) {
        return TWire.read();
    }
    return 0;
}

int16_t i2cReadRegister16(TwoWire &TWire, uint8_t address, uint8_t reg )
{
    TWire.beginTransmission(address);
    TWire.write(reg);
    TWire.endTransmission(false);
    TWire.requestFrom(address, (uint8_t)2);
    if(TWire.available() > 1){
        return (TWire.read()<<8) + TWire.read();
    }
    return 0;
}

uint8_t i2cRequestFrom(TwoWire &TWire, uint8_t address, uint8_t reg, uint8_t count )
{
	TWire.beginTransmission(address);
	TWire.write(reg);
	uint8_t result = TWire.endTransmission(false);
    if (result == 0) { TWire.requestFrom(address, count); }
    return result;
}