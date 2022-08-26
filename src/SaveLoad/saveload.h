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

#ifndef SAVELOAD_H
#define SAVELOAD_H

void saveload_setup();
void saveload_firsttime_done();

// Size of types
// float -> 4 bytes
// int -> 2 bytes
// long -> 4 bytes

struct gyro_offset_t {
    int16_t Xgyro;
    int16_t Ygyro;
    int16_t Zgyro;
    int16_t Xaccel;
    int16_t Yaccel;
    int16_t Zaccel;
};
#define GYRO_OFFSET_ADDRESS_XGYRO 2
#define GYRO_OFFSET_ADDRESS_YGYRO 4
#define GYRO_OFFSET_ADDRESS_ZGYRO 8
#define GYRO_OFFSET_ADDRESS_XACCEL 10
#define GYRO_OFFSET_ADDRESS_YACCEL 12
#define GYRO_OFFSET_ADDRESS_ZACCEL 14
void save_gyro_offset( gyro_offset_t gyro_offset );
bool load_gyro_offset( gyro_offset_t *gyro_offset );


struct attitude_offset_t {
    int16_t pitch; // in milli radians
    int16_t roll; // in milli radians
};
#define ATTITUDE_OFFSET_ADDRESS_PITCH 16
#define ATTITUDE_OFFSET_ADDRESS_ROLL 18
void save_attitude_offset( attitude_offset_t offset );
bool load_attitude_offset( attitude_offset_t *offset );

#endif