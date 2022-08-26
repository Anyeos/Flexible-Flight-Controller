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

#ifndef RECEIVER_H
#define RECEIVER_H
#include <Arduino.h>

#define CHANNEL_ROLL        1 // Aileron
#define CHANNEL_PITCH       2 // Elevator
#define CHANNEL_THROTTLE    3
#define CHANNEL_YAW         4 // Rudder
#define CHANNEL_GEAR        5
#define CHANNEL_FMODE       6
#define CHANNEL_WHEEL       7

#define CHANNEL_COUNT       7

// En realidad son 7 los canales pero el índice de un array es 0 y el primer canal es el 1 se
// requiere un espacio adicional para usar índice en 1. 
extern int16_t channel[CHANNEL_COUNT+1];

extern int16_t channel_scaled[CHANNEL_COUNT+1];
extern int16_t channel_throttle_centered;

extern uint8_t receiver_rssi; // Señal del control remoto

extern bool receiver_have_control; // Tiene el control

void receiver_setup();
void receiver_loop();
int16_t scale_channel(int32_t value, int16_t side_tolerance = 0, int16_t center_tolerance = 0, bool center = true );
void check_for_control();

#endif