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

#include "receiver.h"

int16_t channel[CHANNEL_COUNT+1] = { 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500 };
int16_t channel_scaled[CHANNEL_COUNT+1] = { 0, 0, 0, 0, 0, 0, 0, 0 };
int16_t channel_throttle_centered = 0;

uint8_t receiver_rssi = __UINT8_MAX__;
bool receiver_have_control = false;

int16_t scale_channel(int32_t value, int16_t side_tolerance, int16_t center_tolerance, bool center ) {
    if (center) {
        side_tolerance = side_tolerance+center_tolerance;
        value = value - 1500; // Center
        if (value < 0) {
            if (value > -center_tolerance) {
                value = -center_tolerance;
            }
            value = value+center_tolerance;
            if (value < side_tolerance-500) {
                value = side_tolerance-500;
            }

            return ( ((value)*1000 / (500-side_tolerance)) * 5);
        }
        else
        {
            if (value < center_tolerance) {
                value = center_tolerance;
            }
            value = value-center_tolerance;
            if (value > 500-side_tolerance) {
                value = 500-side_tolerance;
            }
            
            return ( ((value)*1000 / (500-side_tolerance)) * 5);
        }
    } else {
        value = value - 1000;
        if (value > 1000-side_tolerance) {
            value = 1000-side_tolerance;
        }
        if (value < side_tolerance) {
            value = side_tolerance;
        }

        return ((value-side_tolerance)*10000) / (1000-side_tolerance*2);
    }
    
    return 0;
}


void check_for_control() {
    if (
        (channel[CHANNEL_THROTTLE] > 1150) ||
        channel[CHANNEL_ROLL] > 1550 ||
        channel[CHANNEL_ROLL] < 1450 ||
        channel[CHANNEL_PITCH] > 1550 ||
        channel[CHANNEL_PITCH] < 1450 ||
        channel[CHANNEL_YAW] > 1550 ||
        channel[CHANNEL_YAW] < 1450
        )
        receiver_have_control = true;
    else
        receiver_have_control = false;
}