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

#ifndef INDICATORS_H
#define INDICATORS_H

struct indicator_sequence_t {
    uint8_t sequence, ms, b;
    bool on;
    bool one_shot;
    unsigned long previous_millis;
};

void indicator_setup();
void indicator_on();
void indicator_off();
void indicator_delay(uint8_t sequence, uint8_t len = 5, uint8_t ms = 100);
void indicator_do();
void indicator_done();
void indicator_error();
void indicator_start(uint8_t sequence, int ms = 100, bool force = false, bool one_shot = false);
void indicator_stop();
void indicator_loop();

#endif