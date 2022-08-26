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

#ifdef ESC_ANALOG
#include "esc.h"
#include "motor.h"
#include "esc_analog.h"


void esc_calibrate() {
    esc_disarm();
}

void esc_setup() {
    pinMode(ESC1_ANALOG_PIN, OUTPUT);
    pinMode(ESC2_ANALOG_PIN, OUTPUT);
    pinMode(ESC3_ANALOG_PIN, OUTPUT);
    pinMode(ESC4_ANALOG_PIN, OUTPUT);
}

void esc_arm() {
}

void esc_disarm() {
    digitalWrite(ESC1_ANALOG_PIN, 0);
    digitalWrite(ESC2_ANALOG_PIN, 0);
    digitalWrite(ESC3_ANALOG_PIN, 0);
    digitalWrite(ESC4_ANALOG_PIN, 0);
}

// Apply real power to the motors
void esc_loop() {
    uint8_t m;
    m = max(motor.motor1/39-1, 255);
    analogWrite(ESC1_ANALOG_PIN, m);

    m = max(motor.motor2/39-1, 255);
    analogWrite(ESC2_ANALOG_PIN, m);

    m = max(motor.motor3/39-1, 255);
    analogWrite(ESC3_ANALOG_PIN, m);

    m = max(motor.motor4/39-1, 255);
    analogWrite(ESC4_ANALOG_PIN, m);
}



#endif