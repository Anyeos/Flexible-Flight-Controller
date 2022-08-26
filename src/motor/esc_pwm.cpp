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

#ifdef ESC_PWM
#include "esc.h"
#include "motor.h"
#include "esc_pwm.h"

#include "ESC.h"
// ESC_Name (ESC PIN, Minimum Value, Maximum Value, Arm Value)
ESC ESC1 (ESC1_PWM_PIN, 1000, 2000, 500);
ESC ESC2 (ESC2_PWM_PIN, 1000, 2000, 500);
ESC ESC3 (ESC3_PWM_PIN, 1000, 2000, 500);
ESC ESC4 (ESC4_PWM_PIN, 1000, 2000, 500);

void esc_calibrate() {
    /*
    esc_disarm();
    
    ESC1.calib();
    ESC2.calib();
    ESC3.calib();
    ESC4.calib();

    esc_disarm();
    */
}

void esc_setup() {
    esc_arm();
    ESC1.speed(2000);
    ESC2.speed(2000);
    ESC3.speed(2000);
    ESC4.speed(2000);
    delay(3000);
    ESC1.speed(1000);
    ESC2.speed(1000);
    ESC3.speed(1000);
    ESC4.speed(1000);
    esc_disarm();
}

void esc_arm() {
    ESC1.arm();
    ESC2.arm();
    ESC3.arm();
    ESC4.arm();
}

void esc_disarm() {
    ESC1.stop();
    ESC2.stop();
    ESC3.stop();
    ESC4.stop();
}

// Apply real power to the motors
void esc_loop() {
    ESC1.speed(motor.motor1/10+1000);
    ESC2.speed(motor.motor2/10+1000);
    ESC3.speed(motor.motor3/10+1000);
    ESC4.speed(motor.motor4/10+1000);
}



#endif