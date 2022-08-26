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

#ifndef _ESC_PWM_H
#define _ESC_PWM_H

#include "../../config.h"

#if !defined(ESC1_PWM_PIN) and !defined(ESC2_PWM_PIN) and !defined(ESC3_PWM_PIN) and !defined(ESC4_PWM_PIN)
    #ifdef __STM32F1__
        #define ESC1_PWM_PIN PA8
        #define ESC2_PWM_PIN PA9
        #define ESC3_PWM_PIN PA10
        #define ESC4_PWM_PIN PA11
    #else
        #define ESC1_PWM_PIN 3
        #define ESC2_PWM_PIN 5
        #define ESC3_PWM_PIN 6
        #define ESC4_PWM_PIN 9
    #endif
#endif


#endif