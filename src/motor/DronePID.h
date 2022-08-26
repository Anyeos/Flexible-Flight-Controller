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

#ifndef PID_H
#define PID_H

#include <stdint.h>
#include "../../Global.h"
#define fixedPID SFixed<21U, 10U>
//#define fixedPID float

class DronePID {

public:
  DronePID() 
  {
    clear();
  }

  DronePID(float kp, float ki, float kd)
  {
    clear();
    setCoefficients(kp, ki, kd);
    setOutputRange(-750, 750);
    setIRange(-250, 250);
  }

  DronePID(float kp, float ki, float kd, int16_t outmin, int16_t outmax)
  {
    clear();
    setCoefficients(kp, ki, kd);
    setOutputRange(outmin, outmax);
    setIRange(-250, 250);
  }

  ~DronePID();

  void setCoefficients(float kp, float ki, float kd);
  void setOutputRange(int16_t min, int16_t max);
  void setIRange(int16_t min, int16_t max);
  void clear();
  int16_t step(int16_t Sp, int16_t PV, float dt);

private:

  // Configuration
  fixedPID _Kp, _Ki, _Kd;
  int16_t _outmax, _outmin; 
  int16_t _Imax, _Imin;
  fixedPID _I;
  //int16_t _pError;
  //int16_t _pSp;
  int16_t _pPV;
  int16_t _PID;
};

#endif
