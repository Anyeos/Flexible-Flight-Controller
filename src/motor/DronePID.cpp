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

#include "Arduino.h"
#include "DronePID.h"
//#include "ArduPID.h"


DronePID::~DronePID() {
}

void DronePID::clear() {
    //_pError = 0;
    //_pSp = 0;
    _pPV = 0;
    _I = 0;
    _PID = 0;
}

void DronePID::setCoefficients(float kp, float ki, float kd) {
    _Kp = kp;
    _Ki = ki;
    _Kd = kd;
}

void DronePID::setOutputRange(int16_t min, int16_t max) {
    _outmin = min;
    _outmax = max;
}

void DronePID::setIRange(int16_t min, int16_t max) {
    _Imin = min;
    _Imax = max;
}

int16_t DronePID::step(int16_t Sp, int16_t PV, float dt) {
    int16_t Error = Sp - PV;
    
    fixedPID P = (Error * _Kp);
    
    _I = _I + (Error * _Ki * dt);
    if (_I > _Imax) { _I = _Imax; }
    if (_I < _Imin) { _I = _Imin; }

    //int16_t D = (int16_t)((_pError - Error - Sp + _pSp) * _Kd / dt);
    //_pError = Error;
    //_pSp = Sp;

    fixedPID D = ((_pPV - PV) * _Kd / dt);
    _pPV = PV;

    _PID = (int16_t)(P + _I + D);

    if (_PID > _outmax) { _PID = _outmax; }
    if (_PID < _outmin) { _PID = _outmin; }

    return _PID;
}