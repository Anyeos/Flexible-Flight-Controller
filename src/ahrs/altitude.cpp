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

#include "altitude.h"

void altitude_setup() {
    altitude.altitude = 0;
    altitude.velocity = 0;
};

#if !defined(BAROMETER_NO) && !defined(GYRO_NO)
#include "../../Global.h"
#include "../third_party/AltitudeEstimation/altitude.h"
AltitudeEstimator altitude_estimator = AltitudeEstimator(
    0.0005, // sigma Accel
    0.0005, // sigma Gyro
    0.018,   // sigma Baro
    0.5, // ca
    0.1);// accelThreshold

void altitude_loop() {
    float gyroData[3];
    float accelData[3];
    gyroData[0] = (float)attitude.rollspeed;
    gyroData[1] = (float)attitude.pitchspeed;
    gyroData[2] = (float)attitude.yawspeed;
    accelData[0] = (float)attitude.ax;
    accelData[1] = (float)attitude.ay;
    accelData[2] = (float)attitude.az;
    altitude_estimator.estimate(accelData, gyroData, altitude.barometer / 1000.0, micros());

    altitude.altitude = altitude_estimator.getAltitude();
    altitude.velocity = altitude_estimator.getVerticalVelocity();
    DEBUGPRINT("Altitude: ");
    DEBUGPRINT((float)altitude.altitude); DEBUGPRINT("\t"); 
    DEBUGPRINT((float)altitude.velocity); DEBUGPRINT("\t"); 
    DEBUGPRINTLN("");
};
#else
void altitude_loop() {
    altitude.altitude = altitude.barometer;
};
#endif