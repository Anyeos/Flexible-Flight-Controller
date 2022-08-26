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

#ifndef _ATTITUDE_H_
#define _ATTITUDE_H_

#include "../../Global.h"

#define SAMPLED_IMU 0x01
#define SAMPLED_MAG 0x02
#define SAMPLED_YAW_EXACT 0x04 // If yaw is being computed exactly or by approximation
#define SAMPLED_ATTITUDE 0x08



struct attitude_t {
  fixed roll; /*< [rad] Roll angle (-pi..+pi)*/
  fixed pitch; /*< [rad] Pitch angle (-pi..+pi)*/
  fixed yaw; /*< [rad] Yaw angle (-pi..+pi)*/
  fixed rollspeed; /*< [rad/s] Roll angular speed*/
  fixed pitchspeed; /*< [rad/s] Pitch angular speed*/
  fixed yawspeed; /*< [rad/s] Yaw angular speed*/

  // for AHRS
  fixed ax, ay, az; // Accel computed data in m/seg
  fixed gx, gy, gz; // Gyro computed data in rad/seg

  fixed mx, my, mz; // Magnetometer computed data

  // auxiliares
  uint8_t sampled; // What was sampled? IMU, Magneto or Both

  float q0, q1, q2, q3;
  float integralFBx, integralFBy, integralFBz;  // integral error terms scaled by Ki
  unsigned long lastUpdate;
  float deltat;

  //fixed last_roll;
  //fixed last_pitch;
  //fixed last_yaw;
};
extern attitude_t attitude;


void attitude_updateIMU( float ax, float ay, float az, float gx, float gy, float gz );
void attitude_update( float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz );
float invSqrt(float x);
void attitude_compute_angles();

void attitude_setup();
void attitude_loop();
void attitude_calibrate();


#endif