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


#include "attitude.h"
#include "../SaveLoad/saveload.h"

attitude_offset_t attitudeOffset;
void attitude_calibrate() {
	indicator_do();
	DEBUGPRINTLN("Calibrating attitude...");

    attitudeOffset.roll = 0;
    attitudeOffset.pitch = 0;
    attitude_compute_angles();
    attitudeOffset.roll = (float)attitude.roll * 1000;
    attitudeOffset.pitch = (float)attitude.pitch * 1000;
    attitude_compute_angles();
    indicator_done();
}


void attitude_setup() {
    attitude.roll = 0;
    attitude.pitch = 0;
    attitude.yaw = 0;
    attitude.rollspeed = 0;
    attitude.pitchspeed = 0;
    attitude.yawspeed = 0;

    attitude.ax = 0;
    attitude.ay = 0;
    attitude.az = 0;

    attitude.gx = 0;
    attitude.gy = 0;
    attitude.gz = 0;

    attitude.mx = 0;
    attitude.my = 0;
    attitude.mz = 0;

    attitude.sampled = 0;

	attitude.q0 = 1.0;
	attitude.q1 = 0.0;
	attitude.q2 = 0.0;
	attitude.q3 = 0.0;
	attitude.integralFBx = 0.0;
	attitude.integralFBy = 0.0;
	attitude.integralFBz = 0.0;
	attitude.deltat = 0.0;
}

//#include "../third_party/SensorFusion/SensorFusion.h"
//SF fusion;
void attitude_loop() {
    unsigned long us = micros();
    #ifdef PID_US
    if (us - attitude.lastUpdate < PID_US) { return; }
    #else
    if ((attitude.sampled & SAMPLED_IMU) != SAMPLED_IMU) { return; }
    #endif

    attitude.deltat = ((us - attitude.lastUpdate) / 1000000.0);
    attitude.lastUpdate = us;

    if((attitude.mx == 0.0) && (attitude.my == 0.0) && (attitude.mz == 0.0)) {
        attitude_updateIMU(
            (float)attitude.ax, (float)attitude.ay, (float)attitude.az,
            (float)attitude.gx, (float)attitude.gy, (float)attitude.gz);
    } else {
        attitude_update(
            (float)attitude.ax, (float)attitude.ay, (float)attitude.az, 
            (float)attitude.gx, (float)attitude.gy, (float)attitude.gz,
            (float)attitude.mx, (float)attitude.my, (float)attitude.mz);
        attitude.sampled |= SAMPLED_YAW_EXACT;
    }
    attitude_compute_angles();
    // Filtro pasa bajo
	attitude.pitchspeed = attitude.pitchspeed * 0.7 + attitude.gy * 0.2;
    attitude.rollspeed = attitude.rollspeed * 0.7 - attitude.gx * 0.2;
	attitude.yawspeed = attitude.yawspeed * 0.7 - attitude.gz * 0.2;

    /*
    attitude.pitchspeed = (attitude.pitch - attitude.last_pitch) / attitude.deltat;
    attitude.last_pitch = attitude.pitch;
    attitude.rollspeed = (attitude.roll - attitude.last_roll) / attitude.deltat;
    attitude.last_roll = attitude.roll;
    attitude.yawspeed = (attitude.yaw - attitude.last_yaw) / attitude.deltat;
    attitude.last_yaw = attitude.yaw;
    */

	/*attitude.deltat = fusion.deltatUpdate();
	fusion.MahonyUpdate(
		(float)attitude.gx, (float)attitude.gy, (float)attitude.gz, 
		(float)attitude.ax, (float)attitude.ay, (float)attitude.az,
		attitude.deltat);
	attitude.roll = fusion.getRollRadians();
	attitude.pitch = fusion.getPitchRadians();
	attitude.yaw = fusion.getYawRadians();*/

    attitude.sampled |= SAMPLED_ATTITUDE;

    /*
    DEBUGPRINT("Roll:");DEBUGPRINT2(static_cast<float>(attitude.roll), 4);DEBUGPRINT("\t");
	DEBUGPRINT("Pitch:");DEBUGPRINT2(static_cast<float>(attitude.pitch), 4);DEBUGPRINT("\t");
	DEBUGPRINT("Yaw:");DEBUGPRINT2(static_cast<float>(attitude.yaw), 4);DEBUGPRINT("\t");
	DEBUGPRINTLN("");
    */
    
    /*
    DEBUGPRINT("Roll speed:");DEBUGPRINT2(static_cast<float>(attitude.rollspeed), 4);DEBUGPRINT("\t");
	DEBUGPRINT("Pitch speed:");DEBUGPRINT2(static_cast<float>(attitude.pitchspeed), 4);DEBUGPRINT("\t");
	DEBUGPRINT("Yaw speed:");DEBUGPRINT2(static_cast<float>(attitude.yawspeed), 4);DEBUGPRINT("\t");
	DEBUGPRINTLN("");
    */

}

void attitude_updateIMU(float ax, float ay, float az, float gx, float gy, float gz)
{
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Compute feedback only if accelerometer measurement valid
	// (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Estimated direction of gravity
		halfvx = attitude.q1 * attitude.q3 - attitude.q0 * attitude.q2;
		halfvy = attitude.q0 * attitude.q1 + attitude.q2 * attitude.q3;
		halfvz = attitude.q0 * attitude.q0 - 0.5 + attitude.q3 * attitude.q3;

		// Error is sum of cross product between estimated
		// and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		#ifdef ATTITUDE_TWOKI
        // integral error scaled by Ki
        attitude.integralFBx += ATTITUDE_TWOKI * halfex * attitude.deltat;
        attitude.integralFBy += ATTITUDE_TWOKI * halfey * attitude.deltat;
        attitude.integralFBz += ATTITUDE_TWOKI * halfez * attitude.deltat;
        gx += attitude.integralFBx;	// apply integral feedback
        gy += attitude.integralFBy;
        gz += attitude.integralFBz;
        #else
        attitude.integralFBx = 0.0;	// prevent integral windup
        attitude.integralFBy = 0.0;
        attitude.integralFBz = 0.0;
		#endif

		// Apply proportional feedback
		gx += ATTITUDE_TWOKP * halfex;
		gy += ATTITUDE_TWOKP * halfey;
		gz += ATTITUDE_TWOKP * halfez;
	}

	// Integrate rate of change of quaternion
	gx *= (0.5 * attitude.deltat);		// pre-multiply common factors
	gy *= (0.5 * attitude.deltat);
	gz *= (0.5 * attitude.deltat);
	qa = attitude.q0;
	qb = attitude.q1;
	qc = attitude.q2;
	attitude.q0 += (float)(-qb * gx - qc * gy - attitude.q3 * gz);
	attitude.q1 += (float)(qa * gx + qc * gz - attitude.q3 * gy);
	attitude.q2 += (float)(qa * gy - qb * gz + attitude.q3 * gx);
	attitude.q3 += (float)(qa * gz + qb * gy - qc * gx);

	// Normalise quaternion
	recipNorm = invSqrt(attitude.q0 * attitude.q0 + attitude.q1 * attitude.q1 + attitude.q2 * attitude.q2 + attitude.q3 * attitude.q3);
	attitude.q0 *= recipNorm;
	attitude.q1 *= recipNorm;
	attitude.q2 *= recipNorm;
	attitude.q3 *= recipNorm;
}

void attitude_compute_angles()
{
    /*
    attitude.roll = atan2f((float)(attitude.q0*attitude.q1 + attitude.q2*attitude.q3), (float)(0.5 - attitude.q1*attitude.q1 - attitude.q2*attitude.q2));
    attitude.pitch = asinf((float)(-2.0 * (attitude.q1*attitude.q3 - attitude.q0*attitude.q2)));
    attitude.yaw = atan2f((float)(attitude.q1*attitude.q2 + attitude.q0*attitude.q3), (float)(0.5 - attitude.q2*attitude.q2 - attitude.q3*attitude.q3));
    */
    
    /*
    float q1sq = attitude.q1*attitude.q1;

    attitude.roll = atan2f( 2.0 * (attitude.q3*attitude.q2 + attitude.q0*attitude.q1), 1.0 - 2.0 * (q1sq + attitude.q2*attitude.q2) );
    attitude.pitch = asinf( 2.0 * (attitude.q2*attitude.q0 - attitude.q3*attitude.q1));
    attitude.yaw = atan2f( 2.0 * (attitude.q3*attitude.q0 + attitude.q1*attitude.q2), -1.0 + 2.0 * (q1sq + attitude.q0*attitude.q0) );
    */

    float q2sq = attitude.q2 * attitude.q2;
    attitude.roll = -atan2f(attitude.q0 * attitude.q1 + attitude.q2 * attitude.q3, 0.5 - attitude.q1 * attitude.q1 - q2sq) - attitudeOffset.roll / 1000.0;
    attitude.pitch = asinf(-2.0 * (attitude.q1 * attitude.q3 - attitude.q0 * attitude.q2)) - attitudeOffset.pitch / 1000.0;
    attitude.yaw = atan2f(attitude.q1 * attitude.q2 + attitude.q0 * attitude.q3, 0.5 - q2sq - attitude.q3 * attitude.q3);
}


void attitude_update(
    float ax, float ay, float az, 
    float gx, float gy, float gz,
    float mx, float my, float mz) 
{

}


// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
/* Raíz cuadrada inversa */
float invSqrt(float x)
{
  float halfx = 0.5 * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5 - (halfx * y * y));
  y = y * (1.5 - (halfx * y * y));
  return y;
}

