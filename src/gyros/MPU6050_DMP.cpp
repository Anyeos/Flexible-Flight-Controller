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

#ifdef GYRO_MPU6050_DMP

#define GYRO_CONFIGURED true

#include "gyro.h"
#include "../third_party/MPU6050/I2Cdev.h"
//#include "MPU6050_6Axis_MotionApps20.h"
#include "../third_party/MPU6050/MPU6050_6Axis_MotionApps_V6_12.h"
//#include "MPU6050.h"
#include "../SaveLoad/saveload.h"
MPU6050 mpu;

void gyro_calibrate( bool fast ) {
	// Atención: Para calibrar el sensor tiene que estar al derecho (el chip mirando hacia arriba)
	if (fast) {
		mpu.CalibrateAccel(6);
		mpu.CalibrateGyro(6);
	} else {
		mpu.CalibrateAccel();
		mpu.CalibrateGyro();
	}

	gyro_offset_t gyro_offset;
	gyro_offset.Xaccel =  mpu.getXAccelOffset();
	gyro_offset.Yaccel =  mpu.getYAccelOffset();
	gyro_offset.Zaccel =  mpu.getZAccelOffset();

	gyro_offset.Xgyro = mpu.getXGyroOffset();
	gyro_offset.Ygyro = mpu.getYGyroOffset();
	gyro_offset.Zgyro = mpu.getZGyroOffset();

	save_gyro_offset(gyro_offset);
}

void gyro_setup() {
    indicator_do();
    mpu.initialize();
	if (!mpu.testConnection()) 
	{
		indicator_error();
		return;
	}

	if (mpu.dmpInitialize() != 0) {
		indicator_error();
		return;
	}

	if (first_time)
		gyro_calibrate();
	gyro_offset_t gyro_offset;
    load_gyro_offset(&gyro_offset);
	mpu.setXAccelOffset(gyro_offset.Xaccel);
	mpu.setYAccelOffset(gyro_offset.Yaccel);
	mpu.setZAccelOffset(gyro_offset.Zaccel);
	mpu.setXGyroOffset(gyro_offset.Xgyro);
	mpu.setYGyroOffset(gyro_offset.Ygyro);
	mpu.setZGyroOffset(gyro_offset.Zgyro);

	mpu.setDMPEnabled(true);
	mpu.dmpGetFIFOPacketSize();
    indicator_done();
}

uint8_t fifoBuffer[64];
void gyro_loop() {
	// Obtener los datos RAW nos quita algo de tiempo.
	// Sólo lo haremos si realmente es necesario.
    /*mpu.getMotion6(
		&raw_gyro_data.Xaccel,
		&raw_gyro_data.Yaccel,
		&raw_gyro_data.Zaccel,
		&raw_gyro_data.Xgyro,
		&raw_gyro_data.Ygyro,
		&raw_gyro_data.Zgyro
		);

	raw_gyro_data.Temp = mpu.getTemperature();*/
    /*gyro_data.temp = (gyro_data.rawTemp + 12412.0) / 340.0;

    gyro_data.accX = ((float)gyro_data.rawAccX) / 16384.0;
    gyro_data.accY = ((float)gyro_data.rawAccY) / 16384.0;
    gyro_data.accZ = ((float)gyro_data.rawAccZ) / 16384.0;*/

	if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
		VectorFloat gravity;
		Quaternion q;
		float ypr[3];
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);

		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
		attitude.yaw = ypr[0];
		attitude.pitch = ypr[1];
		attitude.roll = ypr[2];

		VectorInt16 aa;
		VectorInt16 aaReal;

		//mpu.dmpGetAccel(&aa, fifoBuffer);
		//mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
		VectorInt16 gyro;
		//mpu.getFullScaleGyroRange(); 
		mpu.dmpGetGyro(&gyro, fifoBuffer);
		attitude.rollspeed = gyro.x / 131.0; // MPU6050_GYRO_FS_250
		attitude.pitchspeed = gyro.y / 131.0;
		attitude.yawspeed = gyro.z / 131.0;
		
		/*long temp;
		mpu.dmpGetTemperature(&temp, fifoBuffer);
		gyro_data.temp = temp;*/
	}
}

#endif