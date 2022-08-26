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

/*
	TODO: Abstract with Adafruit_I2CDevice
	ATENCIÓN: Código obsoleto! Corregir basándose en MPU6050.cpp
*/

/*
	MPU9250 - 9 ejes
	Este sensor giroscópico incorpora una brújula, pero como normalmente se coloca cerca de
	la placa principal, va a estar recibiendo demasiada interferencia magnética.
	Por defecto no aprovechamos su brújula.
	
	Para poder aprovechar la brújula deberás definir COMPASS_MPU9250 en tu Config.h
	pero asegúrate de colocarlo lejos de toda fuente de interferencia magnética (pilas, cables, motores, etc)

	define GYRO_COMPASS para hacer el cálculo junto con la brújula. 
	Esto dará más precisión y estabilidad.
	Nota:	Requiere alguna brújula (define COMPASS_MPU9250 para usar la propia)

	define GYRO_MADGWICK para usar un algoritmo más preciso.
	Atención:	El algoritmo Madgwick puede ser lento para un MCU AVR.
*/

#include "../../Global.h"

#ifdef GYRO_MPU9250

#ifndef MPU9250_I2CADDR
	#define MPU9250_I2CADDR 0x68
#endif

#include "gyro.h"
#include "../third_party/SensorFusion/SensorFusion.h"
SF fusion;
float deltat;


#include "MPU9250.h"
#include "../SaveLoad/saveload.h"
#include "../i2c/i2c.h"

#define GYRO_CALIBRATION_DISCARD 100
#define GYRO_CALIBRATION_MEASURES 1000
gyro_offset_t gyroOffset;
void gyro_calibrate( bool fast ) {
	DEBUGPRINTLN("MPU9250 calibrating...");
	// Enable Gyro DLPF
    writeMPU9250Register(MPU9250_GYRO_CONFIG, 
		readMPU9250Register8(MPU9250_GYRO_CONFIG) & 0xFC);

	// Lowest noise DLPF
	writeMPU9250Register(MPU9250_CONFIG, 
		readMPU9250Register8(MPU9250_CONFIG) & 0xF8 | MPU9250_DLPF_6);

	// Highest resolution
	writeMPU9250Register(MPU9250_GYRO_CONFIG, 
		readMPU9250Register8(MPU9250_GYRO_CONFIG) & 0xE7 | (MPU9250_GYRO_RANGE_1000<<3));

	writeMPU9250Register(MPU9250_ACCEL_CONFIG, 
		readMPU9250Register8(MPU9250_ACCEL_CONFIG) & 0xE7 | (MPU9250_ACC_RANGE_8G<<3));
    
	// Enable Acc DLPF
	writeMPU9250Register(MPU9250_ACCEL_CONFIG_2,
		readMPU9250Register8(MPU9250_ACCEL_CONFIG_2) & ~0x8);
    
	// Acc DLPF highest resolution
	writeMPU9250Register(MPU9250_ACCEL_CONFIG_2, 
		readMPU9250Register8(MPU9250_ACCEL_CONFIG_2) & 0xF8 | MPU9250_DLPF_6);
    delay(100);
    

    for(int i=0; i<GYRO_CALIBRATION_DISCARD; i++) {
		readMPU9250RawValues();
        delay(2);
    }
    
	// To calibrate put all to 0
	memset(&gyroOffset, 0, sizeof(gyroOffset));
	writeMPU9250Offsets();
	delay(100);

	int32_t Xaccel = 0;
	int32_t Yaccel = 0;
	int32_t Zaccel = 0;

	int32_t Xgyro = 0;
	int32_t Ygyro = 0;
	int32_t Zgyro = 0;

    for(int i=0; i<GYRO_CALIBRATION_MEASURES; i++){
        readMPU9250RawValues();
        Xaccel += raw_gyro_data.Xaccel;
		Yaccel += raw_gyro_data.Yaccel;
		Zaccel += raw_gyro_data.Zaccel;

        Xgyro += raw_gyro_data.Xgyro;
		Ygyro += raw_gyro_data.Ygyro;
		Zgyro += raw_gyro_data.Zgyro;
        delay(2);
    }

    Xaccel /= GYRO_CALIBRATION_MEASURES;
	Yaccel /= GYRO_CALIBRATION_MEASURES;
	Zaccel /= GYRO_CALIBRATION_MEASURES;
	// Substract gravity
	Zaccel -= 4096;
	
	gyroOffset.Xaccel = -(Xaccel>>1);
	gyroOffset.Yaccel = -(Yaccel>>1);
	gyroOffset.Zaccel = -(Zaccel>>1);
	//gyroOffset.Xaccel = -(Xaccel);
	//gyroOffset.Yaccel = -(Yaccel);
	//gyroOffset.Zaccel = -(Zaccel);


    Xgyro /= GYRO_CALIBRATION_MEASURES;
	Ygyro /= GYRO_CALIBRATION_MEASURES;
	Zgyro /= GYRO_CALIBRATION_MEASURES;

	gyroOffset.Xgyro = -Xgyro;
	gyroOffset.Ygyro = -Ygyro;
	gyroOffset.Zgyro = -Zgyro;

	//save_gyro_offset(gyroOffset);
}

uint8_t gyrRangeFactor;
uint8_t accRangeFactor;
void gyro_setup() {
    indicator_do();
	if (!InitMPU9250())
	{
		indicator_error();
		return;
	}
	DEBUGPRINTLN("MPU9250 Init OK");
	// Calibration goes here
	//if (first_time)
		gyro_calibrate();
	//load_gyro_offset(&gyroOffset);
	// 5118, -5942, 10166
	// 0, 0, 0
	//gyroOffset.Xaccel = 5118;
	//gyroOffset.Yaccel = -5942;
	//gyroOffset.Zaccel = 10166;
	//gyroOffset.Xaccel = 0;
	//gyroOffset.Yaccel = 0;
	//gyroOffset.Zaccel = 0;
	writeMPU9250Offsets();
	
	gyroOffset.Xaccel = readMPU9250Register8(MPU9250_XA_OFFSET_L);
	gyroOffset.Xaccel |= readMPU9250Register8(MPU9250_XA_OFFSET_H) << 8;
	gyroOffset.Yaccel = readMPU9250Register8(MPU9250_YA_OFFSET_L);
	gyroOffset.Yaccel |= readMPU9250Register8(MPU9250_YA_OFFSET_H) << 8;
	gyroOffset.Zaccel = readMPU9250Register8(MPU9250_ZA_OFFSET_L);
	gyroOffset.Zaccel |= readMPU9250Register8(MPU9250_ZA_OFFSET_H) << 8;

	gyroOffset.Xgyro = readMPU9250Register8(MPU9250_XG_OFFSET_L);
	gyroOffset.Xgyro |= readMPU9250Register8(MPU9250_XG_OFFSET_H) << 8;
	gyroOffset.Ygyro = readMPU9250Register8(MPU9250_YG_OFFSET_L);
	gyroOffset.Ygyro |= readMPU9250Register8(MPU9250_YG_OFFSET_H) << 8;
	gyroOffset.Zgyro = readMPU9250Register8(MPU9250_ZG_OFFSET_L);
	gyroOffset.Zgyro |= readMPU9250Register8(MPU9250_ZG_OFFSET_H) << 8;
	DEBUGPRINTLN("MPU9250 GyroOffsets: ");
	DEBUGPRINT(gyroOffset.Xaccel);DEBUGPRINT(", ");
	DEBUGPRINT(gyroOffset.Yaccel);DEBUGPRINT(", ");
	DEBUGPRINT(gyroOffset.Zaccel);
	DEBUGPRINTLN("");
	DEBUGPRINT(gyroOffset.Xgyro);DEBUGPRINT(", ");
	DEBUGPRINT(gyroOffset.Ygyro);DEBUGPRINT(", ");
	DEBUGPRINT(gyroOffset.Zgyro);
	DEBUGPRINTLN("");
	delay(2000);
	

	// Enable Gyro DLPF
    writeMPU9250Register(MPU9250_GYRO_CONFIG, 
		readMPU9250Register8(MPU9250_GYRO_CONFIG) & 0xFC);

	/* Digital Low Pass Filter for the gyroscope must be enabled to choose the level. 
	*  MPU9250_DPLF_0, MPU9250_DPLF_2, ...... MPU9250_DPLF_7 
	*  
	*  DLPF    Bandwidth [Hz]   Delay [ms]   Output Rate [kHz]
	*    0         250            0.97             8
	*    1         184            2.9              1
	*    2          92            3.9              1
	*    3          41            5.9              1
	*    4          20            9.9              1
	*    5          10           17.85             1
	*    6           5           33.48             1
	*    7        3600            0.17             8
	*    
	*    You achieve lowest noise using level 6  
	*/
	writeMPU9250Register(MPU9250_CONFIG, 
		readMPU9250Register8(MPU9250_CONFIG) & 0xF8 | MPU9250_DLPF_3);

	/* Sample rate divider divides the output rate of the gyroscope and accelerometer.
	*  Sample rate = Internal sample rate / (1 + divider) 
	*  It can only be applied if the corresponding DLPF is enabled and 0<DLPF<7!
	*  Divider is a number 0...255
	*/
	writeMPU9250Register(MPU9250_SMPLRT_DIV, 23);

	/* MPU9250_GYRO_RANGE_250       250 degrees per second (default)
	*  MPU9250_GYRO_RANGE_500       500 degrees per second
	*  MPU9250_GYRO_RANGE_1000     1000 degrees per second
	*  MPU9250_GYRO_RANGE_2000     2000 degrees per second
	*/
	MPU9250_gyroRange gyroRange = MPU9250_GYRO_RANGE_500;
  	writeMPU9250Register(MPU9250_GYRO_CONFIG, 
	  	readMPU9250Register8(MPU9250_GYRO_CONFIG) & 0xE7 | (gyroRange<<3));
  	gyrRangeFactor = (1<<gyroRange);

	/* MPU9250_ACC_RANGE_2G      2 g   (default)
	*  MPU9250_ACC_RANGE_4G      4 g
	*  MPU9250_ACC_RANGE_8G      8 g   
	*  MPU9250_ACC_RANGE_16G    16 g
	*/
	MPU9250_accRange accRange = MPU9250_ACC_RANGE_2G;
  	writeMPU9250Register(MPU9250_ACCEL_CONFIG, 
	  	readMPU9250Register8(MPU9250_ACCEL_CONFIG) & 0xE7 | (accRange<<3));
  	accRangeFactor = (1<<accRange);

	/* Enable/disable the digital low pass filter for the accelerometer 
	*  If disabled the bandwidth is 1.13 kHz, delay is 0.75 ms, output rate is 4 kHz
	*/
	writeMPU9250Register(MPU9250_ACCEL_CONFIG_2, 
		readMPU9250Register8(MPU9250_ACCEL_CONFIG_2) & ~0x8); // Enable (| 0x8 to disable)

	/* Digital low pass filter (DLPF) for the accelerometer, if enabled 
	*  MPU9250_DLPF_0, MPU9250_DLPF_2, ...... MPU9250_DLPF_7 
	*   DLPF     Bandwidth [Hz]      Delay [ms]    Output rate [kHz]
	*     0           460               1.94           1
	*     1           184               5.80           1
	*     2            92               7.80           1
	*     3            41              11.80           1
	*     4            20              19.80           1
	*     5            10              35.70           1
	*     6             5              66.96           1
	*     7           460               1.94           1
	*/
	writeMPU9250Register(MPU9250_ACCEL_CONFIG_2, 
		readMPU9250Register8(MPU9250_ACCEL_CONFIG_2) & 0xF8 | MPU9250_DLPF_3);

	// Enable FIFO
	writeMPU9250Register(MPU9250_USER_CTRL, 
		readMPU9250Register8(MPU9250_USER_CTRL) | 0x40 | 0x04); // 0x40 = enable, 0x04 = reset
	// Start FIFO
	writeMPU9250Register(MPU9250_FIFO_EN, MPU9250_FIFO_ACC_GYR_TEMP);

	raw_gyro_data.configured = true;
    indicator_done();
}

void gyro_loop() {
	uint16_t len = (uint16_t) readMPU9250Register16(MPU9250_FIFO_COUNT) & 0x1FFF;
	if (len >= 14) {
		attitude.yaw_exact = false;

		readMPU9250RawValuesFifo();

		/*writeMPU9250Register(MPU9250_USER_CTRL,
			readMPU9250Register8(MPU9250_USER_CTRL) | 0x04);*/

		//raw_gyro_data.Xaccel += (gyroOffset.Xaccel / accRangeFactor);
		//raw_gyro_data.Yaccel += (gyroOffset.Yaccel / accRangeFactor);
		//raw_gyro_data.Zaccel += (gyroOffset.Zaccel / accRangeFactor);

		attitude.ax = raw_gyro_data.Xaccel * accRangeFactor / 16384.0;
		attitude.ay = raw_gyro_data.Yaccel * accRangeFactor / 16384.0;
		attitude.az = raw_gyro_data.Zaccel * accRangeFactor / 16384.0;

		//raw_gyro_data.Xgyro += (gyroOffset.Xgyro / gyrRangeFactor);
		//raw_gyro_data.Ygyro += (gyroOffset.Ygyro / gyrRangeFactor);
		//raw_gyro_data.Zgyro += (gyroOffset.Zgyro / gyrRangeFactor);

		//fixed gx = raw_gyro_data.Xgyro * gyrRangeFactor * 250.0 / 32768.0 * DEG_TO_RAD;
		attitude.gx = raw_gyro_data.Xgyro * gyrRangeFactor * 0.000133158;
		//fixed gy = raw_gyro_data.Ygyro * gyrRangeFactor * 250.0 / 32768.0 * DEG_TO_RAD;
		attitude.gy = raw_gyro_data.Ygyro * gyrRangeFactor * 0.000133158;
		//fixed gz = raw_gyro_data.Zgyro * gyrRangeFactor * 250.0 / 32768.0 * DEG_TO_RAD;
		attitude.gz = raw_gyro_data.Zgyro * gyrRangeFactor * 0.000133158;

		attitude.rollspeed = gx;
		attitude.pitchspeed = gy;
		attitude.yawspeed = gz;
		attitude.imu_sampled = true;
	}

	#if defined(GYRO_MADGWICK)
	deltat = fusion.deltatUpdate();
	fusion.MadgwickUpdate(
		(float)attitude.gx, (float)attitude.gy, (float)attitude.gz, 
		(float)attitude.ax, (float)attitude.ay, (float)attitude.az,
		deltat);
	attitude.roll = fusion.getRollRadians();
	attitude.pitch = fusion.getPitchRadians();
	attitude.yaw = fusion.getYawRadians();
	#else
	deltat = fusion.deltatUpdate();
	fusion.MahonyUpdate(
		(float)attitude.gx, (float)attitude.gy, (float)attitude.gz, 
		(float)attitude.ax, (float)attitude.ay, (float)attitude.az,
		deltat);
	attitude.roll = fusion.getRollRadians();
	attitude.pitch = fusion.getPitchRadians();
	attitude.yaw = fusion.getYawRadians();
	#endif

	
	/*DEBUGPRINT("Xaccel:");DEBUGPRINT(raw_gyro_data.Xaccel);DEBUGPRINT("\t");
	DEBUGPRINT("Yaccel:");DEBUGPRINT(raw_gyro_data.Yaccel);DEBUGPRINT("\t");
	DEBUGPRINT("Zaccel:");DEBUGPRINT(raw_gyro_data.Zaccel);
	DEBUGPRINTLN("");*/

	
	/*DEBUGPRINT("Xgyro:");DEBUGPRINT(raw_gyro_data.Xgyro);DEBUGPRINT("\t");
	DEBUGPRINT("Ygyro:");DEBUGPRINT(raw_gyro_data.Ygyro);DEBUGPRINT("\t");
	DEBUGPRINT("Zgyro:");DEBUGPRINT(raw_gyro_data.Zgyro);
	DEBUGPRINTLN("");*/

	/*
	DEBUGPRINT("ax:");DEBUGPRINT2((float)ax,4);DEBUGPRINT(" ");
	DEBUGPRINT("ay:");DEBUGPRINT2((float)ay,4);DEBUGPRINT(" ");
	DEBUGPRINT("az:");DEBUGPRINT2((float)az,4);DEBUGPRINT(" ");
	DEBUGPRINTLN("");*/

	/*
	DEBUGPRINT("gx:");DEBUGPRINT2((float)gx,4);DEBUGPRINT(" ");
	DEBUGPRINT("gy:");DEBUGPRINT2((float)gy,4);DEBUGPRINT(" ");
	DEBUGPRINT("gz:");DEBUGPRINT2((float)gz,4);DEBUGPRINT(" ");
	DEBUGPRINTLN("");*/
	
	/*DEBUGPRINT("Roll:");DEBUGPRINT2(static_cast<float>(attitude.roll), 4);DEBUGPRINT("\t");
	DEBUGPRINT("Pitch:");DEBUGPRINT2(static_cast<float>(attitude.pitch), 4);DEBUGPRINT("\t");
	DEBUGPRINT("Yaw:");DEBUGPRINT2(static_cast<float>(attitude.yaw), 4);DEBUGPRINTLN("");*/
	//DEBUGPRINT("Temperature:");DEBUGPRINT2((raw_gyro_data.Temp / 100.0), 4);DEBUGPRINT(" ");
	

}	


bool InitMPU9250() {
	// Reset
	writeMPU9250Register(MPU9250_PWR_MGMT_1, MPU9250_RESET);
    delay(20);
    writeMPU9250Register(MPU9250_INT_PIN_CFG, MPU9250_BYPASS_EN);
    delay(10);

    if(readMPU9250Register8(MPU9250_WHO_AM_I) != MPU9250_WHO_AM_I_CODE) {
        return false;
    }
	// Disable sleep
	writeMPU9250Register(MPU9250_PWR_MGMT_1, readMPU9250Register8(MPU9250_PWR_MGMT_1) & ~(0x40));

#if defined(COMPASS_MPU9250)
	//enable I2C master
    writeMPU9250Register(MPU9250_USER_CTRL, 
		readMPU9250Register8(MPU9250_USER_CTRL) | MPU9250_I2C_MST_EN); 
    writeMPU9250Register(MPU9250_I2C_MST_CTRL, 0x00); // set I2C clock to 400 kHz
    delay(10);

	// Reset magnetometer
    writeAK8963Register(AK8963_CNTL_2, 0x01);
    delay(100);
#endif

	return true;
}

void readMPU9250RawValues() {
    if ((i2cRequestFrom(GYRO_WIRE, MPU9250_I2CADDR, MPU9250_ACCEL_OUT, 14) == 0) &&
		(GYRO_WIRE.available() > 0)
		) {
        raw_gyro_data.Xaccel = (GYRO_WIRE.read()<<8) + GYRO_WIRE.read();
		raw_gyro_data.Yaccel = (GYRO_WIRE.read()<<8) + GYRO_WIRE.read();
		raw_gyro_data.Zaccel = (GYRO_WIRE.read()<<8) + GYRO_WIRE.read();
		raw_gyro_data.Temp = (GYRO_WIRE.read()<<8) + GYRO_WIRE.read();
		raw_gyro_data.Xgyro = (GYRO_WIRE.read()<<8) + GYRO_WIRE.read();
		raw_gyro_data.Ygyro = (GYRO_WIRE.read()<<8) + GYRO_WIRE.read();
		raw_gyro_data.Zgyro = (GYRO_WIRE.read()<<8) + GYRO_WIRE.read();
    }
}

void readMPU9250RawValuesFifo() {
    if ((i2cRequestFrom(GYRO_WIRE, MPU9250_I2CADDR, MPU9250_FIFO_R_W, 14) == 0) &&
		(GYRO_WIRE.available() > 0)
		) {
        raw_gyro_data.Xaccel = (GYRO_WIRE.read()<<8) + GYRO_WIRE.read();
		raw_gyro_data.Yaccel = (GYRO_WIRE.read()<<8) + GYRO_WIRE.read();
		raw_gyro_data.Zaccel = (GYRO_WIRE.read()<<8) + GYRO_WIRE.read();
		raw_gyro_data.Temp = (GYRO_WIRE.read()<<8) + GYRO_WIRE.read();
		raw_gyro_data.Xgyro = (GYRO_WIRE.read()<<8) + GYRO_WIRE.read();
		raw_gyro_data.Ygyro = (GYRO_WIRE.read()<<8) + GYRO_WIRE.read();
		raw_gyro_data.Zgyro = (GYRO_WIRE.read()<<8) + GYRO_WIRE.read();
    }
}

uint8_t readMPU9250Register8(uint8_t reg) {
	return i2cReadRegister(GYRO_WIRE, MPU9250_I2CADDR, reg);
}

int16_t readMPU9250Register16(uint8_t reg) {
	return i2cReadRegister16(GYRO_WIRE, MPU9250_I2CADDR, reg);
}

void writeMPU9250Register(uint8_t reg, uint8_t val) {
	i2cWriteRegister(GYRO_WIRE, MPU9250_I2CADDR, reg, val);
}

void writeMPU9250Offsets() {
	writeMPU9250Register(MPU9250_XG_OFFSET_L, gyroOffset.Xgyro & 0xFF);
	writeMPU9250Register(MPU9250_XG_OFFSET_H, (gyroOffset.Xgyro >> 8) & 0xFF);
	
	writeMPU9250Register(MPU9250_YG_OFFSET_L, gyroOffset.Ygyro & 0xFF);
	writeMPU9250Register(MPU9250_YG_OFFSET_H, (gyroOffset.Ygyro >> 8) & 0xFF);
	
	writeMPU9250Register(MPU9250_ZG_OFFSET_L, gyroOffset.Zgyro & 0xFF);
	writeMPU9250Register(MPU9250_ZG_OFFSET_H, (gyroOffset.Zgyro >> 8) & 0xFF);


	writeMPU9250Register(MPU9250_XA_OFFSET_L, 
		(readMPU9250Register8(MPU9250_XA_OFFSET_L) & 0x1) | ((gyroOffset.Xaccel) & 0xFE));
	writeMPU9250Register(MPU9250_XA_OFFSET_H, (gyroOffset.Xaccel >> 8) & 0xFF);

	writeMPU9250Register(MPU9250_YA_OFFSET_L, 
		(readMPU9250Register8(MPU9250_XA_OFFSET_L) & 0x1) | ((gyroOffset.Yaccel) & 0xFE));
	writeMPU9250Register(MPU9250_YA_OFFSET_H, (gyroOffset.Yaccel >> 8) & 0xFF);

	writeMPU9250Register(MPU9250_ZA_OFFSET_L, 
		(readMPU9250Register8(MPU9250_XA_OFFSET_L) & 0x1) | ((gyroOffset.Zaccel) & 0xFE));
	writeMPU9250Register(MPU9250_ZA_OFFSET_H, (gyroOffset.Zaccel >> 8) & 0xFF);
}

void writeAK8963Register(uint8_t reg, uint8_t val) {
    writeMPU9250Register(MPU9250_I2C_SLV0_ADDR, AK8963_ADDRESS); // write AK8963
    writeMPU9250Register(MPU9250_I2C_SLV0_REG, reg); // define AK8963 register to be written to
    writeMPU9250Register(MPU9250_I2C_SLV0_DO, val);
}


#endif