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

/*
 
*/

#ifdef GYRO_MPU6050
#include "gyro.h"
#include "MPU6050.h"
#include "../SaveLoad/saveload.h"
#include "../i2c/i2c.h"

#ifndef MPU6050_I2CADDR
	#define MPU6050_I2CADDR 0x68
#endif

gyro_offset_t gyroOffset;
float gyrFactor;
float accFactor;

void MPU6050_printOffsets(int wait = 0)
{
	DEBUGPRINTLN("MPU6050 GyroOffsets: ");
	DEBUGPRINT(gyroOffset.Xaccel);DEBUGPRINT(", ");
	DEBUGPRINT(gyroOffset.Yaccel);DEBUGPRINT(", ");
	DEBUGPRINT(gyroOffset.Zaccel);
	DEBUGPRINTLN("");
	DEBUGPRINT(gyroOffset.Xgyro);DEBUGPRINT(", ");
	DEBUGPRINT(gyroOffset.Ygyro);DEBUGPRINT(", ");
	DEBUGPRINT(gyroOffset.Zgyro);
	DEBUGPRINTLN("");
	delay(wait);

}

#ifdef MPU6050_SOFTWARE_OFFSETS
float gyrOffsetX, gyrOffsetY, gyrOffsetZ;
float accOffsetX, accOffsetY, accOffsetZ;
void MPU6050CalculateSoftwareOffsets()
{
	gyrOffsetX = gyroOffset.Xgyro / 32.75 * 0.00872664625997164788;
	gyrOffsetY = gyroOffset.Ygyro / 32.75 * 0.00872664625997164788;
	gyrOffsetZ = gyroOffset.Zgyro / 32.75 * 0.00872664625997164788;

	accOffsetX = gyroOffset.Xaccel / 16384.0;
	accOffsetY = gyroOffset.Yaccel / 16384.0;
	accOffsetZ = gyroOffset.Zaccel / 16384.0;
}
#endif

#define GYRO_CALIBRATION_DISCARD 100
#define GYRO_CALIBRATION_MEASURES 1000
void gyro_calibrate( bool fast ) {
	indicator_do();
	DEBUGPRINTLN("MPU6050 calibrating...");
	if (!InitMPU6050()) { 
		DEBUGPRINTLN("Error: Cannot init MPU6050!");
		indicator_error();
		return;
	}

	// Lowest noise DLPF
	writeMPU6050Register(MPU6050_CONFIG, 
		readMPU6050Register8(MPU6050_CONFIG) & 0xF8 | (MPU6050_DLPF_BW_5));

	writeMPU6050Register(MPU6050_GYRO_CONFIG, 
		readMPU6050Register8(MPU6050_GYRO_CONFIG) & 0xE7 | (MPU6050_GYRO_FS_1000<<MPU6050_GCONFIG_FS_SEL_BIT));

	writeMPU6050Register(MPU6050_ACCEL_CONFIG, 
		readMPU6050Register8(MPU6050_ACCEL_CONFIG) & 0xE7 | (MPU6050_ACCEL_FS_2<<MPU6050_ACONFIG_AFS_SEL_BIT));
    
    delay(100);

    for(int i=0; i<GYRO_CALIBRATION_DISCARD; i++) {
		readMPU6050RawValues();
        delay(2);
    }
    
	// To calibrate put all to 0
	memset(&gyroOffset, 0, sizeof(gyroOffset));
	#ifndef MPU6050_SOFTWARE_OFFSETS
	writeMPU6050Offsets();
	#endif
	delay(100);

	int32_t Xaccel = 0;
	int32_t Yaccel = 0;
	int32_t Zaccel = 0;

	int32_t Xgyro = 0;
	int32_t Ygyro = 0;
	int32_t Zgyro = 0;
    for(int i=0; i<GYRO_CALIBRATION_MEASURES; i++){
        readMPU6050RawValues();
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
	Zaccel -= 16384;

	gyroOffset.Xaccel = -(Xaccel / 9.25);
	gyroOffset.Yaccel = -(Yaccel / 9.25);
	gyroOffset.Zaccel = -(Zaccel / 9.25);


    Xgyro /= GYRO_CALIBRATION_MEASURES;
	Ygyro /= GYRO_CALIBRATION_MEASURES;
	Zgyro /= GYRO_CALIBRATION_MEASURES;

	gyroOffset.Xgyro = -Xgyro;
	gyroOffset.Ygyro = -Ygyro;
	gyroOffset.Zgyro = -Zgyro;

	MPU6050_printOffsets(0);

	#ifndef MPU6050_SOFTWARE_OFFSETS
	writeMPU6050Offsets();
	gyroOffset.Xaccel = readMPU6050Register8(MPU6050_XA_OFFS_L_TC);
	gyroOffset.Xaccel |= readMPU6050Register8(MPU6050_XA_OFFS_H) << 8;
	gyroOffset.Yaccel = readMPU6050Register8(MPU6050_YA_OFFS_L_TC);
	gyroOffset.Yaccel |= readMPU6050Register8(MPU6050_YA_OFFS_H) << 8;
	gyroOffset.Zaccel = readMPU6050Register8(MPU6050_ZA_OFFS_L_TC);
	gyroOffset.Zaccel |= readMPU6050Register8(MPU6050_ZA_OFFS_H) << 8;

	gyroOffset.Xgyro = readMPU6050Register8(MPU6050_XG_OFFS_USRL);
	gyroOffset.Xgyro |= readMPU6050Register8(MPU6050_XG_OFFS_USRH) << 8;
	gyroOffset.Ygyro = readMPU6050Register8(MPU6050_YG_OFFS_USRL);
	gyroOffset.Ygyro |= readMPU6050Register8(MPU6050_YG_OFFS_USRH) << 8;
	gyroOffset.Zgyro = readMPU6050Register8(MPU6050_ZG_OFFS_USRL);
	gyroOffset.Zgyro |= readMPU6050Register8(MPU6050_ZG_OFFS_USRH) << 8;
	MPU6050_printOffsets();
	#else
	MPU6050CalculateSoftwareOffsets();
	#endif
	//save_gyro_offset(gyroOffset);

	indicator_done();
}


void gyro_setup() {
    indicator_do();

	if (!InitMPU6050())
	{
		DEBUGPRINTLN("MPU6050 init error!");
		indicator_error();
		return;
	}
	DEBUGPRINTLN("MPU6050 Init OK");
	//load_gyro_offset(&gyroOffset);
	#ifndef MPU6050_SOFTWARE_OFFSETS
	writeMPU6050Offsets();

	/*gyroOffset.Xaccel = readMPU6050Register8(MPU6050_XA_OFFS_L_TC);
	gyroOffset.Xaccel |= readMPU6050Register8(MPU6050_XA_OFFS_H) << 8;
	gyroOffset.Yaccel = readMPU6050Register8(MPU6050_YA_OFFS_L_TC);
	gyroOffset.Yaccel |= readMPU6050Register8(MPU6050_YA_OFFS_H) << 8;
	gyroOffset.Zaccel = readMPU6050Register8(MPU6050_ZA_OFFS_L_TC);
	gyroOffset.Zaccel |= readMPU6050Register8(MPU6050_ZA_OFFS_H) << 8;

	gyroOffset.Xgyro = readMPU6050Register8(MPU6050_XG_OFFS_USRL);
	gyroOffset.Xgyro |= readMPU6050Register8(MPU6050_XG_OFFS_USRH) << 8;
	gyroOffset.Ygyro = readMPU6050Register8(MPU6050_YG_OFFS_USRL);
	gyroOffset.Ygyro |= readMPU6050Register8(MPU6050_YG_OFFS_USRH) << 8;
	gyroOffset.Zgyro = readMPU6050Register8(MPU6050_ZG_OFFS_USRL);
	gyroOffset.Zgyro |= readMPU6050Register8(MPU6050_ZG_OFFS_USRH) << 8;
	MPU6050_printOffsets();*/
	#else
	//MPU6050CalculateSoftwareOffsets();
	#endif


	// Enable DLPF
	/*
	*          |   ACCELEROMETER    |           GYROSCOPE
	* DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
	* ---------+-----------+--------+-----------+--------+-------------
	* 0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
	* 1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
	* 2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
	* 3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
	* 4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
	* 5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
	* 6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
	* 7        |   -- Reserved --   |   -- Reserved --   | Reserved
	*/
	writeMPU6050Register(MPU6050_CONFIG, 
		readMPU6050Register8(MPU6050_CONFIG) & 0xF8 | (MPU6050_DLPF_BW_188));

	/* Sample rate divider divides the output rate of the gyroscope and accelerometer.
	*  Sample rate = Internal sample rate / (1 + divider) 
	*  It can only be applied if the corresponding DLPF is enabled and 0<DLPF<7!
	*  Divider is a number 0...255
	*/
	writeMPU6050Register(MPU6050_SMPLRT_DIV, 0);

	/*
	* FS_SEL | Full Scale Range   | LSB Sensitivity
	* -------+--------------------+----------------
	* 0      | +/- 250 degrees/s  | 131 LSB/deg/s
	* 1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
	* 2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
	* 3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
	*/
	uint8_t gyroRange = MPU6050_GYRO_FS_250;
	writeMPU6050Register(MPU6050_GYRO_CONFIG, 
		readMPU6050Register8(MPU6050_GYRO_CONFIG) & 0xE7 | (gyroRange<<MPU6050_GCONFIG_FS_SEL_BIT));
	// Conversión completa LSB y DEG a RAD
	gyrFactor = (1<<gyroRange)/131.0*0.00872664625997164788;

	/*
	* AFS_SEL | Full Scale Range | LSB Sensitivity
	* --------+------------------+----------------
	* 0       | +/- 2g           | 8192 LSB/mg
	* 1       | +/- 4g           | 4096 LSB/mg
	* 2       | +/- 8g           | 2048 LSB/mg
	* 3       | +/- 16g          | 1024 LSB/mg
	*/
	uint8_t accRange = MPU6050_ACCEL_FS_2;
	writeMPU6050Register(MPU6050_ACCEL_CONFIG, 
		readMPU6050Register8(MPU6050_ACCEL_CONFIG) & 0xE7 | (accRange<<MPU6050_ACONFIG_AFS_SEL_BIT));
  	accFactor = (1<<accRange)/16384.0;

	// Enable FIFO
	writeMPU6050Register(MPU6050_USER_CTRL, 
		readMPU6050Register8(MPU6050_USER_CTRL) | 
			(1<<MPU6050_USERCTRL_FIFO_EN_BIT) |
			(1<<MPU6050_USERCTRL_FIFO_RESET_BIT));
	// Start FIFO
	writeMPU6050Register(MPU6050_FIFO_EN, 
		(1<<MPU6050_TEMP_FIFO_EN_BIT) |
		(1<<MPU6050_XG_FIFO_EN_BIT) |
		(1<<MPU6050_YG_FIFO_EN_BIT) |
		(1<<MPU6050_ZG_FIFO_EN_BIT) |
		(1<<MPU6050_ACCEL_FIFO_EN_BIT));

	raw_gyro_data.configured = true;
    indicator_done();
}

void gyro_loop() {
	uint16_t len = (uint16_t) (readMPU6050Register8(MPU6050_FIFO_COUNTH)<<8);
	len |= (uint16_t)(readMPU6050Register8(MPU6050_FIFO_COUNTL)) & 0x1FFF;
	if (len < 14) { return; }

	readMPU6050RawValuesFifo();

	/*writeMPU9250Register(MPU9250_USER_CTRL,
		readMPU9250Register8(MPU9250_USER_CTRL) | 0x04);*/

	#ifndef MPU6050_SOFTWARE_OFFSETS
	// meters / sec
	attitude.ax = raw_gyro_data.Xaccel * accFactor;
	attitude.ay = raw_gyro_data.Yaccel * accFactor;
	attitude.az = raw_gyro_data.Zaccel * accFactor;

	// radians / sec
	attitude.gx = raw_gyro_data.Xgyro * gyrFactor;
	attitude.gy = raw_gyro_data.Ygyro * gyrFactor;
	attitude.gz = raw_gyro_data.Zgyro * gyrFactor;
	#else
	// meters / sec
	attitude.ax = raw_gyro_data.Xaccel * accFactor - accOffsetX;
	attitude.ay = raw_gyro_data.Yaccel * accFactor - accOffsetY;
	attitude.az = raw_gyro_data.Zaccel * accFactor - accOffsetZ;

	// radians / sec
	attitude.gx = raw_gyro_data.Xgyro * gyrFactor - gyrOffsetX;
	attitude.gy = raw_gyro_data.Ygyro * gyrFactor - gyrOffsetY;
	attitude.gz = raw_gyro_data.Zgyro * gyrFactor - gyrOffsetZ;
	#endif

	attitude.sampled |= SAMPLED_IMU;

	/*AHRS.deltatUpdate();
	AHRS.updateIMU();
	attitude.roll = AHRS.getRoll();
	attitude.pitch = AHRS.getPitch();
	attitude.yaw = AHRS.getYaw();*/


	/*
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
	*/

	/*DEBUGPRINT("Xaccel:");DEBUGPRINT(raw_gyro_data.Xaccel);DEBUGPRINT("\t");
	DEBUGPRINT("Yaccel:");DEBUGPRINT(raw_gyro_data.Yaccel);DEBUGPRINT("\t");
	DEBUGPRINT("Zaccel:");DEBUGPRINT(raw_gyro_data.Zaccel);
	DEBUGPRINTLN("");
	
	DEBUGPRINT("Xgyro:");DEBUGPRINT(raw_gyro_data.Xgyro);DEBUGPRINT("\t");
	DEBUGPRINT("Ygyro:");DEBUGPRINT(raw_gyro_data.Ygyro);DEBUGPRINT("\t");
	DEBUGPRINT("Zgyro:");DEBUGPRINT(raw_gyro_data.Zgyro);
	DEBUGPRINTLN("");*/

	/*
	DEBUGPRINT("ax:");DEBUGPRINT2((float)attitude.ax,4);DEBUGPRINT("\t");
	DEBUGPRINT("ay:");DEBUGPRINT2((float)attitude.ay,4);DEBUGPRINT("\t");
	DEBUGPRINT("az:");DEBUGPRINT2((float)attitude.az,4);DEBUGPRINT("\t");
	DEBUGPRINTLN("");
	*/

	/*
	DEBUGPRINT("gx:");DEBUGPRINT2((float)attitude.gx,4);DEBUGPRINT("\t");
	DEBUGPRINT("gy:");DEBUGPRINT2((float)attitude.gy,4);DEBUGPRINT("\t");
	DEBUGPRINT("gz:");DEBUGPRINT2((float)attitude.gz,4);DEBUGPRINT("\t");
	DEBUGPRINTLN("");
	*/
	
	//DEBUGPRINT("Temperature:");DEBUGPRINT2((raw_gyro_data.Temp / 100.0), 4);DEBUGPRINT(" ");
	//DEBUGPRINTLN("");

}	


bool InitMPU6050() {
	// Reset
	uint8_t status = writeMPU6050Register(MPU6050_PWR_MGMT_1, 1<<MPU6050_PWR1_DEVICE_RESET_BIT);
    delay(20);

    writeMPU6050Register(MPU6050_INT_PIN_CFG, 1<<MPU6050_INTCFG_I2C_BYPASS_EN_BIT);
    delay(10);

	uint8_t deviceID = readMPU6050Register8(MPU6050_WHO_AM_I)>>1;
    if ((deviceID & 0x3F) != 0x34) {
		DEBUGPRINT(F("Warning: Not a real MPU6050. DeviceID is: "));
		DEBUGPRINTLN(deviceID & 0x3F);
        //return false;
    }

	// Clock Source
	writeMPU6050Register(MPU6050_PWR_MGMT_1, 
		readMPU6050Register8(MPU6050_PWR_MGMT_1) & 0xF8 | (MPU6050_CLOCK_PLL_XGYRO<<MPU6050_PWR1_CLKSEL_BIT));

	// Disable sleep
	writeMPU6050Register(MPU6050_PWR_MGMT_1, 
		readMPU6050Register8(MPU6050_PWR_MGMT_1) & ~(1<<MPU6050_PWR1_SLEEP_BIT));
	delay(20);

	return status == 0;
}

void readMPU6050RawValues() {
    if ((i2cRequestFrom(GYRO_WIRE, MPU6050_I2CADDR, MPU6050_ACCEL_XOUT_H, 14) == 0) &&
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

void readMPU6050RawValuesFifo() {
    if ((i2cRequestFrom(GYRO_WIRE, MPU6050_I2CADDR, MPU6050_FIFO_R_W, 14) == 0) &&
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

uint8_t readMPU6050Register8(uint8_t reg) {
	return i2cReadRegister(GYRO_WIRE, MPU6050_I2CADDR, reg);
}

int16_t readMPU6050Register16(uint8_t reg) {
	return i2cReadRegister16(GYRO_WIRE, MPU6050_I2CADDR, reg);
}

uint8_t writeMPU6050Register(uint8_t reg, uint8_t val) {
    return i2cWriteRegister(GYRO_WIRE, MPU6050_I2CADDR, reg, val);
}

void writeMPU6050Offsets() {
	writeMPU6050Register(MPU6050_XG_OFFS_USRL, gyroOffset.Xgyro & 0xFF);
	writeMPU6050Register(MPU6050_XG_OFFS_USRH, (gyroOffset.Xgyro >> 8) & 0xFF);
	
	writeMPU6050Register(MPU6050_YG_OFFS_USRL, gyroOffset.Ygyro & 0xFF);
	writeMPU6050Register(MPU6050_YG_OFFS_USRH, (gyroOffset.Ygyro >> 8) & 0xFF);
	
	writeMPU6050Register(MPU6050_ZG_OFFS_USRL, gyroOffset.Zgyro & 0xFF);
	writeMPU6050Register(MPU6050_ZG_OFFS_USRH, (gyroOffset.Zgyro >> 8) & 0xFF);


	writeMPU6050Register(MPU6050_XA_OFFS_L_TC, 
		(readMPU6050Register8(MPU6050_XA_OFFS_L_TC) & 0x1) | ((gyroOffset.Xaccel) & 0xFE));
	writeMPU6050Register(MPU6050_XA_OFFS_H, (gyroOffset.Xaccel >> 8) & 0xFF);

	writeMPU6050Register(MPU6050_YA_OFFS_L_TC, 
		(readMPU6050Register8(MPU6050_YA_OFFS_L_TC) & 0x1) | ((gyroOffset.Yaccel) & 0xFE));
	writeMPU6050Register(MPU6050_YA_OFFS_H, (gyroOffset.Yaccel >> 8) & 0xFF);

	writeMPU6050Register(MPU6050_ZA_OFFS_L_TC, 
		(readMPU6050Register8(MPU6050_ZA_OFFS_L_TC) & 0x1) | ((gyroOffset.Zaccel) & 0xFE));
	writeMPU6050Register(MPU6050_ZA_OFFS_H, (gyroOffset.Zaccel >> 8) & 0xFF);
}

#endif