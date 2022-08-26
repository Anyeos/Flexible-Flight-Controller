#include <Arduino.h>

// Default configuration file for Flexible Flight Controller
// Please, read carefully.
// Not on all chips (ie. AVR) can fit everything. If you are planning to make a micro drone you
// can use an Arduino Nano with an MPU6050 and some simple receiver but nothing more.
// Instead, if you want a more powerful flight controller you will need at least an STM32.

// You can copy this file to something like Config.h (or "configs/Your_Config_Name.h")
// to put your own configuration (look at config.h).


/******************
// Default values
******************/
// Default values to start with. But some can be changed by the user as a setting.

// A value to check in EEPROM to determine if it is the first time this Firmware
// is executed so all calibrations and defaults values are started.
// Change it if you want to force the above behaviour
#define FIRST_TIME_VALUE 0xbc

// Comment out to enable DEBUG
// Only recommended if you are developing 
#define DEBUG


//** Indicador para notificar el estado: **//
// 3 parpadeos lentos
//      Ha ocurrido un error (pudo haber fallado un sensor)

// Parpadeo rápido
//      El dron se inicializó sin error

// Led encendido
//      Algunos procedimientos demandante se están ejecutando (ej.: calibrando algo)
//      Si el led normalmente permanece encendido puede ser que hayas elegido mal el valor de LED_PIN_OFF

// Qué pin usar para el indicador LED?
#define LED_PIN LED_BUILTIN
#define LED_PIN_OFF 0 // Salida digital cuando el led está APAGADO


//** Batería **//
// Especifica de cuántas celdas en serie es tu batería.
// Nota: Necesitarás un circuito de divisor resistivo para medir el voltaje.
// Si quieres deshabilitar esta característica comenta el dato.
#define BATTERY_S_CELLS 3

// Pata uasada para medir el voltage de la batería (usando un divisor resistivo)
//#define BATTERY_PIN A0
// Relación del divisor resistivo
//#define BATTERY_DIVISOR 10
// El voltage del ADC (midelo con un multímetro para estar seguro o selecciona alguna referencia estable)
//#define BATTERY_ADC_VOLTAGE 3.3
// Define para especificar el Vref del ADC (no te olvides de también establecer BATTERY_ADC_VOLTAGE)
//#define BATTERY_SET_ADC_VREF INTERNAL



/************************/
// Sensores y agregados //
/*************************
Next, choose your sensors:
* You cannot choose two of the same kind.
* If you don't choose a gyro the only flight mode will be ACRO (manual)

Maybe in a future I will implement choosing secondary sensors
as workaround if the main sensor fails.
You don't need to have every kind of sensor neither.
A gyro is the minimum to auto level but if you have more sensors
you will get more features and control of your aircraft (it will
require more resources from your MCU too).

Some specs uses the I2C bus, some others a group of pins. 
You can override pin definitions except the I2C and SPI.

To know how to connect things please look at "doc" directory.
*******************************/

/*//////////////
Receiver
    Choose your receiver
////////////////*/
// Generic PPM sensor that uses only one pin to read data
//#define RECEIVER_PPM
// The PPM PIN input of your Arduino. Warning: Must support Interrupt
//#define RECEIVER_PPM_PIN 2

// PWM sensor that requires a pin for each channel
//#define RECEIVER_PWM

// FlySky
#define RECEIVER_FLYSKY
// Si no se define busca automáticamente el canal (pero demora más en conectar)
#define RECEIVER_FLYSKY_CHANNEL 80

// No receiver (you still can control it using mavlink/telemetry)
//#define RECEIVER_NO


/*//////////*
//  Motors
*///////////*/
// Standard PWM motor control
// It have the 4 pins to control each motor
#define ESC_PWM
// The PIN output of each PWM. PWM must be supported on that pin.
#define ESC1_PWM_PIN 6
#define ESC2_PWM_PIN 28
#define ESC3_PWM_PIN 27
#define ESC4_PWM_PIN 7


// It is for brushed motors that you can control directly with a MOSFET
//#define ESC_ANALOG
// The PIN output of each ESC. PWM must be supported on that pin.
//#define ESC1_ANALOG_PIN 3
//#define ESC2_ANALOG_PIN 5
//#define ESC3_ANALOG_PIN 6
//#define ESC4_ANALOG_PIN 9

// For testing purposes because if you don't have an ESC how do you think you can use your motors?
//#define ESC_NO

// Offsets
// Offsets de los motores para compensar por un desequilibrio en el peso del dron
// Valores en porcentaje de 0 a 10000 (10000 = 100%)
#define MOTOR1_OFFSET 0
#define MOTOR2_OFFSET 0
#define MOTOR3_OFFSET 0
#define MOTOR4_OFFSET 0



/*///////////////////
// Gyroscope o IMU //
///////////////////*/
// Sensor giroscópico usado para autonivelado

// El bus i2C donde conectarás el sensor giroscópico
#define GYRO_WIRE Wire

//** Roll, Pitch, yaw, algorithm **//
// Madgwick is more precise but slower, not recommended for AVR MCU
//#define GYRO_MADGWICK // Use Madgwick insted of Mahony
//#define GYRO_COMPASS // Use gyro with magnetometer information

// El bien conocido MPU6050 soportado nativamente (sin librería)
// Así que lo puedes usar con un MCU AVR sin problemas
#define GYRO_MPU6050
// Aplicar offsets por software en vez de escribirlos al sensor
#define MPU6050_SOFTWARE_OFFSETS

// A better implementation under MPU6050 using Motion Apps to get Roll, Pitch and Yaw
// Note: It does not support GYRO_WIRE, it will use the default i2C bus available
//#define GYRO_MPU6050_DMP

// A complete and very precise IMU that includes a compass as well.
// supported natively, so it is optimized to be used under AVR too.
//#define GYRO_MPU9250
#define MPU9250_I2CADDR 0x69

// Grove 6 axis digital accelerometer/gyroscope ADIS16470
// STILL UNSUPPORTED (unfinished, this does not work at all)
//#define GYRO_ADIS16470

// If you only want a racing drone you don't need a gyro.
// But the only flight mode will be ACRO (manual)
//#define GYRO_NO




/*////////////////
// Barometer
////////////////*/
// Barometer is used to get altitude by sensing pressure.

// i2C bus where to connect the barometer
#define BAROMETER_WIRE Wire

//#define BAROMETER_MS5611
// The well known BMP280 for something that is cheap (nothing is worst)
//#define BAROMETER_BMP280
// The economic BMP180
//#define BAROMETER_BMP180

#define BAROMETER_NO


/*/////////////////
// Compass
/////////////////*/
// To know where is the magnetic North.
// Don't really neccesary but is used to know looking at direction (yaw).
// Be sure to put it far away from metals

// The i2C bus where you will connect the compass sensor
#define COMPASS_WIRE Wire

// Generic and cheap usefull compass sensor
//#define COMPASS_HMC5883L
// You can opt to use the compass of the MPU9250, dont forget to put it far away from metals
//#define COMPASS_MPU9250

// No compass at all. Don't worry if you don't have a compass, it only gives the Yaw
#define COMPASS_NO


/*///////////////////
// GPS
///////////////////*/
// The Glogal Positioning System
// Really usefull for exact location and automatic travel but not required to make the drone auto level.
// Choose the serial port (be carefull because Telemetry uses a serial too)
//#define GPS_SERIAL Serial1
// No GPS
#define GPS_NO



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Ganancia PID y límites
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Los valores PID permiten controlar el autonivelado y el movimiento con aceleración como por ejemplo hacia arriba y abajo
// Tienes que ajustar muy bien estos valores de acuerdo a las características de tu dron.

// El autonivelado simula una persona queriendo nivelar el dron a través de un joystick.
// Cada PID se corresponde a un eje de una palanca del joystick simulado.

// KP es la influencia directa de la corrección. Si es muy alto se va a pasar de largo.
// KI es útil para aproximarse más al valor. Si es muy alta quedará un offset.
// KD ayuda a amortiguar y que no se pase de largo. Si es muy alta va a sufrir frenasos.

// Valores para predecir AHRS
// La integrada se puede comentar en vez de poner a 0
#define ATTITUDE_TWOKP 15.0
#define ATTITUDE_TWOKI 0.1

// Microsegundos del propio período del lazo PID
#define PID_US 2000 // 4000 us = 250 Hz

/** Pitch PID **/
#define KP_GAIN_PITCH 1.0
#define KI_GAIN_PITCH 0.01
#define KD_GAIN_PITCH 0.03
// Máximo ángulo de inclinación de la palanca del joystick simulado.
// Valores escalados (5000 = 90º)
#define PID_PITCH_MAX 800

/** Roll PID **/
#define KP_GAIN_ROLL 1.0
#define KI_GAIN_ROLL 0.01
#define KD_GAIN_ROLL 0.03
#define PID_ROLL_MAX 800

/** Yaw PID **/
#define KP_GAIN_YAW 0.2
#define KI_GAIN_YAW 0.0
#define KD_GAIN_YAW 0.005

/** Vertical velocity PID **/
#define KP_GAIN_VV 0.3
#define KI_GAIN_VV 0.0
#define KD_GAIN_VV 0.0
//#define PID_HZ_VV 250
// Throttle a m/s en velocidad
#define VVELOCITY_THROTTLE_DIVISOR 25



/******************************
    Other features
*******************************/

/**** Flight modes *****/

//** Standard. Stable auto level flight. **//
// The drone will try to stay stable and auto level fighting against forces like wind or inertia
// This mode is an important part of the capabilities of this software as autopilot.
#define FLIGHT_STANDARD 0x01

//** Cinema. Suitable for better filmmaking **//
// Same as standard but soft and slow movements to take better videos from a mounted camera
#define FLIGHT_CINEMA 0x02

//** Sport. A standard with very fast reaction **//
// It is more safe than ACROBAT and your drone will not crash if you just leave the controls.
// It will let you do flips and fast reactions.
#define FLIGHT_SPORT 0x03

//** The manual (ACRO) mode **//
// The drone will react by angular speed only provided by the human-pilot (no auto level).
// To fly in this mode you will need to practice a lot. You can crash your drone easily if you
// just leave controls alone. As an emergency you can set the fly mode to STANDARD when you
// are in trouble.
#define FLIGHT_ACROBAT 0x00
#define FLIGHT_MANUAL 0x00
// Acrobatic with extreme reaction
#define FLIGHT_ACRO_EXTREME 0x04

/** Flight Mode flags **/
// Enable control as vertical velocity instead of throttle
#define FLIGHT_FLAG_VVEL 0x10

// The flight modes order selections for the radio transmitter controller. It does not affect other order.
// Be careful because if you skip one the code will fail to understand you.
// If you only want 1, put the same on all.
#define FMODE0  FLIGHT_STANDARD | FLIGHT_FLAG_VVEL
//#define FMODE1  FLIGHT_STANDARD | FLIGHT_FLAG_VVEL
#define FMODE2  FLIGHT_STANDARD | FLIGHT_FLAG_VVEL
//#define FMODE0  FLIGHT_MANUAL
#define FMODE1  FLIGHT_MANUAL
//#define FMODE2  FLIGHT_ACROBAT



/* *******

Telemetry 

********* */
// Enable use of telemetry on a Serial UART port
// It is used to get information from the aircraft or to control and configure it.
// You can disable it but only if you want to save space (specially for Arduino Uno type boards).
//#define TELEMETRY_SERIAL Serial
#define TELEMETRY_LINK_SPEED 57600

// Choose supported features of telemetry
#define TELEMETRY_SEND_ATTITUDE
//#define TELEMETRY_SEND_COMPUTER // Sends telemetry with information about the MCU
//#define TELEMETRY_SEND_RC_CHANNELS // Send telemetry about RC channels
//#define TELEMETRY_SEND_RC_CHANNELS_SCALED // Send telemetry about RC channels filtered
//#define TELEMETRY_SEND_SYS_STATUS
//#define TELEMETRY_SEND_RADIO_STATUS
//#define TELEMETRY_SEND_HW_STATUS
//#define TELEMETRY_SEND_BATTERY_STATUS
//#define TELEMETRY_SEND_RAW_IMU
//#define TELEMETRY_SEND_RAW_PRESSURE
//#define TELEMETRY_SEND_ALTITUDE
