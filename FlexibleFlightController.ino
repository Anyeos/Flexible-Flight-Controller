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

/*/////////////////////////////////////////////////////////////////////////////////////
// Safety note:
///////////////////////////////////////////////////////////////////////////////////////
  Although this flight controller try to be as much secure as can, you must take
  special care when you will try to start your aircraft. I am not responsible for
  any injury or damage that can result from the use of this code.
/////////////////////////////////////////////////////////////////////////////////////*/

#include <Arduino.h>
#include "Global.h"
#include "src/battery/battery.h"
#include "src/indicators/indicators.h"
#include "src/SaveLoad/saveload.h"
#include "src/gyros/gyro.h"
#include "src/receiver/receiver.h"
#include "src/telemetry/telemetry.h"
#include "src/motor/motor.h"
#include "src/compass/compass.h"
#include "src/barometer/barometer.h"
#include "src/ahrs/attitude.h"
#include "src/ahrs/altitude.h"
#include "src/gps/gps.h"


/*
#ifdef __STM32F1__
    TwoWire Wire(1, I2C_FAST_MODE);
#else
    TwoWire Wire;
#endif
*/

battery_t battery;

// Values obtained from the gyroscope
raw_gyro_data_t raw_gyro_data;

// Values for the barometer
barometer_data_t barometer_data;

waypoint_t takeoff_point;

altitude_t altitude;
attitude_t attitude;

// We are executing this firmware for first time?
// See: FIRST_TIME_VALUE
// Note: It is managed by saveload_setup()
bool first_time = false;

/*//////////////
 Setup routine
  Initialices everything
  Don't forget to "choose" what you use in your DIY hardware (look at config.h)
//////////////*/
void setup() {
#if defined(DEBUG)
  Serial.begin(57600);
  DEBUGPRINTLN(F("Flexible Flight Controller"));
  DEBUGPRINT(F("Version: "));
  DEBUGPRINTLN(VERSION);
#endif

  indicator_setup();

  saveload_setup();

  //terminal_setup();
  #ifdef BATTERY_S_CELLS
  battery_setup();
  #endif

  telemetry_setup();

  //indicator_do();
  Wire.begin();
  Wire.setClock(400000);
  //indicator_done();

  motor_setup();

	if (first_time)
		gyro_calibrate();
  raw_gyro_data.configured = false;
  gyro_setup();

  gps_setup();

  attitude_setup();

  barometer_setup();

  altitude_setup();

  receiver_setup();

  saveload_firsttime_done();
}


uint8_t state = STATE_INIT;
uint8_t flight_mode = FMODE0;
void loop() {
  switch (state) {
    case STATE_INIT:
    // Do some init stuff
    drone_init();
    break;
    
    case STATE_LANDED:
    // Ready to arm motors
    drone_landed();
    break;
    
    case STATE_ARMED:
    // Ready to fly
    drone_armed();
    break;
    
    case STATE_FLYING:
    drone_flying();
    break;
  }

  // Set control fly mode
  if (channel_scaled[CHANNEL_FMODE] < -2500) {
    SetFlyMode(FMODE0);
  } else
  if (channel_scaled[CHANNEL_FMODE] > 2500) {
    SetFlyMode(FMODE2);
  } else {
    SetFlyMode(FMODE1);
  }

  indicator_loop();

  // Get and process data
  // Order is important because the next overrides the previous
  #ifdef BATTERY_S_CELLS
  battery_loop();
  #endif
  gyro_loop();
  compass_loop();
  gps_loop();
  attitude_loop();
  barometer_loop();
  altitude_loop();
  receiver_loop();
  telemetry_loop();
  motor_loop();
  attitude.sampled = 0;
}


// One time execution
void drone_init() {
  indicator_delay(0b00001010);
  ChangeState(STATE_LANDED);
  //indicator_start(0b01010101, 100);
}


// Loop when is landed
unsigned long commandMillis = 0;
unsigned long waitMillis = 0;
void drone_landed() {
  unsigned long currentMillis = millis();
  if (waitMillis > currentMillis) {
    commandMillis = currentMillis;
  } else
  // Command to arm drone
  if  (
      channel_scaled[CHANNEL_ROLL] > 2500 &&
      channel_scaled[CHANNEL_PITCH] > 2500 &&
      channel_scaled[CHANNEL_THROTTLE] <= 0 &&
      channel_scaled[CHANNEL_YAW] < -2500
      )
  {
    if (currentMillis - commandMillis > 2000) {
      ChangeState(STATE_ARMED);
    }
  #ifndef GYRO_NO
  } else 
  if  (
      channel_scaled[CHANNEL_ROLL] < -2500 &&
      channel_scaled[CHANNEL_PITCH] > 2500 &
      channel_scaled[CHANNEL_THROTTLE] <= 0 &&
      channel_scaled[CHANNEL_YAW] > 2500
      )
  {
    if (currentMillis - commandMillis > 2000) {
      waitMillis = currentMillis + 5000; // Darle tiempo al receiver a estabilizarse
      gyro_calibrate();
      gyro_setup();
      attitude_calibrate();
    }
  #endif
  } else {
    commandMillis = currentMillis;
  }
}


// Ready to fly loop
void drone_armed() {
  if (channel_scaled[CHANNEL_THROTTLE] > 0) {
    ChangeState(STATE_FLYING);
  }
}

// Flying loop
void drone_flying() {
  if (channel_scaled[CHANNEL_THROTTLE] > 0) {
    indicator_start(0b00000001, 100000/channel_scaled[CHANNEL_THROTTLE] );
  }
  else 
  {
    indicator_stop();
  }
}


void ChangeState( uint8_t new_state ) {
  switch (new_state) {
    case STATE_ARMED:
      // Set take off point
    
      // Arm motors
      motor_arm();

      // Show user it is armed
      indicator_delay(0b00000101);
      break;
      
    case STATE_FLYING:
      indicator_start(0b00000001, 100);
      break;
  }

  state = new_state;
}


void SetFlyMode( uint8_t new_flightmode ) {
  uint8_t new_ff = new_flightmode & 0x0F; // filter flight mode flags
  if (
    (new_ff == FLIGHT_STANDARD ||
    new_ff == FLIGHT_CINEMA ||
    new_ff == FLIGHT_SPORT) &&
    (!raw_gyro_data.configured)
  ) {
    new_flightmode = FLIGHT_ACROBAT;
    //DEBUGPRINT(F("SetFlyMode"));
    indicator_error();
  }
  flight_mode = new_flightmode;
}

