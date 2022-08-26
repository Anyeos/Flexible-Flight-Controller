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
    Motor control for quadcopter
    Acrobatic and auto level
*/
#include "../../Global.h"
#include "motor.h"
#include "esc.h"
//#include <FastPID.h>
#include "DronePID.h"

// Fixed point calculations, 2 decimals precision
#define MOTOR_FULL_THROTTLE 10000 // 100%
#define MOTOR_MAX_THROTTLE MOTOR_FULL_THROTTLE
#define MOTOR_ZERO_THROTTLE 0

motor_t motor;

/*unsigned long PID_last_time = 0;
FastPID PID_Roll(KP_GAIN_ROLL, KI_GAIN_ROLL, KD_GAIN_ROLL, PID_HZ, 16, true);
int16_t pid_roll = 0;
FastPID PID_Pitch(KP_GAIN_PITCH, KI_GAIN_PITCH, KD_GAIN_PITCH, PID_HZ, 16, true);
int16_t pid_pitch = 0;
FastPID PID_Yaw(KP_GAIN_YAW, KI_GAIN_YAW, KD_GAIN_YAW, PID_HZ, 16, true);*/

DronePID PID_Pitch(KP_GAIN_PITCH, KI_GAIN_PITCH, KD_GAIN_PITCH, -PID_PITCH_MAX, PID_PITCH_MAX);
DronePID PID_Roll(KP_GAIN_ROLL, KI_GAIN_ROLL, KD_GAIN_ROLL, -PID_ROLL_MAX, PID_ROLL_MAX);
int16_t pid_pitch;
int16_t pid_roll;
DronePID PID_Yaw(KP_GAIN_YAW, KI_GAIN_YAW, KD_GAIN_YAW);
int16_t pid_yaw;

#if !defined(GYRO_NO) && !defined(BAROMETER_NO)
//FastPID PID_VV(KP_GAIN_VV, KI_GAIN_VV, KD_GAIN_VV, PID_HZ_VV, 16, true);
int16_t vvelocity_throttle = 0;
unsigned long vvelocity_last_millis = 0;
#endif

void motor_zero() {
    motor.motor1 = MOTOR_ZERO_THROTTLE;
    motor.motor2 = MOTOR_ZERO_THROTTLE;
    motor.motor3 = MOTOR_ZERO_THROTTLE;
    motor.motor4 = MOTOR_ZERO_THROTTLE;
}

void motor_setup() {
    indicator_do();
    motor_zero();

    /*PID_Roll.setOutputRange(-5000, 5000);
    PID_Pitch.setOutputRange(-5000, 5000);
    PID_Yaw.setOutputRange(-5000, 5000);*/
    pid_pitch = 0;
    pid_roll = 0;
    pid_yaw = 0;

    motor.armed = false;
    esc_setup();
    indicator_done();
}

void motor_limit();
void motor_direct_loop(int16_t throttle, int16_t roll, int16_t pitch, int16_t yaw);
void motor_autolevel_loop(int16_t throttle, int16_t roll, int16_t pitch, int16_t yaw, bool vertical_velocity = false);

void motor_loop() {
    if (!motor.armed) {
        motor_zero();
        return;
    }

#if !defined(GYRO_NO) && !defined(BAROMETER_NO)
    bool vvel = flight_mode & FLIGHT_FLAG_VVEL;
#else
    #define vvel false
#endif
    switch (flight_mode & 0x0F) {
        case FLIGHT_STANDARD:
        motor_autolevel_loop(
            vvel?channel_throttle_centered:channel_scaled[CHANNEL_THROTTLE],
            channel_scaled[CHANNEL_ROLL] / 7,
            channel_scaled[CHANNEL_PITCH] / 7,
            channel_scaled[CHANNEL_YAW] / 5, 
            vvel);
        break;

        case FLIGHT_CINEMA:
        motor_autolevel_loop(
            vvel?channel_throttle_centered:channel_scaled[CHANNEL_THROTTLE],
            channel_scaled[CHANNEL_ROLL] / 8,
            channel_scaled[CHANNEL_PITCH] / 8,
            channel_scaled[CHANNEL_YAW] / 6, 
            vvel);
        break;
        
        case FLIGHT_SPORT:
        motor_autolevel_loop(
            vvel?channel_throttle_centered:channel_scaled[CHANNEL_THROTTLE],
            channel_scaled[CHANNEL_ROLL] / 6,
            channel_scaled[CHANNEL_PITCH] / 6,
            channel_scaled[CHANNEL_YAW] / 4,
            vvel);
        break;

        case FLIGHT_ACRO_EXTREME:
        motor_direct_loop(
            channel_scaled[CHANNEL_THROTTLE],
            channel_scaled[CHANNEL_ROLL] / 6,
            channel_scaled[CHANNEL_PITCH] / 6,
            channel_scaled[CHANNEL_YAW] / 4);
        break;

        default:
        motor_direct_loop(
            channel_scaled[CHANNEL_THROTTLE],
            channel_scaled[CHANNEL_ROLL] / 8,
            channel_scaled[CHANNEL_PITCH] / 8,
            channel_scaled[CHANNEL_YAW] / 5);
    }

    // Aplicamos los valores a los motores reales
    esc_loop();
}

// Disposición de motor en X
/*
  1    2
  o    o
   \  /
    []
   /  \
  o    o
  4    3
*/

// Control manual o acrobático. No se usa el giroscopio sino que se controlan directamente
// los motores del dron.
// throttle = Impulso total de los motores (0 a 10000)
// roll = inclinación lateral izquierda / derecha (-5000 / 5000)
// pitch = inclinación de frente / espalda (-5000 / 5000)
// yaw = giro sobre si mismo antihorario / horario (-5000 / 5000)
void motor_direct_loop(int16_t throttle, int16_t roll, int16_t pitch, int16_t yaw) {
    /*fixed dist1 = sqrt(sq(-5000-roll)+sq(-5000-pitch));
    dist1 = (dist1-7071)/7071;
    fixed dist2 = sqrt(sq(5000-roll)+sq(-5000-pitch));
    dist2 = (dist2-7071)/7071;
    fixed dist3 = sqrt(sq(5000-roll)+sq(5000-pitch));
    dist3 = (dist3-7071)/7071;
    fixed dist4 = sqrt(sq(-5000-roll)+sq(5000-pitch));
    dist4 = (dist4-7071)/7071;*/

    motor.motor1 = throttle;
    motor.motor2 = throttle;
    motor.motor3 = throttle;
    motor.motor4 = throttle;
    if (throttle > 0)
    {
        /*motor.motor1 += (int16_t)(dist1*push);
        motor.motor2 += (int16_t)(dist2*push);
        motor.motor3 += (int16_t)(dist3*push);
        motor.motor4 += (int16_t)(dist4*push);*/
        motor.motor1 = throttle + roll + pitch - yaw + MOTOR1_OFFSET;
        motor.motor2 = throttle - roll + pitch + yaw + MOTOR2_OFFSET;
        motor.motor3 = throttle - roll - pitch - yaw + MOTOR3_OFFSET;
        motor.motor4 = throttle + roll - pitch + yaw + MOTOR4_OFFSET;
    }
    motor_limit();

    /*DEBUGPRINT("M1:");DEBUGPRINT(motor.motor1/100.0);DEBUGPRINT("\t");
    DEBUGPRINT("M2:");DEBUGPRINT(motor.motor2/100.0);DEBUGPRINT("\t");
    DEBUGPRINT("M3:");DEBUGPRINT(motor.motor3/100.0);DEBUGPRINT("\t");
    DEBUGPRINT("M4:");DEBUGPRINT(motor.motor4/100.0);DEBUGPRINT("\t");
    DEBUGPRINT(roll/50.0);DEBUGPRINT("\t");
    DEBUGPRINT(pitch/50.0);DEBUGPRINT("\t");
    DEBUGPRINTLN("");*/
}

void motor_autolevel_loop(
        int16_t throttle, int16_t roll, int16_t pitch, int16_t yaw, 
        bool vertical_velocity)
{
    // 3183.09886184

    int16_t scaled_roll = (int16_t) (attitude.roll * 1591.54943092);
    int16_t scaled_pitch = (int16_t) (attitude.pitch * 1591.54943092);
    //int16_t scaled_yaw = (int16_t) (attitude.yaw * (fixed)1591.54943092);
    int16_t scaled_yawspeed = (int16_t) (attitude.yawspeed * 1591.54943092);

    //int16_t scaled_rollspeed = (int16_t) (attitude.rollspeed * (fixed)1591.54943092 * (fixed)KP_GAIN_ROLLSPEED);
    //int16_t scaled_pitchspeed = (int16_t) (attitude.pitchspeed * (fixed)1591.54943092 * (fixed)KP_GAIN_PITCHSPEED);
    //int16_t scaled_yawspeed = (int16_t) (attitude.yawspeed * (fixed)1591.54943092 * (fixed)KP_GAIN_YAWSPEED);


    if (scaled_roll > 5000) { scaled_roll = 5000; }
    if (scaled_roll < -5000) { scaled_roll = -5000; }
    if (scaled_pitch > 5000) { scaled_pitch = 5000; }
    if (scaled_pitch < -5000) { scaled_pitch = -5000; }

    /*DEBUGPRINT("s_roll:");DEBUGPRINT(scaled_roll);DEBUGPRINT("\t");
    DEBUGPRINT("jroll:");DEBUGPRINT(roll);DEBUGPRINT("\t");
    DEBUGPRINT("s_pitch:");DEBUGPRINT(scaled_pitch);DEBUGPRINT("\t");
    DEBUGPRINT("jpitch:");DEBUGPRINT(pitch);DEBUGPRINT("\t");*/
    //DEBUGPRINT("s_yaw:");DEBUGPRINT(scaled_yaw);
    //DEBUGPRINTLN("");
    
    /*
    if ((millis() - PID_last_time) > (1000/PID_HZ)) {
        pid_roll = PID_Roll.step(roll, scaled_roll);
        pid_pitch = PID_Pitch.step(pitch, scaled_pitch);
        PID_last_time = millis();
    }
    */
    if ((attitude.sampled & SAMPLED_ATTITUDE) == SAMPLED_ATTITUDE) {
        pid_pitch = PID_Pitch.step(pitch, scaled_pitch, attitude.deltat);
        pid_roll = PID_Roll.step(roll, scaled_roll, attitude.deltat);
        pid_yaw = PID_Yaw.step(yaw, scaled_yawspeed, attitude.deltat);
        #ifdef ESC_NO
        /*DEBUGPRINT("PID Roll: ");DEBUGPRINT(pid_roll/50.0);DEBUGPRINT("\t");
        DEBUGPRINT(roll/50.0);DEBUGPRINT("\t");DEBUGPRINT(scaled_roll/50.0);DEBUGPRINT("\t");
        DEBUGPRINT("PID Pitch: ");DEBUGPRINT(pid_pitch/50.0);DEBUGPRINT("\t");
        DEBUGPRINT(pitch/50.0);DEBUGPRINT("\t");DEBUGPRINT(scaled_pitch/50.0);DEBUGPRINT("\t");*/
        DEBUGPRINT("PID Yaw: ");DEBUGPRINT(pid_yaw);DEBUGPRINT("\t");
        DEBUGPRINT(yaw);DEBUGPRINT("\t");DEBUGPRINT(scaled_yawspeed);DEBUGPRINT("\t");
        DEBUGPRINTLN("");
        #endif
    }

#if !defined(GYRO_NO) && !defined(BAROMETER_NO)
    uint16_t delta = millis() - vvelocity_last_millis;
    if (vertical_velocity) {
        // FIXME: Esto es sólo por prueba y error, podría hacerse mejor
        uint16_t velocity = throttle/VVELOCITY_THROTTLE_DIVISOR;
        if (altitude.velocity > velocity) {
            vvelocity_throttle = vvelocity_throttle - delta*(float)altitude.velocity;
        }
        if (altitude.velocity < velocity) {
            vvelocity_throttle = vvelocity_throttle + delta*(float)altitude.velocity;
        }
    }
    else
    {
        vvelocity_throttle = throttle;
    }
    vvelocity_last_millis = millis();
#endif

    motor_direct_loop(
#if !defined(GYRO_NO) && !defined(BAROMETER_NO)
        vvelocity_throttle,
#else
        throttle,
#endif
        pid_roll,
        pid_pitch,
        yaw);
}




void motor_arm() {
    esc_arm();
    motor.armed = true;
    DEBUGPRINTLN("Motors ARMED");
}

void motor_disarm() {
    motor_zero();
    esc_loop();
    esc_disarm();
    motor.armed = false;
    DEBUGPRINTLN("Motors DISARMED");
}


// No deja que se superen los límites
void motor_limit() {
    if (motor.motor1 < 0) { motor.motor1 = 0; }
    if (motor.motor2 < 0) { motor.motor2 = 0; }
    if (motor.motor3 < 0) { motor.motor3 = 0; }
    if (motor.motor4 < 0) { motor.motor4 = 0; }

    if (motor.motor1 > MOTOR_MAX_THROTTLE) { motor.motor1 = MOTOR_MAX_THROTTLE; }
    if (motor.motor2 > MOTOR_MAX_THROTTLE) { motor.motor2 = MOTOR_MAX_THROTTLE; }
    if (motor.motor3 > MOTOR_MAX_THROTTLE) { motor.motor3 = MOTOR_MAX_THROTTLE; }
    if (motor.motor4 > MOTOR_MAX_THROTTLE) { motor.motor4 = MOTOR_MAX_THROTTLE; }
}