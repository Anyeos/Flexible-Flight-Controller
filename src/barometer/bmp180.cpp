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

#ifdef BAROMETER_BMP180
#include <BMP180TwoWire.h>
#include "barometer.h"
#include "../altitude/altitude.h"

//#include "airdata.h"
/*
#include <Adafruit_BMP280.h>

#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

Adafruit_BMP280 bmp; // I2C
*/

#ifndef BMP180_ADDRESS
    #define BMP180_ADDRESS 0x77
#endif


BMP180TwoWire bmp180(&BAROMETER_WIRE, BMP180_ADDRESS);
bool bmp180_started = false;

void barometer_setup() {
    indicator_do();
    barometer_data.pressure = 0;
    barometer_data.pressure_reference = 0;
    barometer_data.temperature = 0;
    barometer_data.humidity = 0;
    if (!bmp180.begin()) {
        indicator_error();
        return;
    }
    bmp180_started = true;

    bmp180.resetToDefaults();
	bmp180.setSamplingMode(BMP180MI::MODE_UHR);

    indicator_done();
}

uint8_t bmp180_measure_step = 0;

//float accumulated_pressure_reference = 0;
uint8_t pressure_reference_count = 1;

int32_t calculated_altitude = 0; // in mm
//uint8_t calculated_altitude_index = 0;
//#define BMP180_ALTITUDE_AVERAGE    32
//int32_t calculated_altitude_values[BMP180_ALTITUDE_AVERAGE];
void barometer_loop() {
    if (!bmp180_started) {
        barometer_setup();
        return;
    }
    if (bmp180_measure_step == 1 && bmp180.hasValue()) {
        barometer_data.temperature = bmp180.getTemperature();
        bmp180_measure_step++;
    }

    if (bmp180_measure_step == 3 && bmp180.hasValue()) {
        barometer_data.pressure = bmp180.getPressure();
        
        if (pressure_reference_count <= 100) {
            fixed dif = barometer_data.pressure - barometer_data.pressure_reference;
            barometer_data.pressure_reference += dif / 5 + 0.5;
            pressure_reference_count++;
        } else {
            // Calculates altitude based on reference
            //calculated_altitude = (pow((double)(barometer_data.pressure / barometer_data.pressure_reference), 0.190223)*100 - 100) * 44330;
            calculated_altitude = 
                (int32_t) (
                (pow( ((float)barometer_data.pressure_reference / (float)barometer_data.pressure), 0.190223 ) - 1.0) *
                ((float)barometer_data.temperature + 273.15) * 153846.15);
        }
        bmp180_measure_step = 0;
    }

    // Smooth (filter) the altitude value
    int32_t dif = calculated_altitude - altitude.barometer;
    altitude.barometer += dif / 200 + 0.5;
    
    /*calculated_altitude_values[calculated_altitude_index] = calculated_altitude;
    calculated_altitude_index = (calculated_altitude_index + 1) % BMP180_ALTITUDE_AVERAGE;
    int32_t values = 0;
    for (uint8_t i = 0; i<BMP180_ALTITUDE_AVERAGE; i++) {
        values += calculated_altitude_values[i];
    }
    altitude.barometer = values / BMP180_ALTITUDE_AVERAGE;*/

    DEBUGPRINT(F("Pressure: "));DEBUGPRINT((float)barometer_data.pressure);DEBUGPRINT(" \t ");
    DEBUGPRINT((float)barometer_data.pressure_reference);DEBUGPRINT(" \t ");
    DEBUGPRINT((float)barometer_data.temperature);DEBUGPRINT("ºC \t ");
    //DEBUGPRINT(static_cast<float>(barometer_data.pressure_reference / barometer_data.pressure));DEBUGPRINT(" \t ");
    DEBUGPRINT(F("Altitude: "));DEBUGPRINT(altitude.barometer/10.0);DEBUGPRINTLN(" cm");

    if (bmp180_measure_step == 0) {
        bmp180.measureTemperature();
        bmp180_measure_step++;
    }
    else
    if (bmp180_measure_step == 2) {
        bmp180.measurePressure();
        bmp180_measure_step++;
    }
}
bool barometer_calibrate() {
    indicator_do();
    /*int tries = 0;
    while (barometer_data.pressure == 0.0) {
        barometer_loop();
        delay(100);
        tries++;
        if (tries > 10) {
            indicator_error();
            return false;
        }
    }*/
    barometer_data.pressure_reference = barometer_data.pressure;
    indicator_done();
    return true;
}

#endif