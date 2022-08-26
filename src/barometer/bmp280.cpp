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

#ifdef BAROMETER_BMP280
#include <BMx280TwoWire.h>
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

#ifndef BMP280_ADDRESS
    #define BMP280_ADDRESS 0x76
#endif


BMx280TwoWire bmx280(&BAROMETER_WIRE, BMP280_ADDRESS);
bool bmx280_started = false;

void barometer_setup() {
    indicator_do();
    barometer_data.pressure = 0;
    barometer_data.pressure_reference = 0;
    barometer_data.temperature = 0;
    barometer_data.humidity = 0;
    if (!bmx280.begin()) {
        indicator_error();
        return;
    }
    bmx280_started = true;

    bmx280.resetToDefaults();
	bmx280.writeOversamplingPressure(BMx280MI::OSRS_P_x16);
	bmx280.writeOversamplingTemperature(BMx280MI::OSRS_T_x02);
    bmx280.writeFilterSetting(BMx280MI::FILTER_x16);

	if (bmx280.isBME280())
		bmx280.writeOversamplingHumidity(BMx280MI::OSRS_H_x16);
    
    bmx280.measure();
    indicator_done();
}

bool bmx280_measure_started = false;

//float accumulated_pressure_reference = 0;
uint8_t pressure_reference_count = 1;

int32_t calculated_altitude = 0; // in mm
//uint8_t calculated_altitude_index = 0;
//#define BMP280_ALTITUDE_AVERAGE    16
//int32_t calculated_altitude_values[BMP280_ALTITUDE_AVERAGE];
void barometer_loop() {
    if (!bmx280_started) {
        barometer_setup();
        return;
    }
    if (bmx280.hasValue()) {
        //DEBUGPRINT(F("Pressure: "));DEBUGPRINTLN(bmx280.getPressure());
        barometer_data.pressure = bmx280.getPressure64();
        barometer_data.temperature = bmx280.getTemperature();
        if (bmx280.isBME280()) {
            barometer_data.humidity = bmx280.getHumidity();
        }
        
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
        bmx280_measure_started = false;
    }

    // Smooth (filter) the altitude value
    int32_t dif = calculated_altitude - altitude.barometer;
    altitude.barometer += dif / 100 + 0.5;
    
    /*calculated_altitude_values[calculated_altitude_index] = calculated_altitude;
    calculated_altitude_index = (calculated_altitude_index + 1) % BMP280_ALTITUDE_AVERAGE;
    int32_t values = 0;
    for (uint8_t i = 0; i<BMP280_ALTITUDE_AVERAGE; i++) {
        values += calculated_altitude_values[i];
    }
    altitude.barometer = values / BMP280_ALTITUDE_AVERAGE;*/

    DEBUGPRINT(F("Pressure: "));DEBUGPRINT((float)barometer_data.pressure);DEBUGPRINT(" \t ");
    DEBUGPRINT((float)barometer_data.pressure_reference);DEBUGPRINT(" \t ");
    DEBUGPRINT((float)barometer_data.temperature);DEBUGPRINT("ºC \t ");
    //DEBUGPRINT(static_cast<float>(barometer_data.pressure_reference / barometer_data.pressure));DEBUGPRINT(" \t ");
    DEBUGPRINT(F("Altitude: "));DEBUGPRINT(altitude.barometer/10.0);DEBUGPRINTLN(" cm");

    if (!bmx280_measure_started) {
        bmx280_measure_started = bmx280.measure();
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