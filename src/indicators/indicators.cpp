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
#include "indicators.h"

indicator_sequence_t indicator_sequence;

#ifdef LED_PIN
    void indicator_setup() {
        pinMode(LED_PIN, OUTPUT);
        indicator_off();
    }
    void indicator_on() {
        indicator_sequence.on = true;
        digitalWrite(LED_PIN, !LED_PIN_OFF);
    }
    void indicator_off() {
        indicator_sequence.on = false;
        digitalWrite(LED_PIN, LED_PIN_OFF);
    }
    void indicator_delay(uint8_t sequence, uint8_t len, uint8_t ms) {
        indicator_sequence.on = false;
        #if LED_PIN_OFF > 0
            sequence = ~sequence;
        #endif
        uint8_t i;
        for (i = 0; i<len; i++) {
            digitalWrite(LED_PIN, (sequence >> i) & 0b00000001);
            delay(ms);
        }
        digitalWrite(LED_PIN, LED_PIN_OFF);
    }
    void indicator_start(uint8_t sequence, int ms, bool force, bool one_shot) {
        indicator_sequence.on = true;
        #if LED_PIN_OFF > 0
            sequence = ~sequence;
        #endif
        if ((sequence == indicator_sequence.sequence) && (ms == indicator_sequence.ms))
        {
            return;
        }
        indicator_sequence.sequence = sequence;
        indicator_sequence.ms = ms;
        if (force) {
            indicator_sequence.b = 0;
            indicator_sequence.previous_millis = 0;
            indicator_sequence.one_shot = one_shot;
        }
    }
    void indicator_stop() {
        if (indicator_sequence.on) {
            indicator_off();
        }
    }
    void indicator_loop() {
        if (!indicator_sequence.on) { return; }
        if (indicator_sequence.one_shot && indicator_sequence.b >= 7) { return; }
        unsigned long currentMillis = millis();
        if (currentMillis - indicator_sequence.previous_millis > indicator_sequence.ms) {
            indicator_sequence.previous_millis = currentMillis;
            digitalWrite(LED_PIN, (indicator_sequence.sequence >> indicator_sequence.b) & 0b00000001);
            indicator_sequence.b = (indicator_sequence.b+1) % 8;
        }
    }
    void indicator_do() {
        DEBUGPRINTLN("DO:");
        if (state <= STATE_LANDED) {
            indicator_on();
        } else {
            indicator_start(0b11111111, 50);
        }
    }
    void indicator_done() {
        DEBUGPRINTLN(":DONE");
        if (state <= STATE_LANDED) {
            indicator_delay(0b00000010, 3, 50);
        } else {
            indicator_start(0b00000010, 50, true, true);
        }
    }
    void indicator_error() {
        DEBUGPRINTLN(":ERROR");
        if (state <= STATE_LANDED) {
            indicator_delay(0b00101010, 8, 200);
        } else {
            indicator_start(0b00101010, 200);
        }
    }
#else
    void indicator_setup() { }
    void indicator_on() { }
    void indicator_off() { }
    void indicator_delay(uint8_t sequence, uint8_t len, uint8_t ms) { }
    void indicator_loop() { }
#endif

#ifdef NON_BLOCKING_INDICATORS
    void indicator_sequence_start(uint8_t sequence, uint8_t ms) {
        indicator_sequence.is_on = true;
        indicator_sequence.ms = ms;
    }
#else
    void indicator_sequence_start(uint8_t sequence, uint8_t ms) { }
#endif



#ifdef BUZZ_PIN
    void buzz_setup() {
        pinMode(BUZZ_PIN, OUTPUT);
        digitalWrite(BUZZ_PIN, BUZZ_PIN_OFF);
    }
    void buzz_on() {
        
    }
    void buzz_off() {
        
    }
    void buzz_sequence(uint8_t sequence, uint8_t len, uint8_t ms) {

    }
#else
    void buzz_setup() { }
    void buzz_on() { }
    void buzz_off() { }
    void buzz_sequence(uint8_t sequence, uint8_t len, uint8_t ms) { }
#endif