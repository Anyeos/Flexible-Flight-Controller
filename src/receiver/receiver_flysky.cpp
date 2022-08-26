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

El Joystick (o control) del ToySky utiliza un XN297.
Para usarlo con un NRF24L01 sólo basta agregarle un NRF24L01 al Joystick también.
Hay que hacer esa modificación o comprar un XN297 para nuestro Dron porque
no encontré manera de recibir correctamente el XN297 con un NRF24L01.

La dirección es CB0430A25A

*/


#include "../../Global.h"
#ifdef RECEIVER_FLYSKY

#ifndef FLYSKY_RCV_ADDR
    #define FLYSKY_RCV_ADDR 0xCB0430A25ALL
#endif

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "../motor/esc.h"

#define FLYSKY_RCV_CE_PIN  22
#define FLYSKY_RCV_CSN_PIN 17
RF24 radio_flysky(FLYSKY_RCV_CE_PIN, FLYSKY_RCV_CSN_PIN);

uint8_t flysky_channel[8];

#include "receiver.h"
bool radio_flysky_started = false;

#ifndef RECEIVER_FLYSKY_CHANNEL
bool radio_flysky_linked = false;
unsigned long radio_flysky_last_received = 0;
uint8_t radio_flysky_step = 0;
#endif
void receiver_setup() {
    indicator_do();
    receiver_rssi = UINT8_MAX;
    if (!radio_flysky.begin()) {
        radio_flysky_started = false;
        indicator_error();
    }

    radio_flysky.setPALevel(RF24_PA_LOW);

    radio_flysky.setAutoAck(false);

    //radio_flysky.enableDynamicPayloads();
    radio_flysky.disableDynamicPayloads();
    radio_flysky.setPayloadSize(8);
    
    radio_flysky.setAddressWidth(5);
    
    radio_flysky.setCRCLength(RF24_CRC_16);
    radio_flysky.disableCRC();

    radio_flysky.startListening();
    radio_flysky.stopListening();

    #ifdef RECEIVER_FLYSKY_CHANNEL
    radio_flysky.setChannel(RECEIVER_FLYSKY_CHANNEL);
    radio_flysky.openReadingPipe(0, FLYSKY_RCV_ADDR);
    radio_flysky.startListening();
    #else
    radio_flysky_step = 0;
    #endif

    radio_flysky_started = true;
    indicator_done();
}

#ifndef RECEIVER_FLYSKY_CHANNEL
int radio_flysky_schannel = 128;
unsigned long radio_flysky_receiving_time = 0;
#endif
void receiver_loop() {
    if (!radio_flysky_started) {
        receiver_setup();
        return;
    }

    #ifndef RECEIVER_FLYSKY_CHANNEL
    if (!radio_flysky_linked) {
        switch (radio_flysky_step)
        {
        case 0:
            radio_flysky_schannel = radio_flysky_schannel % 128 + 1;
            
            radio_flysky.setChannel(radio_flysky_schannel);
            radio_flysky.startListening();
            delayMicroseconds(128);
            radio_flysky.stopListening();

            if (radio_flysky.testCarrier()) {
                radio_flysky.setChannel(radio_flysky_schannel);
                radio_flysky_receiving_time = millis();
                radio_flysky_step = 1;
                /*uint8_t buf = FLYSKY_RCV_ADDR;
                radio_flysky.openWritingPipe(FLYSKY_RCV_ADDR);
                radio_flysky.write(&buf, 5);*/
                radio_flysky.openReadingPipe(0, FLYSKY_RCV_ADDR);
                radio_flysky.startListening();
            }
            break;
        case 1:
            if (millis() - radio_flysky_receiving_time < 250) {
                if (radio_flysky.available()) {
                    DEBUGPRINT("FlySky receiver: On channel ");DEBUGPRINT(radio_flysky_schannel);
                    DEBUGPRINTLN("");
                    radio_flysky_linked = true;
                    radio_flysky_step = 0;
                    indicator_start(0b10101010, 50);
                    break;
                }
            } else {
                radio_flysky.stopListening();
                radio_flysky.closeReadingPipe(0);
                radio_flysky_step = 0;
            }
            break;
        }
    } else {
    #endif
        uint8_t pipe;
        if (radio_flysky.available(&pipe)) {               // is there a payload? get the pipe number that recieved it
            uint8_t bytes = radio_flysky.getPayloadSize(); // get the size of the payload
            radio_flysky.read(&flysky_channel, bytes);     // fetch payload from FIFO
            
            byte estado = flysky_channel[0];
            bool thumb_L  = estado & 0b10000000;
            bool retHome  = estado & 0b100000;
            bool thumb_R  = estado & 0b10000;
            bool holdA    = estado & 0b1;

            if (
                (estado & 0b1000000) ||
                (estado & 0b1000) || 
                (estado & 0b100) || 
                (estado & 0b10) ||
                (flysky_channel[1] > 178) ||
                (flysky_channel[2] > 178) ||
                (flysky_channel[3] > 178) ||
                (flysky_channel[4] > 178))
            {
                //DEBUGPRINT("FlySky receiver: Invalid data received!");
                return;
            }
            
            int throttle = flysky_channel[1];
            if (flysky_channel[1] & 128) {
                throttle = 128 - flysky_channel[1];
            }
            channel_throttle_centered = throttle*10;
            channel_scaled[CHANNEL_THROTTLE] = (throttle + 50)*100;

            int pitch = flysky_channel[2];
            if (flysky_channel[2] & 128) {
                pitch = 128 - flysky_channel[2];
            }
            channel_scaled[CHANNEL_PITCH] = -pitch*100;

            int roll = flysky_channel[3];
            if (flysky_channel[3] & 128) {
                roll = 128 - flysky_channel[3];
            }
            channel_scaled[CHANNEL_ROLL] = roll*100;

            int yaw = flysky_channel[4];
            if (flysky_channel[4] & 128) {
                yaw = 128 - flysky_channel[4];
            }
            channel_scaled[CHANNEL_YAW] = yaw*100;

            if (thumb_L and thumb_R) {
                
            }

            if (thumb_R) {
            }

            if (holdA) {
            }

            if (retHome) {
            }
            

            if (flysky_channel[5] == 51) {
                channel_scaled[CHANNEL_FMODE] = flysky_channel[6]*5000 - 5000;
            }

            /*DEBUGPRINT(channel_scaled[CHANNEL_THROTTLE]);DEBUGPRINT("\t");
            DEBUGPRINT(channel_scaled[CHANNEL_ROLL]);DEBUGPRINT("\t");
            DEBUGPRINT(channel_scaled[CHANNEL_PITCH]);DEBUGPRINT("\t");
            DEBUGPRINT(channel_scaled[CHANNEL_YAW]);
            DEBUGPRINTLN("");*/
        #ifndef RECEIVER_FLYSKY_CHANNEL
            radio_flysky_last_received = millis();
        } else 
        if (millis() - radio_flysky_last_received > 5000) { // No receiving
            DEBUGPRINT("FlySky receiver: Timed out");
            radio_flysky_linked = false;
            radio_flysky_step = 0;
            radio_flysky.stopListening();
            radio_flysky.closeReadingPipe(0);
        }
        #endif
    }
}

#endif