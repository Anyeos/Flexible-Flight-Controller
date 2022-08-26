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
#include "receiver.h"

#ifdef RECEIVER_PPM
#include "../third_party/PPMreader/PPMReader.h"
#include "receiver_ppm.h"

PPMReader ppm(RECEIVER_PPM_PIN, MAX_CHANNELS);

void receiver_setup() {
  indicator_do();
  receiver_rssi = 100;
  indicator_done();
}

void receiver_loop() {
    for (uint8_t ch = 1; ch <= MAX_CHANNELS; ++ch) {
        uint16_t value = ppm.latestValidChannelValue(ch, 0);
        if (value != 0) {
          // We smooth the values to mitigate reception fails
          int16_t dif = value - channel[ch];
          channel[ch] += dif / 7 + 0.5;
          //if (value > channel[ch]) { channel[ch]++; }
          //if (value < channel[ch]) { channel[ch]--; }
          if (ch == CHANNEL_THROTTLE) {
            channel_scaled[ch] = scale_channel(channel[ch], 130, 0, false);
            channel_throttle_centered = scale_channel(channel[ch], 130, 30);
          } else 
          if (ch == CHANNEL_GEAR) {
            channel_scaled[ch] = scale_channel(channel[ch], 200, 50);
          } else
          if (ch == CHANNEL_FMODE) {
            channel_scaled[ch] = scale_channel(channel[ch], 200, 50);
          } else {
            channel_scaled[ch] = scale_channel(channel[ch], 130, 30);
          }
        }
        //DEBUGPRINT(ch);DEBUGPRINT(":");
        //DEBUGPRINT(channel_scaled[ch]);DEBUGPRINT("\t");
    }
    //DEBUGPRINTLN("");
    check_for_control();
}

#endif