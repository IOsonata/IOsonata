/**-------------------------------------------------------------------------
@file	SlimeVRDongle.h

@brief	Shared ESB and USB declarations for the SlimeVR receiver dongle.

@author	Hoang Nguyen Hoan
@date	Nov. 25, 2024

@license

MIT License

Copyright (c) 2024, I-SYST inc., all rights reserved

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

----------------------------------------------------------------------------*/
#ifndef __SLIMEVRDONGLE_H__
#define __SLIMEVRDONGLE_H__

#include <stdint.h>

#define MAX_TRACKERS 			50
#define DETECTION_THRESHOLD 	16

#ifdef __cplusplus
extern "C" {
#endif

void UsbInit();
void init_cli();
uint32_t esb_init(void);
void EsbSetAddr(bool PairMode);
uint16_t AddTracker(uint64_t Addr);
void UpdateRecord();

// Forward one 16 byte tracker record to the host HID IN endpoint. rssi is
// stored in byte 15 for packet types other than 1 and 4.
void hid_write_packet_n(uint8_t *data, int rssi);

#ifdef __cplusplus
}
#endif

#endif // __SLIMEVRDONGLE_H__
