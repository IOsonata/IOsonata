/**-------------------------------------------------------------------------
@file usb_mem_lpc_fs.h
@brief Shared LPC full-speed USB endpoint-list storage.

@author Hoang Nguyen Hoan
@date Oct. 13, 2026
@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

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
#ifndef __USB_MEM_LPC_FS_H__
#define __USB_MEM_LPC_FS_H__

#include <stdint.h>

// Ten physical endpoints, two command words per endpoint. EP0 OUT word 1
// holds the SETUP buffer address. Buffers start on 64-byte boundaries.
// This layout implements 64-byte control/bulk/interrupt packets, not ISO.
typedef struct __Lpc_Usb_Fs_Memory {
	volatile uint32_t Command[20];
	uint8_t Reserved[48];
	uint8_t Setup[64];
	uint8_t Packet[10][64];
} LpcUsbFsMemory_t;

#endif
