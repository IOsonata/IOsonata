/**-------------------------------------------------------------------------
@file	i2s_lpc546xx.h

@brief	LPC546xx I2S on Flexcomm 6 and 7

I2SInit (coredev/i2s.h) on the I2S Flexcomms of the LPC546xx. A Flexcomm
sends or receives, not both. I2SCfg_t::DevNo is the Flexcomm, 6 or 7, and
the data pins given choose the directions:

- SDO only: DevNo sends.
- SDI only: DevNo receives.
- SDI and SDO: DevNo sends, the other I2S Flexcomm receives on the same SCK
and WS. Its SCK and WS pins are entries LPC546XX_I2S_RXSCK_IOPIN_IDX and
LPC546XX_I2S_RXWS_IOPIN_IDX of the pin map, wired to SCK and WS on the board
(LPCXpresso54628: Flexcomm 6 to the codec, Flexcomm 7 from it).

In master mode DevNo makes SCK and WS, the receiving Flexcomm of a full
duplex is a slave on them. In slave mode both take SCK and WS from outside.

Frames: classic I2S (I2SFMT_NORMAL), 16 bit samples, a 32 bit frame (16 SCK
per channel). Stereo frames are 4 bytes, left first. A mono frame
(I2SCHAN_LEFT or I2SCHAN_RIGHT) is 2 bytes: the other channel sends 0 and
its received data is dropped. Left and right justified formats, 8 and 24
bit samples are not supported by this port.

Clocks: the audio PLL (clock_lpc546xx.h) runs at 256 x Freq x K, K the
smallest from 1 making it 12 MHz or more, and clocks the Flexcomms. SCK is
the PLL / (8 x K) in master mode. With the MCK pin (I2S_MCK_IOPIN_IDX), the
MCLK pin sends the PLL / K, 256 x Freq, as I2SDev_t::MClkFreq gives in master
mode. The rate returned is the PLL output / (256 x K), the exact rate at 48
kHz, 44.1 kHz and their usual divisions. One I2S device at a time: it owns
the audio PLL.

Modes: without FIFO memory for a direction, RxData and TxData move the frames
through the hardware FIFOs and wait for them (polling). With FIFO memory and
bIntEn, the Flexcomm interrupts move the frames between the hardware FIFOs
and the CFIFOs, with bDmaEn the DMA moves blocks of 32 frames (the receive
FIFO memory then holds 32 frames or more). Without
bIntEn the transfers move the frames when RxData and TxData are called. The
transmit side sends silence when it has no frame, and a hardware FIFO that
runs empty sends 0. A receive FIFO overrun in interrupt or DMA mode, a full
receive CFIFO, or a DMA error stops the stream (DEVINTRF_EVT_STATECHG or
DEVINTRF_EVT_RX_FIFO_FULL) until Reset or Disable then Enable.

@author	Hoang Nguyen Hoan
@date	Oct. 27, 2026

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
#ifndef __I2S_LPC546XX_H__
#define __I2S_LPC546XX_H__

#include "coredev/i2s.h"

/// SCK and WS of the receiving Flexcomm of a full duplex
#define LPC546XX_I2S_RXSCK_IOPIN_IDX	5
#define LPC546XX_I2S_RXWS_IOPIN_IDX		6
#define LPC546XX_I2S_MAX_NB_IOPIN		7

#endif // __I2S_LPC546XX_H__
