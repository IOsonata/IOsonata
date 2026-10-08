/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 I-SYST inc. All rights reserved.
 */
#ifndef __SPI_SOFT_H__
#define __SPI_SOFT_H__

#include "coredev/spi.h"

/** GPIO SPI master, synchronous only. Uses the usual SPICfg_t pin order.
 * All present pins must select GPIO. Normal SPI permits absent MISO or MOSI
 * (-1/-1) for one-way devices. In 3-wire mode MOSI is the bidirectional data
 * pin and MISO must be absent. Supports 4..16 bits, modes 0..3, MSB/LSB first.
 * Words wider than eight bits occupy two little-endian bytes in buffers.
 * DMA/interrupt requests are rejected. EvtCB is not called for polling I/O.
 * Rate is the nominal delay-based clock ceiling (maximum 500 kHz); GPIO,
 * software overhead and interrupts make the actual clock slower/variable.
 * Pin maps and device storage must outlive the interface. No heap or global
 * device slots. Initialize in main, and serialize lifecycle/config changes
 * with transfers. Reinitialize only after the previous session is stopped.
 */
typedef struct __SPI_Soft_Device {
	SPIDev_t Spi;
	uint32_t HalfPeriodUs;
	bool Enabled;
} SPISoftDev_t;

#ifdef __cplusplus
extern "C" {
#endif
bool SPISoftInit(SPISoftDev_t * const pDev, const SPICfg_t *pCfg);
/** Synchronous full duplex, automatic Start/Stop. NULL TX sends DummyByte;
 * NULL RX discards input. Both NULL and 3-wire full duplex are rejected.
 * Exact in-place TX/RX is supported; other overlapping buffers are not.
 * Returns byte count, or zero on invalid/busy/disabled requests.
 */
int SPISoftTransfer(SPISoftDev_t * const pDev, uint32_t DevCs,
					const uint8_t *pTx, uint8_t *pRx, int Length);
#ifdef __cplusplus
}

class SPISoft : public DeviceIntrf {
public:
	SPISoft() : vDevData{} {}
	SPISoft(const SPISoft&) = delete;
	SPISoft& operator=(const SPISoft&) = delete;
	bool Init(const SPICfg_t &Cfg) { return SPISoftInit(&vDevData, &Cfg); }
	operator DevIntrf_t * () { return &vDevData.Spi.DevIntrf; }
	operator SPIDev_t * () { return &vDevData.Spi; }
	operator SPISoftDev_t * () { return &vDevData; }
	uint32_t Rate(uint32_t RateHz) { return SPISetRate(&vDevData.Spi, RateHz); }
	uint32_t Rate(void) { return SPIGetRate(&vDevData.Spi); }
	int Transfer(uint32_t DevCs, const uint8_t *pTx, uint8_t *pRx, int Length) {
		return SPISoftTransfer(&vDevData, DevCs, pTx, pRx, Length);
	}
private:
	SPISoftDev_t vDevData;
};
#endif
#endif
