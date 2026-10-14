/**-------------------------------------------------------------------------
@file	usb_hw_lpc17xx.h

@brief	Private LPC17xx USB clocks, SIE, endpoint realization and CPU FIFOs.

@author	Hoang Nguyen Hoan
@date	Oct. 12, 2026

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
#ifndef USB_HW_LPC17XX_H
#define USB_HW_LPC17XX_H
#include <stdbool.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
// MainHz is the board's external oscillator frequency. The caller must keep
// that oscillator, the selected PLL and core clock stable until release.
// Requires exclusive ownership: an already-powered USB block is refused.
bool Lpc17UsbHwPrepare(uint32_t MainHz);
// Refuses release while endpoint, interrupt or DMA state remains enabled.
bool Lpc17UsbHwRelease(void);
// Private SIE operations for commands with zero or one data byte. Read output
// is unchanged on failure. Entire command/data sequences preserve PRIMASK.
// A timeout blocks further commands until release and successful preparation;
// the caller must treat controller state as unknown and reinitialize it.
// The USB ISR must leave CCEMPTY/CDFULL to these polling operations.
bool Lpc17UsbSieCommand(uint8_t Command);
bool Lpc17UsbSieWrite(uint8_t Command, uint8_t Data);
bool Lpc17UsbSieRead(uint8_t Command, uint8_t *pData);
// Realize 64-byte control OUT/IN before connection and interrupt enable.
bool Lpc17UsbEp0Realize(void);
// Realize one bulk (Type=2) or interrupt (Type=3) direction while traffic is
// quiescent. EpNo is a logical number, not an endpoint address. Requires EP0
// ready, all interrupt/DMA enables off and only bulk/interrupt data endpoints
// already realized. Refuses duplicate realization and unsupported sizes/types.
// Caller keeps the device disconnected or unconfigured throughout the batch.
// Does not enable the endpoint, clear its status, configure the device or arm
// transfers. A realization timeout latches the fault until release/prepare.
bool Lpc17UsbDataRealize(uint8_t EpNo, bool In, uint8_t Type, uint16_t Size);
// CPU-mode single-packet access to realized bulk/interrupt directions. Uses
// each direction's hardware MPS; never accesses EP0 or ISO. Caller owns IRQ
// acknowledgement and endpoint enable/status. Both preserve PRIMASK and leave
// device/endpoint events pending. No operation starts during a bus event.
// Read returns packet length, -2 if empty, or -1 on failure. Output remains
// unchanged on failure; insufficient capacity leaves the packet queued.
// Write returns true after validation, not host acknowledgement. Full/stalled
// endpoints return false without writing FIFO data. NULL is allowed for ZLP.
// Hardware timeouts/invalid RX data latch the fault until release/prepare.
int Lpc17UsbDataRead(uint8_t EpNo, uint8_t *pData, int Capacity);
bool Lpc17UsbDataWrite(uint8_t EpNo, const uint8_t *pData, int Length);
// Capture one enabled/pending data-endpoint interrupt snapshot. Mask uses
// physical endpoint bits; Status[0..31] uses physical indices (absent = zero).
// Each interrupt is cleared separately and its SIE status consumed before the
// next clear. New arrivals outside the snapshot stay pending. EP0 and device
// summary flags are untouched. Requires realized bulk/interrupt CPU endpoints.
// False leaves outputs unchanged, even after partial acknowledgement: do not
// replay the snapshot. The owner must handle a pending bus event or stop on
// fault. A timeout latches the SIE fault. No callbacks run in this helper.
bool Lpc17UsbDataCollect(uint32_t *pMask, uint8_t Status[32]);
// Set enabled (0), halted (1), or disabled (0x20) status on a realized data
// endpoint, flush/reset its FIFO and acknowledge its stale interrupt. The
// caller masks this endpoint interrupt first. No RAM realization is changed.
// Initializing permits pending bus events only for disabling freshly realized
// endpoints with all device/endpoint IRQ enables off, before data traffic.
bool Lpc17UsbDataSetStatus(uint8_t EpNo, bool In, uint8_t Status, bool Initializing);
// Initialize an exclusively owned, prepared block while disconnected.
// Requires no prior EP0 realization, enabled interrupts, DMA or data endpoints.
// Leaves address zero, 64-byte EP0, NAK interrupts off and all IRQ enables off.
// The caller supplies pin setup, NVIC configuration and later connection.
bool Lpc17UsbDeviceInit(void);
// Read one pending DEV_STAT event after clearing its device interrupt flag.
// Returns 1 for status, 0 if none, -1 on failure. Output changes only on 1.
// Raw status contains CON/CON_CH/SUS/SUS_CH/RST; caller handles reset first.
int Lpc17UsbDeviceStatus(uint8_t *pStatus);
// Only after Get Device Status reports RST: retire stale endpoint interrupts
// and restore 64-byte EP0. Requires DMA off, hardware data realization already
// reset, and the caller's data endpoint interrupt enables already masked.
// Failure leaves device/endpoint interrupts disabled and latches the fault.
bool Lpc17UsbBusReset(void);
// Clear one physical EP0 interrupt and return raw SIE endpoint status.
// In=false selects OUT; In=true selects IN. Does not require an enabled or
// pending interrupt, so OUT can also clear STP/PO during overwrite recovery.
// Output is unchanged on failure; a timeout latches the SIE fault above.
bool Lpc17UsbEp0Ack(bool In, uint8_t *pStatus);
// Capture enabled, pending EP0 interrupts once, before dispatching callbacks.
// Returns physical endpoint mask (OUT=1, IN=2), or -1 on failure. Status[0/1]
// receives OUT/IN status; absent directions are zero. Output stays unchanged
// on failure, even if one endpoint was already acknowledged. Do not replay
// after failure. Caller owns device summary flags and bus-reset handling.
int Lpc17UsbEp0Collect(uint8_t Status[2]);
// Caller services/acknowledges endpoint interrupts separately. Read returns
// packet length, -1 on failure (output unchanged), or -2 for SETUP overwrite.
// On -2, clear PO with Select Endpoint/Clear Interrupt and read the new SETUP.
// Insufficient capacity leaves the packet queued. Zero length accepts NULL.
int Lpc17UsbEp0Read(uint8_t *pData, int Capacity);
// True means buffer validation completed, not host acknowledgement. A new
// SETUP can invalidate an IN packet; the future IRQ layer handles that abort.
bool Lpc17UsbEp0Write(const uint8_t *pData, int Length);
#ifdef __cplusplus
}
#endif
#endif
