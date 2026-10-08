/**-------------------------------------------------------------------------
@file	interrupt_ra4m1.h

@brief	RA4M1 ICU event-to-IRQ registration following the RE01 interface.

@license

MIT License

Copyright (c) 2026 I-SYST inc. All rights reserved.

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
#ifndef __INTERRUPT_RA4M1_H__
#define __INTERRUPT_RA4M1_H__
#include <stdbool.h>
#include "ra4m1xxx.h"

/* R01UH0887EJ0110 Table 13.4. These are event IDs, not NVIC IRQ numbers.
 * IRQ13 does not exist as a pin source; IEL13 is an ordinary usable CPU slot.
 * DMA/DTC-only entries below are names, not a promise of CPU support.
 */
#define RA4M1_IELS_CNT 32
#define RA4M1_EVTID_DISABLE 0U
#define RA4M1_EVTID_PORT_IRQ0                0x01U
#define RA4M1_EVTID_PORT_IRQ1                0x02U
#define RA4M1_EVTID_PORT_IRQ2                0x03U
#define RA4M1_EVTID_PORT_IRQ3                0x04U
#define RA4M1_EVTID_PORT_IRQ4                0x05U
#define RA4M1_EVTID_PORT_IRQ5                0x06U
#define RA4M1_EVTID_PORT_IRQ6                0x07U
#define RA4M1_EVTID_PORT_IRQ7                0x08U
#define RA4M1_EVTID_PORT_IRQ8                0x09U
#define RA4M1_EVTID_PORT_IRQ9                0x0AU
#define RA4M1_EVTID_PORT_IRQ10               0x0BU
#define RA4M1_EVTID_PORT_IRQ11               0x0CU
#define RA4M1_EVTID_PORT_IRQ12               0x0DU
#define RA4M1_EVTID_PORT_IRQ14               0x0FU
#define RA4M1_EVTID_PORT_IRQ15               0x10U
#define RA4M1_EVTID_DMAC0_INT                0x11U
#define RA4M1_EVTID_DMAC1_INT                0x12U
#define RA4M1_EVTID_DMAC2_INT                0x13U
#define RA4M1_EVTID_DMAC3_INT                0x14U
#define RA4M1_EVTID_DTC_COMPLETE             0x15U
#define RA4M1_EVTID_ICU_SNZCANCEL            0x17U
#define RA4M1_EVTID_FCU_FRDYI                0x18U
#define RA4M1_EVTID_LVD_LVD1                 0x19U
#define RA4M1_EVTID_LVD_LVD2                 0x1AU
#define RA4M1_EVTID_VBATT_LVD                0x1BU
#define RA4M1_EVTID_MOSC_STOP                0x1CU
#define RA4M1_EVTID_SYSTEM_SNZREQ            0x1DU
#define RA4M1_EVTID_AGT0_AGTI                0x1EU
#define RA4M1_EVTID_AGT0_AGTCMAI             0x1FU
#define RA4M1_EVTID_AGT0_AGTCMBI             0x20U
#define RA4M1_EVTID_AGT1_AGTI                0x21U
#define RA4M1_EVTID_AGT1_AGTCMAI             0x22U
#define RA4M1_EVTID_AGT1_AGTCMBI             0x23U
#define RA4M1_EVTID_IWDT_NMIUNDF             0x24U
#define RA4M1_EVTID_WDT_NMIUNDF              0x25U
#define RA4M1_EVTID_RTC_ALM                  0x26U
#define RA4M1_EVTID_RTC_PRD                  0x27U
#define RA4M1_EVTID_RTC_CUP                  0x28U
#define RA4M1_EVTID_ADC140_ADI               0x29U
#define RA4M1_EVTID_ADC140_GBADI             0x2AU
#define RA4M1_EVTID_ADC140_CMPAI             0x2BU
#define RA4M1_EVTID_ADC140_CMPBI             0x2CU
#define RA4M1_EVTID_ADC140_WCMPM             0x2DU
#define RA4M1_EVTID_ADC140_WCMPUM            0x2EU
#define RA4M1_EVTID_ACMP_LP0                 0x2FU
#define RA4M1_EVTID_ACMP_LP1                 0x30U
#define RA4M1_EVTID_USBFS_D0FIFO             0x31U
#define RA4M1_EVTID_USBFS_D1FIFO             0x32U
#define RA4M1_EVTID_USBFS_USBI               0x33U
#define RA4M1_EVTID_USBFS_USBR               0x34U
#define RA4M1_EVTID_IIC0_RXI                 0x35U
#define RA4M1_EVTID_IIC0_TXI                 0x36U
#define RA4M1_EVTID_IIC0_TEI                 0x37U
#define RA4M1_EVTID_IIC0_EEI                 0x38U
#define RA4M1_EVTID_IIC0_WUI                 0x39U
#define RA4M1_EVTID_IIC1_RXI                 0x3AU
#define RA4M1_EVTID_IIC1_TXI                 0x3BU
#define RA4M1_EVTID_IIC1_TEI                 0x3CU
#define RA4M1_EVTID_IIC1_EEI                 0x3DU
#define RA4M1_EVTID_SSIE0_SSITXI             0x3EU
#define RA4M1_EVTID_SSIE0_SSIRXI             0x3FU
#define RA4M1_EVTID_SSIE0_SSIF               0x41U
#define RA4M1_EVTID_CTSU_CTSUWR              0x42U
#define RA4M1_EVTID_CTSU_CTSURD              0x43U
#define RA4M1_EVTID_CTSU_CTSUFN              0x44U
#define RA4M1_EVTID_KEY_INTKR                0x45U
#define RA4M1_EVTID_DOC_DOPCI                0x46U
#define RA4M1_EVTID_CAC_FERRI                0x47U
#define RA4M1_EVTID_CAC_MENDI                0x48U
#define RA4M1_EVTID_CAC_OVFI                 0x49U
#define RA4M1_EVTID_CAN0_ERS                 0x4AU
#define RA4M1_EVTID_CAN0_RXF                 0x4BU
#define RA4M1_EVTID_CAN0_TXF                 0x4CU
#define RA4M1_EVTID_CAN0_RXM                 0x4DU
#define RA4M1_EVTID_CAN0_TXM                 0x4EU
#define RA4M1_EVTID_IOPORT_GROUP1            0x4FU
#define RA4M1_EVTID_IOPORT_GROUP2            0x50U
#define RA4M1_EVTID_IOPORT_GROUP3            0x51U
#define RA4M1_EVTID_IOPORT_GROUP4            0x52U
#define RA4M1_EVTID_ELC_SWEVT0               0x53U
#define RA4M1_EVTID_ELC_SWEVT1               0x54U
#define RA4M1_EVTID_POEG_GROUP0              0x55U
#define RA4M1_EVTID_POEG_GROUP1              0x56U
#define RA4M1_EVTID_GPT0_CCMPA               0x57U
#define RA4M1_EVTID_GPT0_CCMPB               0x58U
#define RA4M1_EVTID_GPT0_CMPC                0x59U
#define RA4M1_EVTID_GPT0_CMPD                0x5AU
#define RA4M1_EVTID_GPT0_CMPE                0x5BU
#define RA4M1_EVTID_GPT0_CMPF                0x5CU
#define RA4M1_EVTID_GPT0_OVF                 0x5DU
#define RA4M1_EVTID_GPT0_UDF                 0x5EU
#define RA4M1_EVTID_GPT1_CCMPA               0x5FU
#define RA4M1_EVTID_GPT1_CCMPB               0x60U
#define RA4M1_EVTID_GPT1_CMPC                0x61U
#define RA4M1_EVTID_GPT1_CMPD                0x62U
#define RA4M1_EVTID_GPT1_CMPE                0x63U
#define RA4M1_EVTID_GPT1_CMPF                0x64U
#define RA4M1_EVTID_GPT1_OVF                 0x65U
#define RA4M1_EVTID_GPT1_UDF                 0x66U
#define RA4M1_EVTID_GPT2_CCMPA               0x67U
#define RA4M1_EVTID_GPT2_CCMPB               0x68U
#define RA4M1_EVTID_GPT2_CMPC                0x69U
#define RA4M1_EVTID_GPT2_CMPD                0x6AU
#define RA4M1_EVTID_GPT2_CMPE                0x6BU
#define RA4M1_EVTID_GPT2_CMPF                0x6CU
#define RA4M1_EVTID_GPT2_OVF                 0x6DU
#define RA4M1_EVTID_GPT2_UDF                 0x6EU
#define RA4M1_EVTID_GPT3_CCMPA               0x6FU
#define RA4M1_EVTID_GPT3_CCMPB               0x70U
#define RA4M1_EVTID_GPT3_CMPC                0x71U
#define RA4M1_EVTID_GPT3_CMPD                0x72U
#define RA4M1_EVTID_GPT3_CMPE                0x73U
#define RA4M1_EVTID_GPT3_CMPF                0x74U
#define RA4M1_EVTID_GPT3_OVF                 0x75U
#define RA4M1_EVTID_GPT3_UDF                 0x76U
#define RA4M1_EVTID_GPT4_CCMPA               0x77U
#define RA4M1_EVTID_GPT4_CCMPB               0x78U
#define RA4M1_EVTID_GPT4_CMPC                0x79U
#define RA4M1_EVTID_GPT4_CMPD                0x7AU
#define RA4M1_EVTID_GPT4_CMPE                0x7BU
#define RA4M1_EVTID_GPT4_CMPF                0x7CU
#define RA4M1_EVTID_GPT4_OVF                 0x7DU
#define RA4M1_EVTID_GPT4_UDF                 0x7EU
#define RA4M1_EVTID_GPT5_CCMPA               0x7FU
#define RA4M1_EVTID_GPT5_CCMPB               0x80U
#define RA4M1_EVTID_GPT5_CMPC                0x81U
#define RA4M1_EVTID_GPT5_CMPD                0x82U
#define RA4M1_EVTID_GPT5_CMPE                0x83U
#define RA4M1_EVTID_GPT5_CMPF                0x84U
#define RA4M1_EVTID_GPT5_OVF                 0x85U
#define RA4M1_EVTID_GPT5_UDF                 0x86U
#define RA4M1_EVTID_GPT6_CCMPA               0x87U
#define RA4M1_EVTID_GPT6_CCMPB               0x88U
#define RA4M1_EVTID_GPT6_CMPC                0x89U
#define RA4M1_EVTID_GPT6_CMPD                0x8AU
#define RA4M1_EVTID_GPT6_CMPE                0x8BU
#define RA4M1_EVTID_GPT6_CMPF                0x8CU
#define RA4M1_EVTID_GPT6_OVF                 0x8DU
#define RA4M1_EVTID_GPT6_UDF                 0x8EU
#define RA4M1_EVTID_GPT7_CCMPA               0x8FU
#define RA4M1_EVTID_GPT7_CCMPB               0x90U
#define RA4M1_EVTID_GPT7_CMPC                0x91U
#define RA4M1_EVTID_GPT7_CMPD                0x92U
#define RA4M1_EVTID_GPT7_CMPE                0x93U
#define RA4M1_EVTID_GPT7_CMPF                0x94U
#define RA4M1_EVTID_GPT7_OVF                 0x95U
#define RA4M1_EVTID_GPT7_UDF                 0x96U
#define RA4M1_EVTID_GPT_UVWEDGE              0x97U
#define RA4M1_EVTID_SCI0_RXI                 0x98U
#define RA4M1_EVTID_SCI0_TXI                 0x99U
#define RA4M1_EVTID_SCI0_TEI                 0x9AU
#define RA4M1_EVTID_SCI0_ERI                 0x9BU
#define RA4M1_EVTID_SCI0_AM                  0x9CU
#define RA4M1_EVTID_SCI0_RXI_OR_ERI          0x9DU
#define RA4M1_EVTID_SCI1_RXI                 0x9EU
#define RA4M1_EVTID_SCI1_TXI                 0x9FU
#define RA4M1_EVTID_SCI1_TEI                 0xA0U
#define RA4M1_EVTID_SCI1_ERI                 0xA1U
#define RA4M1_EVTID_SCI1_AM                  0xA2U
#define RA4M1_EVTID_SCI2_RXI                 0xA3U
#define RA4M1_EVTID_SCI2_TXI                 0xA4U
#define RA4M1_EVTID_SCI2_TEI                 0xA5U
#define RA4M1_EVTID_SCI2_ERI                 0xA6U
#define RA4M1_EVTID_SCI2_AM                  0xA7U
#define RA4M1_EVTID_SCI9_RXI                 0xA8U
#define RA4M1_EVTID_SCI9_TXI                 0xA9U
#define RA4M1_EVTID_SCI9_TEI                 0xAAU
#define RA4M1_EVTID_SCI9_ERI                 0xABU
#define RA4M1_EVTID_SCI9_AM                  0xACU
#define RA4M1_EVTID_SPI0_SPRI                0xADU
#define RA4M1_EVTID_SPI0_SPTI                0xAEU
#define RA4M1_EVTID_SPI0_SPII                0xAFU
#define RA4M1_EVTID_SPI0_SPEI                0xB0U
#define RA4M1_EVTID_SPI0_SPTEND              0xB1U
#define RA4M1_EVTID_SPI1_SPRI                0xB2U
#define RA4M1_EVTID_SPI1_SPTI                0xB3U
#define RA4M1_EVTID_SPI1_SPII                0xB4U
#define RA4M1_EVTID_SPI1_SPEI                0xB5U
#define RA4M1_EVTID_SPI1_SPTEND              0xB6U

typedef void (*Ra4m1IRQHandler_t)(int IntNo, void *pCtx);
#ifdef __cplusplus
extern "C" {
#endif
/* Native counterpart of Re01RegisterIntHandler. CPU interrupts only, no DTC
 * or DMAC routing. Configure/negate the peripheral before registration. For
 * non-pin sources the callback MUST negate/read back the peripheral request
 * and allow its two module-clock synchronization cycles before returning;
 * the dispatcher then clears IR. Edge-mode IRQ-pin requests are acknowledged
 * before callback entry. No normal ISR exit clears NVIC pending state.
 * A callback receives the allocated CPU slot, not the event ID.
 * Callbacks are synchronous ISR calls and must not block. NMI is not supported.
 */
IRQn_Type Ra4m1RegisterIntHandler(uint8_t EvtId, int Prio,
    Ra4m1IRQHandler_t pHandler, void *pCtx);
/* Quiesce the event source first. A released integer IRQ is not a persistent
 * handle; do not retain or use it after release. Raw changes to an allocated
 * IELSR/DTCE are not supported. Custom strong IEL handlers reserve their slot.
 * Unregister does not wait for an already active callback. Keep its code and
 * context alive until that callback returns, including across preemption.
 */
void Ra4m1UnregisterIntHandler(IRQn_Type IrqNo);
/* Negate/read back the peripheral source and wait its synchronization cycles
 * first. An active registered handler may acknowledge here before application
 * callbacks; the dispatcher will not clear a fresh event after that callback.
 * Also usable on a stopped owned source during Disable/Reset. */
bool Ra4m1AcknowledgeInt(IRQn_Type IrqNo);
#ifdef __cplusplus
}
#endif
#endif
