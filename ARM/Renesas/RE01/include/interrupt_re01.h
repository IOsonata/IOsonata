/**-------------------------------------------------------------------------
@file	interrupt_re01.h

@brief	Renesas RE01 interrupt manager ICU

The RE01 does not have dedicated for each peripheral but rather a share
interrupt. Worst is that there are too many interrupt sources but only 32
interrupt lines are available.  Once an interrupt line is allocated for a
peripheral uses, it cannot be reused by an other.  It's kind a shared exclusive.
Therefore this interrupt manager module attempts to handle interrupt allocations.


@author	Hoang Nguyen Hoan
@date	Feb. 8, 2022

@license

MIT License

Copyright (c) 2022 I-SYST inc. All rights reserved.

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
#ifndef __INTERRUPT_RE01_H__
#define __INTERRUPT_RE01_H__

#include "re01xxx.h"

// Event number definitions (see table 16.6 of the datasheet)
#define RE01_EVTID_DISABLE				0
#define RE01_EVTID_PORT_IRQ0			1
#define RE01_EVTID_PORT_IRQ1			2
#define RE01_EVTID_PORT_IRQ2			3
#define RE01_EVTID_PORT_IRQ3			4
#define RE01_EVTID_PORT_IRQ4			5
#define RE01_EVTID_PORT_IRQ5			6
#define RE01_EVTID_PORT_IRQ6			7
#define RE01_EVTID_PORT_IRQ7			8
#define RE01_EVTID_PORT_IRQ8			0xB4
#define RE01_EVTID_PORT_IRQ9			0xB5
#define RE01_EVTID_DMAC0_INT			9
#define RE01_EVTID_DMAC1_INT			0xA
#define RE01_EVTID_DMAC2_INT			0xB
#define RE01_EVTID_DMAC3_INT			0xC
#define RE01_EVTID_DTC_COMPLETE			0xD
#define RE01_EVTID_ICU_SNZCANCEL		0xF
#define RE01_EVTID_FCU_FIFERR			0x10
#define RE01_EVTID_FCU_FRDYI			0x11
#define RE01_EVTID_LVD_LVD1				0x12
#define RE01_EVTID_LVD_LVDBAT			0x13
#define RE01_EVTID_MOSC_STOP			0x14
#define RE01_EVTID_SYSTEM_SNZREQ		0x15
#define RE01_EVTID_SOL_DH				0x16
#define RE01_EVTID_SOL_DL				0x17
#define RE01_EVTID_AGT0_AGTI			0x18
#define RE01_EVTID_AGT0_AGTCMAI			0x1D
#define RE01_EVTID_AGT0_AGTCMBI			0x1A
#define RE01_EVTID_AGT1_AGTI			0x1B
#define RE01_EVTID_AGT1_AGTCMAI			0x1C
//#define RE01_EVTID_AGT1_AGTCMBI			// Not avail
#define RE01_EVTID_IWDT_NMIUNDF			0x1E
#define RE01_EVTID_WDT_NMIUNDF			0x1F
#define RE01_EVTID_RTC_ALM				0x20
#define RE01_EVTID_RTC_PRD				0x21
#define RE01_EVTID_RTC_CUP				0x22
#define RE01_EVTID_ADC140_ADI			0x23
#define RE01_EVTID_ADC140_GBADI			0x24
#define RE01_EVTID_ADC140_CMPAI			0x25
#define RE01_EVTID_ADC140_CMPBI			0x26
#define RE01_EVTID_ADC140_WCMPM			0x27
#define RE01_EVTID_ADC140_WCMPUM		0x28
#define RE01_EVTID_ADC140_GCADI			0x29
#define RE01_EVTID_ACMP_CMPI			0x2A
#define RE01_EVTID_USBFS_D0FIFO			0x2B
#define RE01_EVTID_USBFS_D1FIFO			0x2C
#define RE01_EVTID_USBFS_USBI			0x2D
#define RE01_EVTID_USBFS_USBR			0x2E
#define RE01_EVTID_IIC0_RXI				0x2F
#define RE01_EVTID_IIC0_TXI				0x30
#define RE01_EVTID_IIC0_TEI				0x31
#define RE01_EVTID_IIC0_EEI				0x32
#define RE01_EVTID_IIC1_RXI				0x33
#define RE01_EVTID_IIC1_TXI				0x34
#define RE01_EVTID_IIC1_TEI				0x35
#define RE01_EVTID_IIC1_EEI				0x36
#define RE01_EVTID_KEY_INTKR			0x37
#define RE01_EVTID_DOC_DOPCI			0x38
#define RE01_EVTID_CAC_FEERI			0x39
#define RE01_EVTID_CAC_MENDI			0x3A
#define RE01_EVTID_CAC_OVFI				0x3B
#define RE01_EVTID_IOPORT_GROUP3		0x3C
#define RE01_EVTID_IOPORT_GROUP2		0x3D
#define RE01_EVTID_ELC_SWEVT0			0x3E
#define RE01_EVTID_ELC_SWEVT1			0x3F
#define RE01_EVTID_POEG_GROUPA			0x40
#define RE01_EVTID_POEG_GROUPB			0x41
#define RE01_EVTID_TMR_CMIA0			0x42
#define RE01_EVTID_TMR_CMIB0			0x43
#define RE01_EVTID_TMR_OVF0				0x44
#define RE01_EVTID_TMR_CMIA1			0x45
#define RE01_EVTID_TMR_CMIB1			0x46
#define RE01_EVTID_TMR_OVF1				0x47
#define RE01_EVTID_CCC_PRD				0x48
#define RE01_EVTID_CCC_CUP				0x49
#define RE01_EVTID_CCC_ERR				0x4A
#define RE01_EVTID_MTDV_PM1INT			0x4B
#define RE01_EVTID_MTDV_PM25INT			0x4C
#define RE01_EVTID_MTDV_PM36INT			0x4D
#define RE01_EVTID_ELC_INT0				0x4E
#define RE01_EVTID_ELC_INT1				0x4F
#define RE01_EVTID_GPT0_CCMPA			0x50
#define RE01_EVTID_GPT0_CCMPB			0x51
#define RE01_EVTID_GPT0_CMPC			0x52
#define RE01_EVTID_GPT0_CMPD			0x53
#define RE01_EVTID_GPT0_OVF				0x54
#define RE01_EVTID_GPT0_UDF				0x55
#define RE01_EVTID_GTP1_CCMPA			0x56
#define RE01_EVTID_GPT1_CCMPB			0x57
#define RE01_EVTID_GPT1_CMPC			0x58
#define RE01_EVTID_GPT1_CMPD			0x59
#define RE01_EVTID_GPT1_OVF				0x5A
#define RE01_EVTID_GPT1_UDF				0x5B
#define RE01_EVTID_GTP2_CCMPA			0x5C
#define RE01_EVTID_GPT2_CCMPB			0x5D
#define RE01_EVTID_GPT2_CMPC			0x5E
#define RE01_EVTID_GPT2_CMPD			0x5F
#define RE01_EVTID_GPT2_OVF				0x60
#define RE01_EVTID_GPT2_UDF				0x61
#define RE01_EVTID_GTP3_CCMPA			0x62
#define RE01_EVTID_GPT3_CCMPB			0x63
#define RE01_EVTID_GPT3_CMPC			0x64
#define RE01_EVTID_GPT3_CMPD			0x65
#define RE01_EVTID_GPT3_OVF				0x66
#define RE01_EVTID_GPT3_UDF				0x67
#define RE01_EVTID_GTP4_CCMPA			0x68
#define RE01_EVTID_GPT4_CCMPB			0x69
#define RE01_EVTID_GPT4_CMPC			0x6A
#define RE01_EVTID_GPT4_CMPD			0x6B
#define RE01_EVTID_GPT4_OVF				0x6C
#define RE01_EVTID_GPT4_UDF				0x6D
#define RE01_EVTID_GTP5_CCMPA			0x6E
#define RE01_EVTID_GPT5_CCMPB			0x6F
#define RE01_EVTID_GPT5_CMPC			0x70
#define RE01_EVTID_GPT5_CMPD			0x71
#define RE01_EVTID_GPT5_OVF				0x72
#define RE01_EVTID_GPT5_UDF				0x73
#define RE01_EVTID_GPT_UVWEDGE			0x74
#define RE01_EVTID_SCI0_SCI0_RXI		0x75
#define RE01_EVTID_SCI0_SCI0_TXI		0x76
#define RE01_EVTID_SCI0_SCI0_TEI		0x77
#define RE01_EVTID_SCI0_SCI0_ERI		0x78
#define RE01_EVTID_SCI0_SCI0_AM			0x79
#define RE01_EVTID_SCI0_SCI0_RXI_OR_ERI	0x7A
#define RE01_EVTID_SCI0_SCI1_RXI		0x7B
#define RE01_EVTID_SCI0_SCI1_TXI		0x7C
#define RE01_EVTID_SCI0_SCI1_TEI		0x7D
#define RE01_EVTID_SCI0_SCI1_ERI		0x7E
#define RE01_EVTID_SCI0_SCI1_AM			0x7F
#define RE01_EVTID_SCI0_SCI2_RXI		0x80
#define RE01_EVTID_SCI0_SCI2_TXI		0x81
#define RE01_EVTID_SCI0_SCI2_TEI		0x82
#define RE01_EVTID_SCI0_SCI2_ERI		0x83
#define RE01_EVTID_SCI0_SCI2_AM			0x84
#define RE01_EVTID_SCI0_SCI3_RXI		0x85
#define RE01_EVTID_SCI0_SCI3_TXI		0x86
#define RE01_EVTID_SCI0_SCI3_TEI		0x87
#define RE01_EVTID_SCI0_SCI3_ERI		0x88
#define RE01_EVTID_SCI0_SCI3_AM			0x89
#define RE01_EVTID_SCI0_SCI4_RXI		0x8A
#define RE01_EVTID_SCI0_SCI4_TXI		0x8B
#define RE01_EVTID_SCI0_SCI4_TEI		0x8C
#define RE01_EVTID_SCI0_SCI4_ERI		0x8D
#define RE01_EVTID_SCI0_SCI4_AM			0x8E
#define RE01_EVTID_SCI0_SCI5_RXI		0x8F
#define RE01_EVTID_SCI0_SCI5_TXI		0x90
#define RE01_EVTID_SCI0_SCI5_TEI		0x91
#define RE01_EVTID_SCI0_SCI5_ERI		0x92
#define RE01_EVTID_SCI0_SCI5_AM			0x93
#define RE01_EVTID_SCI0_SCI9_RXI		0x94
#define RE01_EVTID_SCI0_SCI9_TXI		0x95
#define RE01_EVTID_SCI0_SCI9_TEI		0x96
#define RE01_EVTID_SCI0_SCI9_ERI		0x97
#define RE01_EVTID_SCI0_SCI9_AM			0x98
#define RE01_EVTID_SPI0_SPRI			0x99
#define RE01_EVTID_SPI0_SPTI			0x9A
#define RE01_EVTID_SPI0_SPII			0x9B
#define RE01_EVTID_SPI0_SPEI			0x9C
#define RE01_EVTID_SPI0_SPTEND			0x9D
#define RE01_EVTID_SPI1_SPRI			0x9E
#define RE01_EVTID_SPI1_SPTI			0x9F
#define RE01_EVTID_SPI1_SPII			0xA0
#define RE01_EVTID_SPI1_SPEI			0xA1
#define RE01_EVTID_SPI1_SPTEND			0xA2
#define RE01_EVTID_QSPI_INTR			0xA3
#define RE01_EVTID_DIV_CALCCOMP			0xA4
#define RE01_EVTID_MLCD_TEI				0xA6
#define RE01_EVTID_MLCD_TEMI			0xA7
#define RE01_EVTID_GDT_DATII			0xA8
#define RE01_EVTID_GDT_DATOI			0xA9
#define RE01_EVTID_GDT_FDCENDI			0xAA
#define RE01_EVTID_PROC_BUSY			0xAB
#define RE01_EVTID_ROMOK				0xAC
#define RE01_EVTID_LONG_PLG				0xAD
#define RE01_EVTID_TEST_BUSY			0xAE
#define RE01_EVTID_WRRDY0				0xAF
#define RE01_EVTID_WRRDY4				0xB0
#define RE01_EVTID_RDRDY0				0xB1
#define RE01_EVTID_INTEGRATE_WRRDY		0xB2
#define RE01_EVTID_INTEGRATE_RDRDY		0xB3


#define RE01_IELS_CNT		32

/**
 * @brief	User's IRQ handler callback function type
 *
 * @param 	IntNo	: IRQn number
 * @param 	pCtx	: Pointer to user's private data
 */
typedef void (*Re01IRQHandler_t)(int IntNo, void *pCtx);

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Register Interrupt hnadler
 *
 * Call this function to allocate IRQ and register handler callback.
 * All NVIC interrupt calls are handled internally. User Handler function
 * does not need to call NVIC_...
 *
 * @param 	EvtId 	: Peripheral Event ID listed above RE01_EVTID_...
 * @param 	Prio	: Interrupt priority number
 * @param 	pHandler: Pointer to user's handler function
 * @param 	pCtx	: Pointer to user private data to be passed to the handler callback
 *
 * @return	IRQn number allocated if success
 * 			-1 failed
 */
IRQn_Type Re01RegisterIntHandler(uint8_t EvtId, int Prio, Re01IRQHandler_t pHandler, void *pCtx);

/**
 * @brief	Release interrupt reserved by Re01RegisterIntHandler
 *
 * Call this to de-allocate the interrupt
 *
 * @param 	IrqNo : IRQn number returned by preivous call to Re01RegisterIntHandler
 *
 */
void Re01UnregisterIntHandler(IRQn_Type IrqNo);

#ifdef __cplusplus
}
#endif

#endif

