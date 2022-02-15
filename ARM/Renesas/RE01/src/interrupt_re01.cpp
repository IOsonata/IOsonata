/**-------------------------------------------------------------------------
@file	interrupt_re01.cpp

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
#include "re01xxx.h"

#include "interrupt_re01.h"

#define IELS_GROUP_CNT		8

typedef struct {
	uint8_t iels[IELS_GROUP_CNT];
} Re01EvtIdMap_t;

// Table 16.7
static const Re01EvtIdMap_t s_Re01EvtIdMap[] = {
	{0,},						// RE01_EVTID_DISABLE				0
	{1, 0, 0, 0, 1, 0, 0, 0},	// RE01_EVTID_PORT_IRQ0				1
	{0, 1, 0, 0, 0, 1, 0, 0},	// RE01_EVTID_PORT_IRQ1				2
	{0, 0, 1, 0, 0, 0, 1, 0},	// RE01_EVTID_PORT_IRQ2				3
	{0, 0, 0, 1, 0, 0, 0, 1},	// RE01_EVTID_PORT_IRQ3				4
	{0, 0, 0, 0, 0x13, 0, 0, 0},// RE01_EVTID_PORT_IRQ4				5
	{0, 0, 0, 0, 0, 0x12, 0, 0},// RE01_EVTID_PORT_IRQ5				6
	{0, 0, 0, 0, 0, 0, 0x12, 0},// RE01_EVTID_PORT_IRQ6				7
	{0, 0, 0, 0, 0, 0, 0, 0x13},// RE01_EVTID_PORT_IRQ7				8
	{2, 0, 0, 0, 2, 0, 0, 0},	// RE01_EVTID_DMAC0_INT				9
	{0, 2, 0, 0, 0, 2, 0, 0},	// RE01_EVTID_DMAC1_INT				0xA
	{0, 0, 2, 0, 0, 0, 2, 0},	// RE01_EVTID_DMAC2_INT				0xB
	{0, 0, 0, 2, 0, 0, 0, 2},	// RE01_EVTID_DMAC3_INT				0xC
	{3, 0, 0, 0, 3, 0, 0, 0},	// RE01_EVTID_DTC_COMPLETE			0xD
	{0,},						// 0xE
	{4, 0, 0, 0, 4, 0, 0, 0},	// RE01_EVTID_ICU_SNZCANCEL			0xF
	{0, 3, 0, 0, 0, 3, 0, 0},	// RE01_EVTID_FCU_FIFERR			0x10
	{0, 0, 3, 0, 0, 0, 3, 0},	// RE01_EVTID_FCU_FRDYI				0x11
	{5, 0, 0, 0, 5, 0, 0, 0},	// RE01_EVTID_LVD_LVD1				0x12
	{0, 4, 0, 0, 0, 4, 0, 0},	// RE01_EVTID_LVD_LVDBAT			0x13
	{0, 0, 0, 0, 0, 0, 0x13, 0},// RE01_EVTID_MOSC_STOP				0x14
	{0, 0, 0, 3, 0, 0, 0, 3},	// RE01_EVTID_SYSTEM_SNZREQ			0x15
	{0x13, 0, 0, 0, 0, 0, 0, 0},// RE01_EVTID_SOL_DH				0x16
	{0, 0x12, 0, 0, 0, 0, 0, 0},// RE01_EVTID_SOL_DL				0x17
	{0, 0, 0, 0x13, 0, 0, 0, 0},// RE01_EVTID_AGT0_AGTI				0x18
	{0,},						// 0x19
	{0, 0x13, 0, 0, 0, 0, 0, 0},// RE01_EVTID_AGT0_AGTCMBI			0x1A
	{6, 0, 0, 0, 6, 0, 0, 0},	// RE01_EVTID_AGT1_AGTI				0x1B
	{0, 5, 0, 0, 0, 5, 0, 0},	// RE01_EVTID_AGT1_AGTCMAI			0x1C
	{0, 0, 4, 0, 0, 0, 4, 0},	// RE01_EVTID_AGT0_AGTCMAI			0x1D
	{0, 0, 0, 4, 0, 0, 0, 4},	// RE01_EVTID_IWDT_NMIUNDF			0x1E
	{7, 0, 0, 0, 7, 0, 0, 0},	// RE01_EVTID_WDT_NMIUNDF			0x1F
	{0, 6, 0, 0, 0, 6, 0, 0},	// RE01_EVTID_RTC_ALM				0x20
	{0, 0, 5, 0, 0, 0, 5, 0},	// RE01_EVTID_RTC_PRD				0x21
	{0, 0, 0, 5, 0, 0, 0, 5},	// RE01_EVTID_RTC_CUP				0x22
	{8, 0, 0, 0, 8, 0, 0, 0},	// RE01_EVTID_ADC140_ADI			0x23
	{0, 7, 0, 0, 0, 7, 0, 0},	// RE01_EVTID_ADC140_GBADI			0x24
	{0, 0, 6, 0, 0, 0, 6, 0},	// RE01_EVTID_ADC140_CMPAI			0x25
	{0, 0, 0, 6, 0, 0, 0, 6},	// RE01_EVTID_ADC140_CMPBI			0x26
	{9, 0, 0, 0, 9, 0, 0, 0},	// RE01_EVTID_ADC140_WCMPM			0x27
	{0, 8, 0, 0, 0, 8, 0, 0},	// RE01_EVTID_ADC140_WCMPUM			0x28
	{0, 0, 7, 0, 0, 0, 7, 0},	// RE01_EVTID_ADC140_GCADI			0x29
	{0, 0, 0, 7, 0, 0, 0, 7},	// RE01_EVTID_ACMP_CMPI				0x2A
	{0x15, 0, 0, 0, 0, 0, 0, 0},// RE01_EVTID_USBFS_D0FIFO			0x2B
	{0, 0x14, 0, 0, 0, 0, 0, 0},// RE01_EVTID_USBFS_D1FIFO			0x2C
	{0, 0, 0x12, 0, 0, 0, 0, 0},// RE01_EVTID_USBFS_USBI			0x2D
	{0, 0, 0, 0x14, 0, 0, 0, 0},// RE01_EVTID_USBFS_USBR			0x2E
	{0xa, 0, 0, 0, 0xa, 0, 0, 0},// RE01_EVTID_IIC0_RXI				0x2F
	{0, 9, 0, 0, 0, 9, 0, 0},	// RE01_EVTID_IIC0_TXI				0x30
	{0, 0, 8, 0, 0, 0, 8, 0},	// RE01_EVTID_IIC0_TEI				0x31
	{0, 0, 0, 8, 0, 0, 0, 8},	// RE01_EVTID_IIC0_EEI				0x32
	{0x16, 0, 0, 0, 0, 0, 0, 0},// RE01_EVTID_IIC1_RXI				0x33
	{0, 0x15, 0, 0, 0, 0, 0, 0},// RE01_EVTID_IIC1_TXI				0x34
	{0, 0, 0, 0, 0x14, 0, 0, 0},// RE01_EVTID_IIC1_TEI				0x35
	{0, 0, 0, 0, 0, 0x13, 0, 0},// RE01_EVTID_IIC1_EEI				0x36
	{0x17, 0, 0, 0, 0, 0, 0, 0},// RE01_EVTID_KEY_INTKR				0x37
	{0, 0xa, 0, 0, 0, 0xa, 0, 0},// RE01_EVTID_DOC_DOPCI			0x38
	{0, 0xb, 0, 0, 0, 0xb, 0, 0},// RE01_EVTID_CAC_FEERI			0x39
	{0, 0, 9, 0, 0, 0, 9, 0},	// RE01_EVTID_CAC_MENDI				0x3A
	{0, 0, 0, 9, 0, 0, 0, 9},	// RE01_EVTID_CAC_OVFI				0x3B
	{0, 0x16, 0, 0, 0, 0, 0, 0},// RE01_EVTID_IOPORT_GROUP3			0x3C
	{0, 0, 0x13, 0, 0, 0, 0, 0},// RE01_EVTID_IOPORT_GROUP2			0x3D
	{0, 0, 0xa, 0, 0, 0, 0xa, 0},// RE01_EVTID_ELC_SWEVT0			0x3E
	{0, 0, 0, 0xa, 0, 0, 0, 0xa},// RE01_EVTID_ELC_SWEVT1			0x3F
	{0, 0, 0xb, 0, 0, 0, 0xb, 0},// RE01_EVTID_POEG_GROUPA			0x40
	{0, 0, 0, 0xb, 0, 0, 0, 0xb},// RE01_EVTID_POEG_GROUPB			0x41
	{0, 0, 0x14, 0, 0, 0, 0, 0},// RE01_EVTID_TMR_CMIA0				0x42
	{0, 0, 0, 0x15, 0, 0, 0, 0},// RE01_EVTID_TMR_CMIB0				0x43
	{0, 0, 0, 0, 0x15, 0, 0, 0},// RE01_EVTID_TMR_OVF0				0x44
	{0, 0, 0, 0, 0, 0x14, 0, 0},// RE01_EVTID_TMR_CMIA1				0x45
	{0, 0, 0, 0, 0, 0, 0x14, 0},// RE01_EVTID_TMR_CMIB1				0x46
	{0, 0, 0, 0, 0, 0, 0, 0x14},// RE01_EVTID_TMR_OVF1				0x47
	{0xb, 0, 0, 0, 0xb, 0, 0, 0},// RE01_EVTID_CCC_PRD				0x48
	{0, 0xc, 0, 0, 0, 0xc, 0, 0},// RE01_EVTID_CCC_CUP				0x49
	{0, 0, 0, 0, 0, 0, 0x15, 0},// RE01_EVTID_CCC_ERR				0x4A
	{0, 0, 0, 0xc, 0, 0, 0, 0xc},// RE01_EVTID_MTDV_PM1INT			0x4B
	{0, 0, 0, 0, 0x16, 0, 0, 0},// RE01_EVTID_MTDV_PM25INT			0x4C
	{0, 0, 0, 0, 0, 0x15, 0, 0},// RE01_EVTID_MTDV_PM36INT			0x4D
	{0, 0, 0, 0x16, 0, 0, 0, 0},// RE01_EVTID_ELC_INT0				0x4E
	{0, 0, 0, 0, 0, 0, 0, 0x15},// RE01_EVTID_ELC_INT1				0x4F
	{0xc, 0, 0, 0, 0xc, 0, 0, 0},// RE01_EVTID_GPT0_CCMPA			0x50
	{0, 0xd, 0, 0, 0, 0xd, 0, 0},// RE01_EVTID_GPT0_CCMPB			0x51
	{0, 0, 0xc, 0, 0, 0, 0xc, 0},// RE01_EVTID_GPT0_CMPC			0x52
	{0, 0, 0, 0xd, 0, 0, 0, 0xd},// RE01_EVTID_GPT0_CMPD			0x53
	{0xd, 0, 0, 0, 0xd, 0, 0, 0},// RE01_EVTID_GPT0_OVF				0x54
	{0, 0xe, 0, 0, 0, 0xe, 0, 0},// RE01_EVTID_GPT0_UDF				0x55
	{0x18, 0, 0, 0, 0, 0, 0, 0},// RE01_EVTID_GTP1_CCMPA			0x56
	{0, 0x17, 0, 0, 0, 0, 0, 0},// RE01_EVTID_GPT1_CCMPB			0x57
	{0, 0, 0x15, 0, 0, 0, 0, 0},// RE01_EVTID_GPT1_CMPC				0x58
	{0, 0, 0, 0x17, 0, 0, 0, 0},// RE01_EVTID_GPT1_CMPD				0x59
	{0, 0, 0, 0, 0x17, 0, 0, 0},// RE01_EVTID_GPT1_OVF				0x5A
	{0, 0, 0, 0, 0, 0x16, 0, 0},// RE01_EVTID_GPT1_UDF				0x5B
	{0xe, 0, 0, 0, 0xe, 0, 0, 0},// RE01_EVTID_GTP2_CCMPA			0x5C
	{0, 0xf, 0, 0, 0, 0xf, 0, 0},// RE01_EVTID_GPT2_CCMPB			0x5D
	{0, 0, 0xd, 0, 0, 0, 0xd, 0},// RE01_EVTID_GPT2_CMPC			0x5E
	{0, 0, 0, 0xe, 0, 0, 0, 0xe},// RE01_EVTID_GPT2_CMPD			0x5F
	{0, 0, 0xe, 0, 0, 0, 0xe, 0},// RE01_EVTID_GPT2_OVF				0x60
	{0, 0, 0, 0xf, 0, 0, 0, 0xf},// RE01_EVTID_GPT2_UDF				0x61
	{0, 0, 0, 0, 0x18, 0, 0, 0},// RE01_EVTID_GTP3_CCMPA			0x62
	{0, 0, 0, 0, 0, 0x17, 0, 0},// RE01_EVTID_GPT3_CCMPB			0x63
	{0, 0, 0, 0, 0, 0, 0x16, 0},// RE01_EVTID_GPT3_CMPC				0x64
	{0, 0, 0, 0, 0, 0, 0, 0x16},// RE01_EVTID_GPT3_CMPD				0x65
	{0x19, 0, 0, 0, 0, 0, 0, 0},// RE01_EVTID_GPT3_OVF				0x66
	{0, 0x18, 0, 0, 0, 0, 0, 0},// RE01_EVTID_GPT3_UDF				0x67
	{0x1a, 0, 0, 0, 0, 0, 0, 0},// RE01_EVTID_GTP4_CCMPA			0x68
	{0, 0x19, 0, 0, 0, 0, 0, 0},// RE01_EVTID_GPT4_CCMPB			0x69
	{0, 0, 0x16, 0, 0, 0, 0, 0},// RE01_EVTID_GPT4_CMPC				0x6A
	{0, 0, 0, 0x18, 0, 0, 0, 0},// RE01_EVTID_GPT4_CMPD				0x6B
	{0, 0, 0, 0, 0, 0, 0x17, 0},// RE01_EVTID_GPT4_OVF				0x6C
	{0, 0, 0, 0, 0, 0, 0, 0x17},// RE01_EVTID_GPT4_UDF				0x6D
	{0, 0, 0, 0, 0x19, 0, 0, 0},// RE01_EVTID_GTP5_CCMPA			0x6E
	{0, 0, 0, 0, 0, 0x18, 0, 0},// RE01_EVTID_GPT5_CCMPB			0x6F
	{0, 0, 0, 0, 0, 0, 0x18, 0},// RE01_EVTID_GPT5_CMPC				0x70
	{0, 0, 0, 0, 0, 0, 0, 0x18},// RE01_EVTID_GPT5_CMPD				0x71
	{0, 0, 0x17, 0, 0, 0, 0, 0},// RE01_EVTID_GPT5_OVF				0x72
	{0, 0, 0, 0x19, 0, 0, 0, 0},// RE01_EVTID_GPT5_UDF				0x73
	{0xf, 0, 0, 0, 0xf, 0, 0, 0},// RE01_EVTID_GPT_UVWEDGE			0x74
	{0x10, 0, 0, 0, 0x10, 0, 0, 0},// RE01_EVTID_SCI0_SCI0_RXI		0x75
	{0, 0x10, 0, 0, 0, 0x10, 0, 0},// RE01_EVTID_SCI0_SCI0_TXI		0x76
	{0, 0, 0xf, 0, 0, 0, 0xf, 0},// RE01_EVTID_SCI0_SCI0_TEI		0x77
	{0, 0, 0, 0x10, 0, 0, 0, 0x10},// RE01_EVTID_SCI0_SCI0_ERI		0x78
	{0x11, 0, 0, 0, 0x11, 0, 0, 0},// RE01_EVTID_SCI0_SCI0_AM		0x79
	{0,},						// RE01_EVTID_SCI0_SCI0_RXI_OR_ERI	0x7A
	{0x1b, 0, 0, 0, 0, 0, 0, 0},// RE01_EVTID_SCI0_SCI1_RXI			0x7B
	{0, 0x1a, 0, 0, 0, 0, 0, 0},// RE01_EVTID_SCI0_SCI1_TXI			0x7C
	{0, 0, 0x18, 0, 0, 0, 0, 0},// RE01_EVTID_SCI0_SCI1_TEI			0x7D
	{0, 0, 0, 0x1a, 0, 0, 0, 0},// RE01_EVTID_SCI0_SCI1_ERI			0x7E
	{0, 0, 0, 0, 0, 0x19, 0, 0},// RE01_EVTID_SCI0_SCI1_AM			0x7F
	{0, 0, 0, 0, 0x1a, 0, 0, 0},// RE01_EVTID_SCI0_SCI2_RXI			0x80
	{0, 0, 0, 0, 0, 0x1a, 0, 0},// RE01_EVTID_SCI0_SCI2_TXI			0x81
	{0, 0, 0, 0, 0, 0, 0x19, 0},// RE01_EVTID_SCI0_SCI2_TEI			0x82
	{0, 0, 0, 0, 0, 0, 0, 0x19},// RE01_EVTID_SCI0_SCI2_ERI			0x83
	{0, 0x1b, 0, 0, 0, 0, 0, 0},// RE01_EVTID_SCI0_SCI2_AM			0x84
	{0x1c, 0, 0, 0, 0, 0, 0, 0},// RE01_EVTID_SCI0_SCI3_RXI			0x85
	{0, 0x1c, 0, 0, 0, 0, 0, 0},// RE01_EVTID_SCI0_SCI3_TXI			0x86
	{0, 0, 0x19, 0, 0, 0, 0, 0},// RE01_EVTID_SCI0_SCI3_TEI			0x87
	{0, 0, 0, 0x1b, 0, 0, 0, 0},// RE01_EVTID_SCI0_SCI3_ERI			0x88
	{0, 0, 0, 0, 0, 0, 0x1a, 0},// RE01_EVTID_SCI0_SCI3_AM			0x89
	{0, 0, 0, 0, 0x1b, 0, 0, 0},// RE01_EVTID_SCI0_SCI4_RXI			0x8A
	{0, 0, 0, 0, 0, 0x1b, 0, 0},// RE01_EVTID_SCI0_SCI4_TXI			0x8B
	{0, 0, 0, 0, 0, 0, 0x1b, 0},// RE01_EVTID_SCI0_SCI4_TEI			0x8C
	{0, 0, 0, 0, 0, 0, 0, 0x1a},// RE01_EVTID_SCI0_SCI4_ERI			0x8D
	{0, 0, 0x1a, 0, 0, 0, 0, 0},// RE01_EVTID_SCI0_SCI4_AM			0x8E
	{0x1d, 0, 0, 0, 0, 0, 0, 0},// RE01_EVTID_SCI0_SCI5_RXI			0x8F
	{0, 0x1d, 0, 0, 0, 0, 0, 0},// RE01_EVTID_SCI0_SCI5_TXI			0x90
	{0, 0, 0x1b, 0, 0, 0, 0, 0},// RE01_EVTID_SCI0_SCI5_TEI			0x91
	{0, 0, 0, 0x1c, 0, 0, 0, 0},// RE01_EVTID_SCI0_SCI5_ERI			0x92
	{0, 0, 0, 0, 0, 0, 0, 0x1b},// RE01_EVTID_SCI0_SCI5_AM			0x93
	{0, 0, 0, 0, 0x1c, 0, 0, 0},// RE01_EVTID_SCI0_SCI9_RXI			0x94
	{0, 0, 0, 0, 0, 0x1c, 0, 0},// RE01_EVTID_SCI0_SCI9_TXI			0x95
	{0, 0, 0, 0, 0, 0, 0x1c, 0},// RE01_EVTID_SCI0_SCI9_TEI			0x96
	{0, 0, 0, 0, 0, 0, 0, 0x1c},// RE01_EVTID_SCI0_SCI9_ERI			0x97
	{0, 0, 0, 0x1d, 0, 0, 0, 0},// RE01_EVTID_SCI0_SCI9_AM			0x98
	{0x12, 0, 0, 0, 0x12, 0, 0, 0},// RE01_EVTID_SPI0_SPRI			0x99
	{0, 0x11, 0, 0, 0, 0x11, 0, 0},// RE01_EVTID_SPI0_SPTI			0x9A
	{0, 0, 0x10, 0, 0, 0, 0x10, 0},// RE01_EVTID_SPI0_SPII			0x9B
	{0, 0, 0, 0x11, 0, 0, 0, 0x11},// RE01_EVTID_SPI0_SPEI			0x9C
	{0, 0, 0x11, 0, 0, 0, 0x11, 0},// RE01_EVTID_SPI0_SPTEND		0x9D
	{0, 0, 0, 0, 0x1d, 0, 0, 0},// RE01_EVTID_SPI1_SPRI				0x9E
	{0, 0, 0, 0, 0, 0x1d, 0, 0},// RE01_EVTID_SPI1_SPTI				0x9F
	{0, 0, 0x1c, 0, 0, 0, 0, 0},// RE01_EVTID_SPI1_SPII				0xA0
	{0, 0, 0, 0, 0, 0, 0, 0x1d},// RE01_EVTID_SPI1_SPEI				0xA1
	{0, 0, 0, 0, 0, 0, 0x1d, 0},// RE01_EVTID_SPI1_SPTEND			0xA2
	{0, 0, 0, 0x12, 0, 0, 0, 0x12},// RE01_EVTID_QSPI_INTR			0xA3
	{0, 0, 0, 0, 0, 0, 0, 0x1e},// RE01_EVTID_DIV_CALCCOMP			0xA4
	{0, 0x1e, 0, 0, 0, 0, 0, 0},// RE01_EVTID_MLCD_TEI				0xA6
	{0, 0, 0x1d, 0, 0, 0, 0, 0},// RE01_EVTID_MLCD_TEMI				0xA7
	{0, 0, 0, 0x1e, 0, 0, 0, 0},// RE01_EVTID_GDT_DATII				0xA8
	{0, 0, 0, 0, 0x1e, 0, 0, 0},// RE01_EVTID_GDT_DATOI				0xA9
	{0, 0, 0, 0, 0, 0x1e, 0, 0},// RE01_EVTID_GDT_FDCENDI			0xAA
	{0x1f, 0, 0, 0, 0, 0, 0, 0},// RE01_EVTID_PROC_BUSY				0xAB
	{0, 0x1f, 0, 0, 0, 0, 0, 0},// RE01_EVTID_ROMOK					0xAC
	{0, 0, 0x1e, 0, 0, 0, 0, 0},// RE01_EVTID_LONG_PLG				0xAD
	{0, 0, 0, 0x1f, 0, 0, 0, 0},// RE01_EVTID_TEST_BUSY				0xAE
	{0, 0, 0, 0, 0x1f, 0, 0, 0},// RE01_EVTID_WRRDY0				0xAF
	{0, 0, 0, 0, 0, 0x1f, 0, 0},// RE01_EVTID_WRRDY4				0xB0
	{0, 0, 0, 0, 0, 0, 0x1e, 0},// RE01_EVTID_RDRDY0				0xB1
	{0, 0, 0, 0, 0, 0, 0, 0x1f},// RE01_EVTID_INTEGRATE_WRRDY		0xB2
	{0, 0, 0, 0, 0, 0, 0x1f, 0},// RE01_EVTID_INTEGRATE_RDRDY		0xB3
	{0x1e, 0, 0, 0, 0, 0, 0, 0},// RE01_EVTID_PORT_IRQ8				0xB4
	{0, 0, 0x1f, 0, 0, 0, 0, 0},// RE01_EVTID_PORT_IRQ9				0xB5
};

static const int  s_NbRe01EvtIdMap = sizeof(s_Re01EvtIdMap) / sizeof(Re01EvtIdMap_t);

typedef struct {
	Re01IRQHandler_t Handler;
	void *pCtx;
} Re01IrqTbl_t;

static Re01IrqTbl_t s_IrqHandlerTbl[RE01_IELS_CNT] = {
	{0,},
};

IRQn_Type Re01RegisterIntHandler(uint8_t EvtId, int Prio, Re01IRQHandler_t pHandler, void *pCtx)
{
	for (int grp = 0; grp < IELS_GROUP_CNT; grp++)
	{
		if (s_Re01EvtIdMap[EvtId].iels[grp] != 0)
		{
			for (int i = 0; i < 4; i++)
			{
				uint8_t idx = (i << 3) + grp;
				if (s_IrqHandlerTbl[idx].Handler == 0)
				{
					s_IrqHandlerTbl[idx].Handler = pHandler;
					s_IrqHandlerTbl[idx].pCtx = pCtx;

					IRQn_Type irq = (IRQn_Type)((int)IEL0_IRQn + idx);

					__IOM uint32_t *ielsr = (__IOM uint32_t*)&ICU->IELSR0;

					ielsr[idx] = s_Re01EvtIdMap[EvtId].iels[grp];

					NVIC_ClearPendingIRQ(irq);
					NVIC_SetPriority(irq, Prio);
					NVIC_EnableIRQ(irq);

					return irq;
				}
			}
		}
	}

	return (IRQn_Type)-1;
}

void Re01UnregisterIntHandler(IRQn_Type IrqNo)
{
	if (IrqNo >= IEL0_IRQn && IrqNo <= IEL31_IRQn)
	{
		int idx = IrqNo - IEL0_IRQn;

		__IOM uint32_t *ielsr = (__IOM uint32_t*)&ICU->IELSR0;

		ielsr[idx] = 0;

		NVIC_DisableIRQ(IrqNo);
		NVIC_ClearPendingIRQ(IrqNo);

		s_IrqHandlerTbl[idx].Handler = nullptr;
		s_IrqHandlerTbl[idx].pCtx = nullptr;
	}
}

extern "C" {

static inline void __Re01IntHandler(int IntNo)
{
	if (s_IrqHandlerTbl[IntNo].Handler)
	{
		s_IrqHandlerTbl[IntNo].Handler(IntNo, s_IrqHandlerTbl[IntNo].pCtx);
	}
}

__WEAK void IEL0_IRQHandler()
{
	if (ICU->IELSR0 & ICU_IELSR0_IR_Msk)
	{
		__Re01IntHandler(0);

		ICU->IELSR0 &= ~ICU_IELSR0_IR_Msk;
	}
	NVIC_ClearPendingIRQ(IEL0_IRQn);
}

__WEAK void IEL1_IRQHandler()
{
	if (ICU->IELSR1 & ICU_IELSR1_IR_Msk)
	{
		__Re01IntHandler(1);

		ICU->IELSR1 &= ~ICU_IELSR1_IR_Msk;
	}
	NVIC_ClearPendingIRQ(IEL1_IRQn);
}

__WEAK void IEL2_IRQHandler()
{
	if (ICU->IELSR2 & ICU_IELSR2_IR_Msk)
	{
		__Re01IntHandler(2);

		ICU->IELSR2 &= ~ICU_IELSR2_IR_Msk;
	}
	NVIC_ClearPendingIRQ(IEL2_IRQn);
}

__WEAK void IEL3_IRQHandler()
{
	if (ICU->IELSR3 & ICU_IELSR3_IR_Msk)
	{
		__Re01IntHandler(3);

		ICU->IELSR3 &= ~ICU_IELSR3_IR_Msk;
	}
	NVIC_ClearPendingIRQ(IEL3_IRQn);
}

__WEAK void IEL4_IRQHandler()
{
	if (ICU->IELSR4 & ICU_IELSR4_IR_Msk)
	{
		__Re01IntHandler(4);

		ICU->IELSR4 &= ~ICU_IELSR4_IR_Msk;
	}
	NVIC_ClearPendingIRQ(IEL4_IRQn);
}

__WEAK void IEL5_IRQHandler()
{
	if (ICU->IELSR5 & ICU_IELSR5_IR_Msk)
	{
		__Re01IntHandler(5);

		ICU->IELSR5 &= ~ICU_IELSR5_IR_Msk;
	}
	NVIC_ClearPendingIRQ(IEL5_IRQn);
}

__WEAK void IEL6_IRQHandler()
{
	if (ICU->IELSR6 & ICU_IELSR6_IR_Msk)
	{
		__Re01IntHandler(6);

		ICU->IELSR6 &= ~ICU_IELSR6_IR_Msk;
	}
	NVIC_ClearPendingIRQ(IEL6_IRQn);
}

__WEAK void IEL7_IRQHandler()
{
	if (ICU->IELSR7 & ICU_IELSR7_IR_Msk)
	{
		__Re01IntHandler(7);

		ICU->IELSR7 &= ~ICU_IELSR7_IR_Msk;
	}
	NVIC_ClearPendingIRQ(IEL7_IRQn);
}

__WEAK void IEL8_IRQHandler()
{
	if (ICU->IELSR8 & ICU_IELSR8_IR_Msk)
	{
		__Re01IntHandler(8);

		ICU->IELSR8 &= ~ICU_IELSR8_IR_Msk;
	}
	NVIC_ClearPendingIRQ(IEL8_IRQn);
}

__WEAK void IEL9_IRQHandler()
{
	if (ICU->IELSR9 & ICU_IELSR9_IR_Msk)
	{
		__Re01IntHandler(9);

		ICU->IELSR9 &= ~ICU_IELSR9_IR_Msk;
	}
	NVIC_ClearPendingIRQ(IEL9_IRQn);
}

__WEAK void IEL10_IRQHandler()
{
	if (ICU->IELSR10 & ICU_IELSR10_IR_Msk)
	{
		__Re01IntHandler(10);

		ICU->IELSR10 &= ~ICU_IELSR10_IR_Msk;
	}
	NVIC_ClearPendingIRQ(IEL10_IRQn);
}

__WEAK void IEL11_IRQHandler()
{
	if (ICU->IELSR11 & ICU_IELSR11_IR_Msk)
	{
		__Re01IntHandler(11);

		ICU->IELSR11 &= ~ICU_IELSR11_IR_Msk;
	}
	NVIC_ClearPendingIRQ(IEL11_IRQn);
}

__WEAK void IEL12_IRQHandler()
{
	if (ICU->IELSR12 & ICU_IELSR12_IR_Msk)
	{
		__Re01IntHandler(12);

		ICU->IELSR12 &= ~ICU_IELSR12_IR_Msk;
	}
	NVIC_ClearPendingIRQ(IEL12_IRQn);
}

__WEAK void IEL13_IRQHandler()
{
	if (ICU->IELSR13 & ICU_IELSR13_IR_Msk)
	{
		__Re01IntHandler(13);

		ICU->IELSR13 &= ~ICU_IELSR13_IR_Msk;
	}
	NVIC_ClearPendingIRQ(IEL13_IRQn);
}

__WEAK void IEL14_IRQHandler()
{
	if (ICU->IELSR14 & ICU_IELSR14_IR_Msk)
	{
		__Re01IntHandler(14);

		ICU->IELSR14 &= ~ICU_IELSR14_IR_Msk;
	}
	NVIC_ClearPendingIRQ(IEL14_IRQn);
}

__WEAK void IEL15_IRQHandler()
{
	if (ICU->IELSR15 & ICU_IELSR15_IR_Msk)
	{
		__Re01IntHandler(15);

		ICU->IELSR15 &= ~ICU_IELSR15_IR_Msk;
	}
	NVIC_ClearPendingIRQ(IEL15_IRQn);
}

__WEAK void IEL16_IRQHandler()
{
	if (ICU->IELSR16 & ICU_IELSR16_IR_Msk)
	{
		__Re01IntHandler(16);

		ICU->IELSR16 &= ~ICU_IELSR16_IR_Msk;
	}
	NVIC_ClearPendingIRQ(IEL16_IRQn);
}

__WEAK void IEL17_IRQHandler()
{
	if (ICU->IELSR17 & ICU_IELSR17_IR_Msk)
	{
		__Re01IntHandler(17);

		ICU->IELSR17 &= ~ICU_IELSR17_IR_Msk;
	}
	NVIC_ClearPendingIRQ(IEL17_IRQn);
}

__WEAK void IEL18_IRQHandler()
{
	if (ICU->IELSR18 & ICU_IELSR18_IR_Msk)
	{
		__Re01IntHandler(18);

		ICU->IELSR18 &= ~ICU_IELSR18_IR_Msk;
	}
	NVIC_ClearPendingIRQ(IEL18_IRQn);
}

__WEAK void IEL19_IRQHandler()
{
	if (ICU->IELSR19 & ICU_IELSR19_IR_Msk)
	{
		__Re01IntHandler(19);

		ICU->IELSR19 &= ~ICU_IELSR19_IR_Msk;
	}
	NVIC_ClearPendingIRQ(IEL19_IRQn);
}

__WEAK void IEL20_IRQHandler()
{
	if (ICU->IELSR20 & ICU_IELSR20_IR_Msk)
	{
		__Re01IntHandler(20);

		ICU->IELSR20 &= ~ICU_IELSR20_IR_Msk;
	}
	NVIC_ClearPendingIRQ(IEL20_IRQn);
}

__WEAK void IEL21_IRQHandler()
{
	if (ICU->IELSR21 & ICU_IELSR21_IR_Msk)
	{
		__Re01IntHandler(21);

		ICU->IELSR21 &= ~ICU_IELSR21_IR_Msk;
	}
	NVIC_ClearPendingIRQ(IEL21_IRQn);
}

__WEAK void IEL22_IRQHandler()
{
	if (ICU->IELSR22 & ICU_IELSR22_IR_Msk)
	{
		__Re01IntHandler(22);

		ICU->IELSR22 &= ~ICU_IELSR22_IR_Msk;
	}
	NVIC_ClearPendingIRQ(IEL22_IRQn);
}

__WEAK void IEL23_IRQHandler()
{
	if (ICU->IELSR23 & ICU_IELSR23_IR_Msk)
	{
		__Re01IntHandler(23);

		ICU->IELSR23 &= ~ICU_IELSR23_IR_Msk;
	}
	NVIC_ClearPendingIRQ(IEL23_IRQn);
}

__WEAK void IEL24_IRQHandler()
{
	if (ICU->IELSR24 & ICU_IELSR24_IR_Msk)
	{
		__Re01IntHandler(24);

		ICU->IELSR24 &= ~ICU_IELSR24_IR_Msk;
	}
	NVIC_ClearPendingIRQ(IEL24_IRQn);
}

__WEAK void IEL25_IRQHandler()
{
	if (ICU->IELSR25 & ICU_IELSR25_IR_Msk)
	{
		__Re01IntHandler(25);

		ICU->IELSR25 &= ~ICU_IELSR25_IR_Msk;
	}
	NVIC_ClearPendingIRQ(IEL25_IRQn);
}

__WEAK void IEL26_IRQHandler()
{
	if (ICU->IELSR26 & ICU_IELSR26_IR_Msk)
	{
		__Re01IntHandler(26);

		ICU->IELSR26 &= ~ICU_IELSR26_IR_Msk;
	}
	NVIC_ClearPendingIRQ(IEL26_IRQn);
}

__WEAK void IEL27_IRQHandler()
{
	if (ICU->IELSR27 & ICU_IELSR27_IR_Msk)
	{
		__Re01IntHandler(27);

		ICU->IELSR27 &= ~ICU_IELSR27_IR_Msk;
	}
	NVIC_ClearPendingIRQ(IEL27_IRQn);
}

__WEAK void IEL28_IRQHandler()
{
	if (ICU->IELSR28 & ICU_IELSR28_IR_Msk)
	{
		__Re01IntHandler(28);

		ICU->IELSR28 &= ~ICU_IELSR28_IR_Msk;
	}
	NVIC_ClearPendingIRQ(IEL28_IRQn);
}

__WEAK void IEL29_IRQHandler()
{
	if (ICU->IELSR29 & ICU_IELSR29_IR_Msk)
	{
		__Re01IntHandler(29);

		ICU->IELSR29 &= ~ICU_IELSR29_IR_Msk;
	}
	NVIC_ClearPendingIRQ(IEL29_IRQn);
}

__WEAK void IEL30_IRQHandler()
{
	if (ICU->IELSR30 & ICU_IELSR30_IR_Msk)
	{
		__Re01IntHandler(30);

		ICU->IELSR30 &= ~ICU_IELSR30_IR_Msk;
	}
	NVIC_ClearPendingIRQ(IEL30_IRQn);
}

__WEAK void IEL31_IRQHandler()
{
	if (ICU->IELSR31 & ICU_IELSR31_IR_Msk)
	{
		__Re01IntHandler(31);

		ICU->IELSR31 &= ~ICU_IELSR31_IR_Msk;
	}
	NVIC_ClearPendingIRQ(IEL31_IRQn);
}

} // extern "C"
