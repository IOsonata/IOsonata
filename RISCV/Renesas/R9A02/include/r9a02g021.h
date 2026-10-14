/**-------------------------------------------------------------------------
@file	r9a02g021.h

@brief	Renesas R9A02G021 register map

IOsonata register definitions for the R9A02G021 (Renesas RISC-V core, CLIC,
48 MHz). Renesas provides no CMSIS device header for this part. Every
address and bit below is taken from the R9A02G021 User's Manual: Hardware,
R01UH1036EJ0110 Rev.1.10 (Feb 29, 2024). The section number is given next to
each block.

Only the blocks used by the IOsonata port are described field by field. The
other blocks have their base address only.

The I/O port, SAU and TAU blocks follow the manual layout, not the RA family
layout. The serial and timer blocks are SAU, UARTA, IICA, TAU and TML32;
there is no SCI, SPI, RIIC, AGT or GPT on this part.

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
#ifndef __R9A02G021_H__
#define __R9A02G021_H__

#include <stdint.h>

#ifndef __I
#ifdef __cplusplus
#define __I		volatile
#else
#define __I		volatile const
#endif
#endif
#ifndef __O
#define __O		volatile
#endif
#ifndef __IO
#define __IO	volatile
#endif

// Core intrinsics, as the Arm CMSIS ones used by the shared examples. The
// core has no event wait, __WFE waits for an interrupt as __WFI (2.7.3).
#ifndef __NOP
#define __NOP()			__asm volatile ("nop")
#endif
#ifndef __WFI
#define __WFI()			__asm volatile ("wfi" ::: "memory")
#endif
#ifndef __WFE
#define __WFE()			__asm volatile ("wfi" ::: "memory")
#endif

// Memory map, manual 4.1 and 34.1

#define R9A02_FLASH_BASE				0x00000000UL	//!< Code flash
#define R9A02_FLASH_SIZE				0x00020000UL	//!< 128 KB on every part
#define R9A02_FLASH_BLOCK_SIZE			0x800UL			//!< 2 KB erase block (datasheet 2.9.1)
#define R9A02_DATA_FLASH_BASE			0x40100000UL
#define R9A02_DATA_FLASH_SIZE			0x00001000UL	//!< 4 KB
#define R9A02_SRAM_ECC_BASE				0x20000000UL	//!< SRAM0 ECC area
#define R9A02_SRAM_ECC_SIZE				0x00001000UL	//!< 4 KB
#define R9A02_SRAM_BASE					0x20004000UL	//!< SRAM0 parity area
#define R9A02_SRAM_SIZE					0x00003000UL	//!< 12 KB

// Option-setting memory, manual 6.2 and 13.6
// The program flash area holds OFS0, OFS1, the flash read protection words
// and OSIS. The configuration area (SECS, AWS, UIDS) is written with the
// configuration setting command, not by programming.

#define R9A02_OFS0_ADDR					0x00000400UL
#define R9A02_OFS1_ADDR					0x00000404UL
#define R9A02_FLRPROTS_ADDR				0x00000418UL
#define R9A02_FLRPROTE_ADDR				0x0000041CUL
#define R9A02_FLRPROTAC_ADDR			0x00000438UL
#define R9A02_OPTION_END_ADDR			0x0000043CUL	//!< End of the 0x400 option block
#define R9A02_OSIS_ADDR					0x00000800UL	//!< 16 bytes, all 0xFF disables ID code protection
#define R9A02_OSIS_END_ADDR				0x00000810UL
#define R9A02_SECS_ADDR					0x01010008UL
#define R9A02_AWS_ADDR					0x01010010UL

#define R9A02_OFS0_IWDTSTRT				(1UL << 1)		//!< 1: IWDT stopped after reset
#define R9A02_OFS0_WDTSTRT				(1UL << 17)		//!< 1: WDT stopped after reset

#define R9A02_OFS1_LVDAS				(1UL << 2)		//!< 1: voltage monitor 0 reset disabled
#define R9A02_OFS1_HOCOEN				(1UL << 8)		//!< 0: HOCO starts at reset
#define R9A02_OFS1_HOCOFRQ1_Pos			12U
#define R9A02_OFS1_HOCOFRQ1_Msk			(7UL << R9A02_OFS1_HOCOFRQ1_Pos)
#define R9A02_OFS1_HOCOFRQ1_24MHZ		(0UL << R9A02_OFS1_HOCOFRQ1_Pos)
#define R9A02_OFS1_HOCOFRQ1_32MHZ		(2UL << R9A02_OFS1_HOCOFRQ1_Pos)
#define R9A02_OFS1_HOCOFRQ1_48MHZ		(4UL << R9A02_OFS1_HOCOFRQ1_Pos)
#define R9A02_OFS1_ICSATS				(1UL << 31)		//!< 1: clock supply type A, ICLK and PCLKB set separately

// Oscillators, manual 8.1 and datasheet 1.1

#define R9A02_HOCO_24MHZ				24000000UL
#define R9A02_HOCO_32MHZ				32000000UL
#define R9A02_HOCO_48MHZ				48000000UL
#define R9A02_MOCO_FREQ					8000000UL
#define R9A02_LOCO_FREQ					32768UL
#define R9A02_SOSC_FREQ					32768UL
#define R9A02_IWDTLOCO_FREQ				15000UL
#define R9A02_EXTAL_FREQ_MAX			20000000UL
#define R9A02_ICLK_MAX					48000000UL
#define R9A02_ICLK_NOWAIT_MAX			32000000UL		//!< ICLK above this needs MEMWAIT and FLDWAIT1
#define R9A02_ICLK_MIDDLE_MAX			24000000UL		//!< Middle-speed mode limit
#define R9A02_ICLK_LOW_MAX				1000000UL		//!< Low-speed mode limit

// Peripheral base addresses, manual register descriptions

#define R9A02_SRAMC_BASE				0x40002000UL	//!< SRAM control, 34.2
#define R9A02_BUS_BASE					0x40003000UL	//!< 13.5
#define R9A02_DTC_BASE					0x40005400UL	//!< 14.2
#define R9A02_ICU_BASE					0x40006000UL	//!< 12.2
#define R9A02_CPU_AUX_BASE				0x4001A000UL	//!< 2.5.8
#define R9A02_CPU_DBG_BASE				0x4001B000UL	//!< 2.5.9
#define R9A02_SYSC_BASE					0x4001E000UL	//!< 5, 7, 8, 10, 11
#define R9A02_PORT_BASE					0x40040000UL	//!< 16.2.1
#define R9A02_PFS_BASE					0x40040800UL	//!< 16.2.5
#define R9A02_ELC_BASE					0x40041000UL	//!< 15
#define R9A02_WDT_BASE					0x40044200UL	//!< 21
#define R9A02_IWDT_BASE					0x40044400UL	//!< 22
#define R9A02_CAC_BASE					0x40044600UL	//!< 9
#define R9A02_MSTP_BASE					0x40047000UL	//!< 10.2.3
#define R9A02_DAC8_BASE					0x4005E000UL	//!< 30
#define R9A02_CRC_BASE					0x40074000UL	//!< 27
#define R9A02_KINT_BASE					0x40080000UL	//!< 17
#define R9A02_DOC_BASE					0x40085F00UL	//!< 33
#define R9A02_PORGA_BASE				0x40091000UL	//!< Port and peripheral glue registers
#define R9A02_TRNG_BASE					0x40091100UL	//!< 28
#define R9A02_CMP_BASE					0x40091200UL	//!< 31
#define R9A02_RTC_BASE					0x40092000UL	//!< 20
#define R9A02_REMC_BASE					0x40092100UL	//!< 26
#define R9A02_TML32_BASE				0x40092200UL	//!< 19.2
#define R9A02_IICA_BASE					0x40093000UL	//!< 24
#define R9A02_SAU0_BASE					0x40094000UL	//!< 23.3
#define R9A02_SAU1_BASE					0x40094100UL	//!< 23.3
#define R9A02_TAU_BASE					0x40095000UL	//!< 18.2
#define R9A02_UARTA_BASE				0x40096000UL	//!< 25.2
#define R9A02_ADC12_BASE				0x4009C000UL	//!< 29
#define R9A02_FLCN_BASE					0x407EC000UL	//!< Flash control, 8.2.4 and 35
#define R9A02_CLIC_BASE					0xE2000000UL	//!< 2.5.3
#define R9A02_IMT_BASE					0xE6000000UL	//!< Machine timer, 2.5.4

// Generic register access by address

#define R9A02_REG8(Addr)				(*(__IO uint8_t *)(Addr))
#define R9A02_REG16(Addr)				(*(__IO uint16_t *)(Addr))
#define R9A02_REG32(Addr)				(*(__IO uint32_t *)(Addr))

// SYSC: clock, low power, reset and write protection, manual 5, 8, 10, 11

#define R9A02_SYSC_SBYCR				R9A02_REG16(R9A02_SYSC_BASE + 0x00C)
#define R9A02_SYSC_MSTPCRA				R9A02_REG32(R9A02_SYSC_BASE + 0x01C)
#define R9A02_SYSC_SCKDIVCR				R9A02_REG32(R9A02_SYSC_BASE + 0x020)
#define R9A02_SYSC_SCKSCR				R9A02_REG8(R9A02_SYSC_BASE + 0x026)
#define R9A02_SYSC_MEMWAIT				R9A02_REG8(R9A02_SYSC_BASE + 0x031)
#define R9A02_SYSC_MOSCCR				R9A02_REG8(R9A02_SYSC_BASE + 0x032)
#define R9A02_SYSC_HOCOCR				R9A02_REG8(R9A02_SYSC_BASE + 0x036)
#define R9A02_SYSC_HOCOCR2				R9A02_REG8(R9A02_SYSC_BASE + 0x037)
#define R9A02_SYSC_MOCOCR				R9A02_REG8(R9A02_SYSC_BASE + 0x038)
#define R9A02_SYSC_OSCSF				R9A02_REG8(R9A02_SYSC_BASE + 0x03C)
#define R9A02_SYSC_CKOCR				R9A02_REG8(R9A02_SYSC_BASE + 0x03E)
#define R9A02_SYSC_LPOPT				R9A02_REG8(R9A02_SYSC_BASE + 0x04C)
#define R9A02_SYSC_OSMCR				R9A02_REG8(R9A02_SYSC_BASE + 0x054)
#define R9A02_SYSC_MOCOUTCR				R9A02_REG8(R9A02_SYSC_BASE + 0x061)
#define R9A02_SYSC_HOCOUTCR				R9A02_REG8(R9A02_SYSC_BASE + 0x062)
#define R9A02_SYSC_OPCCR				R9A02_REG8(R9A02_SYSC_BASE + 0x0A0)
#define R9A02_SYSC_SOPCCR				R9A02_REG8(R9A02_SYSC_BASE + 0x0AA)
#define R9A02_SYSC_SNZCR				R9A02_REG8(R9A02_SYSC_BASE + 0x092)
#define R9A02_SYSC_SNZEDCR0				R9A02_REG8(R9A02_SYSC_BASE + 0x094)
#define R9A02_SYSC_SNZEDCR1				R9A02_REG8(R9A02_SYSC_BASE + 0x095)
#define R9A02_SYSC_SNZREQCR0			R9A02_REG32(R9A02_SYSC_BASE + 0x098)
#define R9A02_SYSC_PSMCR				R9A02_REG8(R9A02_SYSC_BASE + 0x09F)
#define R9A02_SYSC_RSTSR1				R9A02_REG16(R9A02_SYSC_BASE + 0x0C0)
#define R9A02_SYSC_PRCR					R9A02_REG16(R9A02_SYSC_BASE + 0x3FE)
#define R9A02_SYSC_RSTSR0				R9A02_REG8(R9A02_SYSC_BASE + 0x410)
#define R9A02_SYSC_RSTSR2				R9A02_REG8(R9A02_SYSC_BASE + 0x411)
#define R9A02_SYSC_SOSCCR				R9A02_REG8(R9A02_SYSC_BASE + 0x480)
#define R9A02_SYSC_SOMCR				R9A02_REG8(R9A02_SYSC_BASE + 0x481)
#define R9A02_SYSC_LOCOCR				R9A02_REG8(R9A02_SYSC_BASE + 0x490)
#define R9A02_SYSC_LOCOUTCR				R9A02_REG8(R9A02_SYSC_BASE + 0x492)

#define R9A02_SCKDIVCR_PCKB_Pos			8U				//!< PCLKB = source / 2^PCKB
#define R9A02_SCKDIVCR_PCKB_Msk			(7UL << R9A02_SCKDIVCR_PCKB_Pos)
#define R9A02_SCKDIVCR_ICK_Pos			24U				//!< ICLK = source / 2^ICK
#define R9A02_SCKDIVCR_ICK_Msk			(7UL << R9A02_SCKDIVCR_ICK_Pos)
#define R9A02_SCKDIVCR_FIXED			0x4UL			//!< Bits [2:0] must be written 100b
#define R9A02_SCKDIVCR_DIV_MAX			6U				//!< Divide by 64

#define R9A02_SCKSCR_CKSEL_Msk			7U
#define R9A02_SCKSCR_HOCO				0U
#define R9A02_SCKSCR_MOCO				1U
#define R9A02_SCKSCR_LOCO				2U
#define R9A02_SCKSCR_EXTAL				3U
#define R9A02_SCKSCR_SOSC				4U

#define R9A02_MEMWAIT_MEMWAIT			(1U << 0)
#define R9A02_MOSCCR_MOSTP				(1U << 0)
#define R9A02_HOCOCR_HCSTP				(1U << 0)
#define R9A02_HOCOCR2_HCFRQ1_Pos		3U
#define R9A02_HOCOCR2_HCFRQ1_Msk		(7U << R9A02_HOCOCR2_HCFRQ1_Pos)
#define R9A02_HOCOCR2_24MHZ				(0U << R9A02_HOCOCR2_HCFRQ1_Pos)
#define R9A02_HOCOCR2_32MHZ				(2U << R9A02_HOCOCR2_HCFRQ1_Pos)
#define R9A02_HOCOCR2_48MHZ				(4U << R9A02_HOCOCR2_HCFRQ1_Pos)
#define R9A02_MOCOCR_MCSTP				(1U << 0)
#define R9A02_OSCSF_HOCOSF				(1U << 0)
#define R9A02_LPOPT_FIXED				(1U << 6)		//!< Bit 6 must be written 1
#define R9A02_OSMCR_WUTMMCK0			(1U << 0)		//!< 1: RTC, TML32, UARTA, REMC run from LOCO
#define R9A02_OPCCR_OPCM_Msk			3U
#define R9A02_OPCCR_HIGH_SPEED			0U
#define R9A02_OPCCR_MIDDLE_SPEED		1U
#define R9A02_OPCCR_LOW_SPEED			3U
#define R9A02_OPCCR_OPCMTSF				(1U << 4)
#define R9A02_SOPCCR_SOPCM				(1U << 0)
#define R9A02_SOPCCR_SOPCMTSF			(1U << 4)
#define R9A02_SOSCCR_SOSTP				(1U << 0)
#define R9A02_LOCOCR_LCSTP				(1U << 0)
#define R9A02_SBYCR_SSBY				(1U << 15)		//!< 1: WFI enters Software Standby (10.2.1)

// Snooze, manual 10.2.8 to 10.2.11. PRC1 protected, not written in Snooze.
#define R9A02_SNZCR_SNZDTCEN			(1U << 1)		//!< DTC and SRAM run in Snooze
#define R9A02_SNZCR_RXD0REQEN			(1U << 2)		//!< RXD0 or SCK00 edge requests Snooze
#define R9A02_SNZCR_RXD2REQEN			(1U << 3)		//!< RXD2 or SCK20 edge requests Snooze
#define R9A02_SNZCR_REMC0REQEN			(1U << 4)		//!< RIN0 edge requests Snooze
#define R9A02_SNZCR_SNZE				(1U << 7)		//!< Snooze enabled
#define R9A02_SNZEDCR0_DTCZRED			(1U << 1)		//!< Last DTC transfer ends Snooze
#define R9A02_SNZEDCR0_DTCNZRED			(1U << 2)		//!< Each other DTC transfer ends Snooze
#define R9A02_SNZEDCR0_ADNCRED			(1U << 3)		//!< ADC12 compare mismatch ends Snooze
#define R9A02_SNZEDCR1_TMLOIED			(1U << 0)		//!< TML32_OUTI ends Snooze
#define R9A02_SNZEDCR1_REMCNCRED		(1U << 1)		//!< REMC without interrupt ends Snooze
#define R9A02_SNZEDCR1_SAU0NCRED		(1U << 4)		//!< SAU0 error without interrupt ends Snooze
#define R9A02_SNZEDCR1_SAU1NCRED		(1U << 5)		//!< SAU1 error without interrupt ends Snooze
#define R9A02_SNZREQCR0_IRQ(n)			(1UL << (n))	//!< IRQ0 to IRQ7 pins request Snooze
#define R9A02_SNZREQCR0_KEY				(1UL << 17)
#define R9A02_SNZREQCR0_RTC				(1UL << 24)
#define R9A02_SNZREQCR0_TML32			(1UL << 26)

#define R9A02_PRCR_KEY					0xA500U
#define R9A02_PRCR_PRC0					(1U << 0)		//!< Clock generation registers
#define R9A02_PRCR_PRC1					(1U << 1)		//!< Low power mode registers, also set for MSTPCRx (figure 10.11)
#define R9A02_PRCR_PRC3					(1U << 3)		//!< LVD registers

#define R9A02_FLCN_FLDWAITR				R9A02_REG8(R9A02_FLCN_BASE + 0x3FC4)
#define R9A02_FLCN_PFBER				R9A02_REG8(R9A02_FLCN_BASE + 0x3FC8)	//!< 35.3.2
#define R9A02_FLCN_FENTRYR				R9A02_REG16(R9A02_FLCN_BASE + 0x3FB0)	//!< Not 0 in P/E mode
#define R9A02_FLCN_TSCDR				R9A02_REG16(R9A02_FLCN_BASE + 0x0228)	//!< Temperature sensor calibration, 32.2.1
#define R9A02_TSCDR_CAL140_Msk			0x0FFFU			//!< 12-bit result at Tj 140 C, VCC 3.3 V
#define R9A02_FLDWAITR_FLDWAIT1			(1U << 0)
#define R9A02_PFBER_PFBE				(1U << 0)		//!< Code flash prefetch buffer on (35.4)

// Module stop, manual 10.2.2 to 10.2.5
// Write 1 to every reserved bit. MSTPCRA is in SYSC, B to D in MSTP.

#define R9A02_MSTP_MSTPCRB				R9A02_REG32(R9A02_MSTP_BASE + 0x000)
#define R9A02_MSTP_MSTPCRC				R9A02_REG32(R9A02_MSTP_BASE + 0x004)
#define R9A02_MSTP_MSTPCRD				R9A02_REG32(R9A02_MSTP_BASE + 0x008)
#define R9A02_MSTP_LSMRWDIS				R9A02_REG16(R9A02_MSTP_BASE + 0x00C)

#define R9A02_MSTPCRA_DTC				(1UL << 22)
#define R9A02_MSTPCRB_IICA1				(1UL << 8)
#define R9A02_MSTPCRB_IICA0				(1UL << 9)
#define R9A02_MSTPCRB_UARTA				(1UL << 17)		//!< UARTA0 and UARTA1
#define R9A02_MSTPCRB_SAU1				(1UL << 20)
#define R9A02_MSTPCRB_SAU0				(1UL << 21)
#define R9A02_MSTPCRC_CAC				(1UL << 0)
#define R9A02_MSTPCRC_CRC				(1UL << 1)
#define R9A02_MSTPCRC_DOC				(1UL << 13)
#define R9A02_MSTPCRC_ELC				(1UL << 14)
#define R9A02_MSTPCRC_REMC				(1UL << 19)
#define R9A02_MSTPCRC_DAC8				(1UL << 22)		//!< Same value as MSTPCRD.MSTPD20
#define R9A02_MSTPCRC_TRNG				(1UL << 28)
#define R9A02_MSTPCRD_TML32				(1UL << 9)
#define R9A02_MSTPCRD_TAU				(1UL << 10)
#define R9A02_MSTPCRD_ADC12				(1UL << 16)
#define R9A02_MSTPCRD_DAC8				(1UL << 20)		//!< Same value as MSTPCRC.MSTPC22
#define R9A02_MSTPCRD_CMP				(1UL << 28)

// CPU auxiliary registers, manual 2.5.8

#define R9A02_CPU_AUX_MACTCR			R9A02_REG32(R9A02_CPU_AUX_BASE + 0x000)
#define R9A02_CPU_AUX_SWRCR				R9A02_REG32(R9A02_CPU_AUX_BASE + 0x100)
#define R9A02_CPU_AUX_NMIADDR			R9A02_REG32(R9A02_CPU_AUX_BASE + 0x200)

#define R9A02_MACTCR_ENABLE				(1UL << 0)		//!< Clock supplied to the machine timer
#define R9A02_MACTCR_CLOCKSOURCE		(1UL << 1)		//!< 0: MTCLK (LOCO), 1: CPU clock
#define R9A02_SWRCR_SYSRESETREQ			(1UL << 0)

// Machine timer, manual 2.5.4. Counts MTCLK or ICLK, see MACTCR.
// ICLK must be at least twice MTCLK (2.6).

typedef struct {
	__IO uint32_t MTIME_LO;			//!< 0x000
	__IO uint32_t MTIME_HI;			//!< 0x004
	__IO uint32_t MTIMECMP_LO;		//!< 0x008
	__IO uint32_t MTIMECMP_HI;		//!< 0x00C
	uint8_t RESERVED0[0xFEC];
	__IO uint32_t MSIP;				//!< 0xFFC
} R9A02_IMT_Type;

#define R9A02_IMT						((R9A02_IMT_Type *)R9A02_IMT_BASE)

// CLIC, manual 2.5.3. Use with RISCV/include/clic.h
// IDs 3 and 7 are the machine software and timer interrupts, IDs 19 to 50
// are the ICU outputs IELSR0 to IELSR31 (12.3.1). Trigger and mode are
// fixed (rising edge, machine mode). lvl_prio is clicintctl[7:4].

#define R9A02_CLIC_CTRL_BASE			(R9A02_CLIC_BASE + 0x1000UL)
#define R9A02_CLIC_CLICCFG				R9A02_REG8(R9A02_CLIC_BASE + 0x0000)
#define R9A02_CLIC_CLICINFO				R9A02_REG32(R9A02_CLIC_BASE + 0x0004)
#define R9A02_CLIC_INT_COUNT			51U
#define R9A02_CLIC_CTLBITS				4U				//!< Implemented clicintctl bits
#define R9A02_CLIC_PRIO_MAX				15U				//!< 16 levels or priorities
#define R9A02_CLIC_ID_MSIP				3U
#define R9A02_CLIC_ID_MTIP				7U
#define R9A02_CLIC_ID_IELSR0			19U
#define R9A02_CLICCFG_NLBITS_Pos		1U
#define R9A02_CLICCFG_NLBITS_Msk		(0xFU << R9A02_CLICCFG_NLBITS_Pos)

// Exception codes, manual 2.7.5.1 (mcause with INTERRUPT = 0)

#define R9A02_EXC_ILLEGAL_INSTR			2U
#define R9A02_EXC_BREAKPOINT			3U
#define R9A02_EXC_LOAD_MISALIGN			4U
#define R9A02_EXC_STORE_MISALIGN		6U
#define R9A02_EXC_ECALL_M				11U
#define R9A02_EXC_STACK_OVERFLOW		24U

// ICU, manual 12.2

typedef struct {
	__IO uint8_t IRQCR[8];			//!< 0x000 IRQ0 to IRQ7 pin control
	uint8_t RESERVED0[0xF8];
	__IO uint8_t NMICR;				//!< 0x100
	uint8_t RESERVED1[0x1F];
	__IO uint16_t NMIER;			//!< 0x120
	uint8_t RESERVED2[0x0E];
	__IO uint16_t NMICLR;			//!< 0x130
	uint8_t RESERVED3[0x0E];
	__I uint16_t NMISR;				//!< 0x140
	uint8_t RESERVED4[0x5E];
	__IO uint32_t WUPEN0;			//!< 0x1A0
	__IO uint32_t WUPEN1;			//!< 0x1A4
	uint8_t RESERVED5[0x18];
	__IO uint8_t IELEN;				//!< 0x1C0
	uint8_t RESERVED6[0x3F];
	__IO uint16_t SELSR0;			//!< 0x200
	uint8_t RESERVED7[0xFE];
	__IO uint32_t IELSR[32];		//!< 0x300
} R9A02_ICU_Type;

#define R9A02_ICU						((R9A02_ICU_Type *)R9A02_ICU_BASE)
#define R9A02_ICU_IELSR_CNT				32U
#define R9A02_ICU_GROUP_CNT				8U				//!< IELSRn group is n % 8

#define R9A02_IRQCR_IRQMD_Msk			3U
#define R9A02_IRQCR_IRQMD_FALLING		0U
#define R9A02_IRQCR_IRQMD_RISING		1U
#define R9A02_IRQCR_IRQMD_BOTH			2U
#define R9A02_IRQCR_IRQMD_LOW			3U
#define R9A02_IRQCR_FCLKSEL_Pos			4U
#define R9A02_IRQCR_FLTEN				(1U << 7)

// Software Standby wake-up enables, 12.2.7 and 12.2.8
#define R9A02_WUPEN0_IRQ(n)				(1UL << (n))	//!< IRQ0 to IRQ7 pins
#define R9A02_WUPEN0_IWDT				(1UL << 16)
#define R9A02_WUPEN0_KEY				(1UL << 17)
#define R9A02_WUPEN0_LVD1				(1UL << 18)
#define R9A02_WUPEN0_LVD2				(1UL << 19)
#define R9A02_WUPEN0_RTC				(1UL << 24)
#define R9A02_WUPEN0_TML32				(1UL << 26)
#define R9A02_WUPEN1_IICA0				(1UL << 0)
#define R9A02_WUPEN1_IICA1				(1UL << 1)
#define R9A02_WUPEN1_UARTA_RX_ENDI0		(1UL << 2)
#define R9A02_WUPEN1_UARTA_RX_ERI0		(1UL << 3)
#define R9A02_WUPEN1_UARTA_RX_ENDI1		(1UL << 4)
#define R9A02_WUPEN1_UARTA_RX_ERI1		(1UL << 5)
#define R9A02_WUPEN1_COMP_DET0			(1UL << 6)
#define R9A02_WUPEN1_COMP_DET1			(1UL << 7)

#define R9A02_IELSR_IELS_Msk			0x1FUL
#define R9A02_IELSR_IR					(1UL << 16)		//!< Write 0 to clear, writing 1 is prohibited
#define R9A02_IELSR_DTCE				(1UL << 24)

// ICU event numbers, manual table 12.4. The value written to IELSRn.IELS
// depends on the group of IELSRn, table 12.7. interrupt_r9a02.cpp holds the
// mapping.

#define R9A02_EVT_PORT_IRQ0				0x01U
#define R9A02_EVT_PORT_IRQ1				0x02U
#define R9A02_EVT_PORT_IRQ2				0x03U
#define R9A02_EVT_PORT_IRQ3				0x04U
#define R9A02_EVT_PORT_IRQ4				0x05U
#define R9A02_EVT_PORT_IRQ5				0x06U
#define R9A02_EVT_PORT_IRQ6				0x07U
#define R9A02_EVT_PORT_IRQ7				0x08U
#define R9A02_EVT_DTC_COMPLETE			0x09U
#define R9A02_EVT_ICU_SNZCANCEL			0x0BU
#define R9A02_EVT_FCU_FRDYI				0x0CU
#define R9A02_EVT_LVD_LVD1				0x0DU
#define R9A02_EVT_LVD_LVD2				0x0EU
#define R9A02_EVT_SYSTEM_SNZREQ			0x10U
#define R9A02_EVT_IWDT_NMIUNDF			0x11U
#define R9A02_EVT_WDT_NMIUNDF			0x12U
#define R9A02_EVT_RTC_ALM_OR_PRD		0x13U
#define R9A02_EVT_IIC0_ENDI				0x2BU
#define R9A02_EVT_IIC1_ENDI				0x2CU
#define R9A02_EVT_KEY_INTKR				0x30U
#define R9A02_EVT_CAC_FERRI				0x31U
#define R9A02_EVT_CAC_MENDI				0x32U
#define R9A02_EVT_CAC_OVFI				0x33U
#define R9A02_EVT_IOPORT_GROUP1			0x34U
#define R9A02_EVT_IOPORT_GROUP2			0x35U
#define R9A02_EVT_TRNG_RD_REQ			0x38U
#define R9A02_EVT_ELC_SWEVT0			0x39U
#define R9A02_EVT_ELC_SWEVT1			0x3AU
#define R9A02_EVT_SAU0_ENDI0			0x3BU
#define R9A02_EVT_SAU0_ENDI1			0x3CU
#define R9A02_EVT_SAU0_ENDI2			0x3DU
#define R9A02_EVT_SAU0_ENDI3			0x3EU
#define R9A02_EVT_SAU1_ENDI0			0x3FU
#define R9A02_EVT_SAU1_ENDI1			0x40U
#define R9A02_EVT_SAU0_INTSRE0			0x43U
#define R9A02_EVT_SAU0_INTSRE1			0x44U
#define R9A02_EVT_SAU1_INTSRE2			0x45U
#define R9A02_EVT_TAU0_ENDI0			0x47U
#define R9A02_EVT_TAU0_ENDI1			0x48U
#define R9A02_EVT_TAU0_ENDI2			0x49U
#define R9A02_EVT_TAU0_ENDI3			0x4AU
#define R9A02_EVT_TAU0_ENDI4			0x4BU
#define R9A02_EVT_TAU0_ENDI5			0x4CU
#define R9A02_EVT_TAU0_ENDI6			0x4DU
#define R9A02_EVT_TAU0_ENDI7			0x4EU
#define R9A02_EVT_TAU0_MODE8_ENDI1		0x4FU
#define R9A02_EVT_TAU0_MODE8_ENDI3		0x50U
#define R9A02_EVT_TML32_OUTI			0x5BU
#define R9A02_EVT_REMC_OUTI				0x5CU
#define R9A02_EVT_UARTA_TX_ENDI0		0x5DU
#define R9A02_EVT_UARTA_RX_ENDI0		0x5EU
#define R9A02_EVT_UARTA_RX_ERI0			0x5FU
#define R9A02_EVT_UARTA_TX_ENDI1		0x60U
#define R9A02_EVT_UARTA_RX_ENDI1		0x61U
#define R9A02_EVT_UARTA_RX_ERI1			0x62U
#define R9A02_EVT_COMP_DET0				0x63U
#define R9A02_EVT_COMP_DET1				0x64U
#define R9A02_EVT_ADC_ENDI				0x65U
#define R9A02_EVT_DOC_DOPCI				0x69U
#define R9A02_EVT_MAX					0x69U

// I/O ports, manual 16.2.1 to 16.2.4
// In the 32-bit PCNTRn view the low field is bits [15:0]. The 16-bit view
// puts the high field first: PODR at +0 and PDR at +2, EIDR at +4 and
// PIDR at +6, PORR at +8 and POSR at +10, EORR at +12 and EOSR at +14.

typedef struct {
	union {
		__IO uint32_t PCNTR1;		//!< 0x00 PDR [15:0], PODR [31:16]
		struct {
			__IO uint16_t PODR;		//!< 0x00 Output data
			__IO uint16_t PDR;		//!< 0x02 Direction, 1 = output
		};
	};
	union {
		__I uint32_t PCNTR2;		//!< 0x04 PIDR [15:0], EIDR [31:16]
		struct {
			__I uint16_t EIDR;		//!< 0x04 Event input data
			__I uint16_t PIDR;		//!< 0x06 Pin state
		};
	};
	union {
		__O uint32_t PCNTR3;		//!< 0x08 POSR [15:0], PORR [31:16]
		struct {
			__O uint16_t PORR;		//!< 0x08 Write 1 to drive low
			__O uint16_t POSR;		//!< 0x0A Write 1 to drive high
		};
	};
	union {
		__IO uint32_t PCNTR4;		//!< 0x0C EOSR [15:0], EORR [31:16], ports 1 and 2
		struct {
			__IO uint16_t EORR;		//!< 0x0C
			__IO uint16_t EOSR;		//!< 0x0E
		};
	};
} R9A02_PORT_Type;

#define R9A02_PORT_COUNT				5U				//!< PORT0 to PORT4
#define R9A02_PORT_PIN_COUNT			16U
#define R9A02_PORT(n)					((R9A02_PORT_Type *)(R9A02_PORT_BASE + ((uint32_t)(n) << 5)))

// Valid pins per port on the 48-pin part, datasheet 1.5
#define R9A02_PORT0_PIN_MSK				0x0FCFU			//!< P000 to P003, P006 to P011
#define R9A02_PORT1_PIN_MSK				0x0FFFU			//!< P100 to P111
#define R9A02_PORT2_PIN_MSK				0x00FFU			//!< P200 (input only) to P207
#define R9A02_PORT3_PIN_MSK				0x00FFU			//!< P300 to P307
#define R9A02_PORT4_PIN_MSK				0x000FU			//!< P400 to P403

// Pin function select, manual 16.2.5 and 16.2.6

#define R9A02_PFS(Port, Pin)			R9A02_REG32(R9A02_PFS_BASE + ((uint32_t)(Port) << 6) + ((uint32_t)(Pin) << 2))
#define R9A02_PFS_PWPR					R9A02_REG8(R9A02_PFS_BASE + 0x503)
#define R9A02_PFS_PRWCNTR				R9A02_REG8(R9A02_PFS_BASE + 0x50F)

#define R9A02_PFS_PODR					(1UL << 0)
#define R9A02_PFS_PIDR					(1UL << 1)
#define R9A02_PFS_PDR					(1UL << 2)
#define R9A02_PFS_PCR					(1UL << 4)		//!< Input pull-up
#define R9A02_PFS_NCODR					(1UL << 6)		//!< N-channel open drain
#define R9A02_PFS_DSCR_Pos				10U				//!< Output current control, P100, P302, P303 only
#define R9A02_PFS_DSCR_Msk				(3UL << R9A02_PFS_DSCR_Pos)
#define R9A02_PFS_EOFR_Pos				12U				//!< Port group event edge, ports 1 and 2
#define R9A02_PFS_EOFR_Msk				(3UL << R9A02_PFS_EOFR_Pos)
#define R9A02_PFS_EOFR_RISING			(1UL << R9A02_PFS_EOFR_Pos)
#define R9A02_PFS_EOFR_FALLING			(2UL << R9A02_PFS_EOFR_Pos)
#define R9A02_PFS_EOFR_BOTH				(3UL << R9A02_PFS_EOFR_Pos)
#define R9A02_PFS_ISEL					(1UL << 14)		//!< IRQn input
#define R9A02_PFS_ASEL					(1UL << 15)		//!< Analog input
#define R9A02_PFS_PMR					(1UL << 16)		//!< Peripheral function
#define R9A02_PFS_PSEL_Pos				24U
#define R9A02_PFS_PSEL_Msk				(0x1FUL << R9A02_PFS_PSEL_Pos)

#define R9A02_PWPR_PFSWE				(1U << 6)
#define R9A02_PWPR_B0WI					(1U << 7)

// PSEL peripheral codes, manual 16.6. The IOsonata PinOp of a pin is this
// code, IOPINOP_FUNC31 selects the analog function (ASEL).

#define R9A02_PSEL_UARTA				0x06U
#define R9A02_PSEL_IICA					0x07U
#define R9A02_PSEL_CLKOUT_CMP			0x09U
#define R9A02_PSEL_CAC					0x0AU
#define R9A02_PSEL_REMC					0x0EU
#define R9A02_PSEL_SAU					0x11U
#define R9A02_PSEL_KINT					0x12U
#define R9A02_PSEL_TAU					0x19U
#define R9A02_PSEL_RTC					0x1BU

// Port and peripheral glue registers (PORGA), manual 16.2.8, 18.2, 23.3

#define R9A02_PORGA_SNFEN				R9A02_REG8(R9A02_PORGA_BASE + 0x00)
#define R9A02_PORGA_TNFEN				R9A02_REG8(R9A02_PORGA_BASE + 0x01)
#define R9A02_PORGA_ISC					R9A02_REG8(R9A02_PORGA_BASE + 0x03)
#define R9A02_PORGA_TIS0				R9A02_REG8(R9A02_PORGA_BASE + 0x04)
#define R9A02_PORGA_ULBS				R9A02_REG8(R9A02_PORGA_BASE + 0x09)
#define R9A02_PORGA_CCDE				R9A02_REG8(R9A02_PORGA_BASE + 0x48)
#define R9A02_PORGA_CCTRM				R9A02_REG8(R9A02_PORGA_BASE + 0x81)

#define R9A02_SNFEN_SNFEN00				(1U << 0)		//!< RxD0 noise filter
#define R9A02_SNFEN_SNFEN10				(1U << 2)		//!< RxD1 noise filter
#define R9A02_SNFEN_SNFEN20				(1U << 4)		//!< RxD2 noise filter
#define R9A02_ISC_ISC43_Msk				(3U << 3)		//!< SPI00 clock source, 0 for UART or I2C on channel 0
#define R9A02_ISC_ISC76_Msk				(3U << 6)		//!< SPI01 clock source, 0 for UART or I2C on channel 1
#define R9A02_ULBS_UART0				(1U << 0)		//!< UART0 TX looped back to RX
#define R9A02_ULBS_UART1				(1U << 1)
#define R9A02_ULBS_UART2				(1U << 2)
#define R9A02_ULBS_UARTA0				(1U << 4)
#define R9A02_ULBS_UARTA1				(1U << 5)

// Serial array unit, manual 23.3
// UART0: SAU0 channels 0 (TX) and 1 (RX). UART1: SAU0 channels 2 and 3.
// UART2: SAU1 channels 0 and 1. SAU1 has channels 0 and 1 only.

typedef struct {
	__IO uint16_t SDR[4];			//!< 0x00 Data and STCLK[15:9]
	uint8_t RESERVED0[0x78];
	__I uint16_t SSR[4];			//!< 0x80 Status
	__IO uint16_t SIR[4];			//!< 0x88 Error flag clear trigger
	__IO uint16_t SMR[4];			//!< 0x90 Mode
	__IO uint16_t SCR[4];			//!< 0x98 Communication setting
	__I uint16_t SE;				//!< 0xA0 Channel enable status
	__IO uint16_t SS;				//!< 0xA2 Channel start trigger
	__IO uint16_t ST;				//!< 0xA4 Channel stop trigger
	__IO uint16_t SPS;				//!< 0xA6 Prescaler, CKm0 [3:0], CKm1 [7:4]
	__IO uint16_t SO;				//!< 0xA8 Output level
	__IO uint16_t SOE;				//!< 0xAA Output enable
	uint8_t RESERVED1[0x08];
	__IO uint16_t SOL;				//!< 0xB4 Output level inversion
	uint8_t RESERVED2[0x02];
	__IO uint16_t SSC;				//!< 0xB8 Snooze control
} R9A02_SAU_Type;

#define R9A02_SAU0						((R9A02_SAU_Type *)R9A02_SAU0_BASE)
#define R9A02_SAU1						((R9A02_SAU_Type *)R9A02_SAU1_BASE)

#define R9A02_SAU_SDR_STCLK_Pos			9U				//!< Baud = fMCK / (STCLK + 1) / 2
#define R9A02_SAU_SDR_STCLK_Msk			(0x7FU << R9A02_SAU_SDR_STCLK_Pos)
#define R9A02_SAU_SDR_STCLK_UART_MIN	2U				//!< 0 and 1 prohibited for UART

#define R9A02_SAU_SPS_PRS0_Pos			0U				//!< CKm0
#define R9A02_SAU_SPS_PRS1_Pos			4U				//!< CKm1
#define R9A02_SAU_SPS_PRS_Msk			0xFU			//!< CKmk = PCLKB / 2^PRSk

#define R9A02_SAU_SSR_OVF				(1U << 0)
#define R9A02_SAU_SSR_PEF				(1U << 1)
#define R9A02_SAU_SSR_FEF				(1U << 2)		//!< Odd channels only
#define R9A02_SAU_SSR_BFF				(1U << 5)		//!< Buffer register holds data
#define R9A02_SAU_SSR_TSF				(1U << 6)		//!< Transfer in progress
#define R9A02_SAU_SSR_ERR_Msk			(R9A02_SAU_SSR_OVF | R9A02_SAU_SSR_PEF | R9A02_SAU_SSR_FEF)

#define R9A02_SAU_SIR_OVCT				(1U << 0)
#define R9A02_SAU_SIR_PECT				(1U << 1)
#define R9A02_SAU_SIR_FECT				(1U << 2)

#define R9A02_SAU_SMR_MD0				(1U << 0)		//!< 1: buffer empty interrupt
#define R9A02_SAU_SMR_MD_SPI			(0U << 1)		//!< MD1[1:0] = 00b, simplified SPI
#define R9A02_SAU_SMR_MD_UART			(1U << 1)		//!< MD1[1:0] = 01b
#define R9A02_SAU_SMR_MD_I2C			(2U << 1)		//!< MD1[1:0] = 10b, simplified I2C
#define R9A02_SAU_SMR_FIXED				(1U << 5)		//!< Bit 5 must be written 1
#define R9A02_SAU_SMR_SIS0				(1U << 6)		//!< RX level inversion, odd channels
#define R9A02_SAU_SMR_STS				(1U << 8)		//!< Start on RxD edge, odd channels
#define R9A02_SAU_SMR_CCS				(1U << 14)
#define R9A02_SAU_SMR_CKS				(1U << 15)		//!< 0: CKm0, 1: CKm1

#define R9A02_SAU_SCR_DLS_7BIT			0x6U			//!< Bits [2:0], bit 2 fixed to 1
#define R9A02_SAU_SCR_DLS_8BIT			0x7U
#define R9A02_SAU_SCR_DLS_9BIT			0x5U			//!< UART0 and UART2 only
#define R9A02_SAU_SCR_SLC_1BIT			(1U << 4)
#define R9A02_SAU_SCR_SLC_2BIT			(2U << 4)		//!< Transmit channels only
#define R9A02_SAU_SCR_DIR_LSB			(1U << 7)
#define R9A02_SAU_SCR_PTC_ZERO			(1U << 8)
#define R9A02_SAU_SCR_PTC_EVEN			(2U << 8)
#define R9A02_SAU_SCR_PTC_ODD			(3U << 8)
#define R9A02_SAU_SCR_EOC				(1U << 10)		//!< Error interrupt enable, odd channels
#define R9A02_SAU_SCR_DCP_Pos			12U				//!< Simplified SPI data and clock phase
#define R9A02_SAU_SCR_DCP_Msk			(3U << R9A02_SAU_SCR_DCP_Pos)
#define R9A02_SAU_SCR_DCP_DATA_EARLY	(1U << 13)		//!< DCP[1], data half a clock early: CPHA = 0
#define R9A02_SAU_SCR_DCP_CLK_REV		(1U << 12)		//!< DCP[0], clock reversed, SCK idle low: CPOL = 0 (figure 23.3)
#define R9A02_SAU_SCR_TRXE_RX			(1U << 14)
#define R9A02_SAU_SCR_TRXE_TX			(2U << 14)
#define R9A02_SAU_SCR_TRXE_TXRX			(3U << 14)

#define R9A02_SAU_SO_CKO(Ch)			(1U << (8U + (Ch)))	//!< Serial clock output level

#define R9A02_SAU_SSC_SWC				(1U << 0)		//!< Reception in Snooze (23.3.25)
#define R9A02_SAU_SSC_SSEC				(1U << 1)		//!< No error interrupt in Snooze

// Realtime clock, manual 20.2. Time and alarm registers are BCD.

typedef struct {
	__IO uint8_t SEC;				//!< 0x00
	__IO uint8_t MIN;				//!< 0x01
	__IO uint8_t HOUR;				//!< 0x02
	__IO uint8_t WEEK;				//!< 0x03 0 Sunday to 6 Saturday
	__IO uint8_t DAY;				//!< 0x04
	__IO uint8_t MONTH;				//!< 0x05
	__IO uint8_t YEAR;				//!< 0x06 00 to 99
	__IO uint8_t SUBCUD;			//!< 0x07 Time error correction
	__IO uint8_t ALARMWM;			//!< 0x08 Alarm minute
	__IO uint8_t ALARMWH;			//!< 0x09 Alarm hour
	__IO uint8_t ALARMWW;			//!< 0x0A Alarm weekdays, bit n weekday n
	__IO uint8_t RTCC0;				//!< 0x0B Control 0
	__IO uint8_t RTCC1;				//!< 0x0C Control 1
} R9A02_RTC_Type;

#define R9A02_RTC						((R9A02_RTC_Type *)R9A02_RTC_BASE)

#define R9A02_RTCC0_CT_Msk				7U				//!< Fixed-cycle interrupt period
#define R9A02_RTCC0_CT_HALF_SEC			1U
#define R9A02_RTCC0_CT_SEC				2U
#define R9A02_RTCC0_CT_MIN				3U
#define R9A02_RTCC0_CT_HOUR				4U
#define R9A02_RTCC0_CT_DAY				5U
#define R9A02_RTCC0_CT_MONTH			6U
#define R9A02_RTCC0_AMPM				(1U << 3)		//!< 24-hour
#define R9A02_RTCC0_RTC128EN			(1U << 4)		//!< 128 Hz count clock
#define R9A02_RTCC0_RCLOE1				(1U << 5)		//!< 1 Hz output on RTC1HZ
#define R9A02_RTCC0_RTCE				(1U << 7)		//!< Counting

#define R9A02_RTCC1_RWAIT				(1U << 0)		//!< Counters stopped for access
#define R9A02_RTCC1_RWST				(1U << 1)		//!< Counters accessible
#define R9A02_RTCC1_RIFG				(1U << 3)		//!< Fixed-cycle interrupt, write 0 to clear
#define R9A02_RTCC1_WAFG				(1U << 4)		//!< Alarm match, write 0 to clear
#define R9A02_RTCC1_WALIE				(1U << 6)		//!< Alarm interrupt
#define R9A02_RTCC1_WALE				(1U << 7)		//!< Alarm compare

// I2C bus interface IICA, manual 24.2. IICA1 is at IICA0 + 0x100. All
// registers are 8-bit.

typedef struct {
	__IO uint8_t IICA;				//!< 0x00 Shift register
	__I uint8_t IICS;				//!< 0x01 Status, ALD clears on read
	__IO uint8_t IICF;				//!< 0x02 Flag, read-only while IICE = 1
	uint8_t RESERVED0[0x7D];
	__IO uint8_t IICCTL0;			//!< 0x80 Control 0
	__IO uint8_t IICCTL1;			//!< 0x81 Control 1, write with IICE = 0
	__IO uint8_t IICWL;				//!< 0x82 SCL low width, write with IICE = 0
	__IO uint8_t IICWH;				//!< 0x83 SCL high width, write with IICE = 0
	__IO uint8_t SVA;				//!< 0x84 Own slave address
	uint8_t RESERVED1[0x7B];
} R9A02_IICA_Type;

#define R9A02_IICA0						((R9A02_IICA_Type *)R9A02_IICA_BASE)
#define R9A02_IICA1						((R9A02_IICA_Type *)(R9A02_IICA_BASE + 0x100UL))

#define R9A02_IICS_SPD					(1U << 0)		//!< Stop detected
#define R9A02_IICS_STD					(1U << 1)		//!< Start detected
#define R9A02_IICS_ACKD					(1U << 2)		//!< ACK detected
#define R9A02_IICS_TRC					(1U << 3)		//!< Transmit state
#define R9A02_IICS_COI					(1U << 4)		//!< Address match
#define R9A02_IICS_EXC					(1U << 5)		//!< Extension code
#define R9A02_IICS_ALD					(1U << 6)		//!< Arbitration lost
#define R9A02_IICS_MSTS					(1U << 7)		//!< Master state

#define R9A02_IICF_IICRSV				(1U << 0)		//!< 1: communication reservation off
#define R9A02_IICF_STCEN				(1U << 1)		//!< 1: start without a prior stop
#define R9A02_IICF_IICBSY				(1U << 6)		//!< Bus busy
#define R9A02_IICF_STCF					(1U << 7)		//!< Start request rejected

#define R9A02_IICCTL0_SPT				(1U << 0)		//!< Stop condition, reads 0
#define R9A02_IICCTL0_STT				(1U << 1)		//!< Start condition, reads 0
#define R9A02_IICCTL0_ACKE				(1U << 2)		//!< ACK on reception
#define R9A02_IICCTL0_WTIM				(1U << 3)		//!< 1: interrupt and stretch at the 9th clock
#define R9A02_IICCTL0_SPIE				(1U << 4)		//!< Interrupt on stop detection
#define R9A02_IICCTL0_WREL				(1U << 5)		//!< Release from clock stretch, reads 0
#define R9A02_IICCTL0_LREL				(1U << 6)		//!< Exit communication, reads 0
#define R9A02_IICCTL0_IICE				(1U << 7)		//!< Operation enable

#define R9A02_IICCTL1_PRS				(1U << 0)		//!< fMCK = PCLKB / 2, PCLKB above 20 MHz
#define R9A02_IICCTL1_DFC				(1U << 2)		//!< Digital filter, fast modes only
#define R9A02_IICCTL1_SMC				(1U << 3)		//!< Fast mode or fast mode plus
#define R9A02_IICCTL1_DAD				(1U << 4)		//!< SDA level
#define R9A02_IICCTL1_CLD				(1U << 5)		//!< SCL level
#define R9A02_IICCTL1_SVADIS			(1U << 6)		//!< All address match
#define R9A02_IICCTL1_WUP				(1U << 7)		//!< Address match wake-up

#define R9A02_IICA_PCLKB_MAX			32000000UL		//!< Notes of table 8.2
#define R9A02_IICA_FMCK_MAX				20000000UL		//!< IICCTLn1.PRS, 24.2

// UARTA, manual 25.2

typedef struct {
	__IO uint8_t TXBA;				//!< 0x00
	__I uint8_t RXBA;				//!< 0x01
	__IO uint8_t ASIMA0;			//!< 0x02
	__IO uint8_t ASIMA1;			//!< 0x03
	__IO uint8_t BRGCA;				//!< 0x04
	__I uint8_t ASISA;				//!< 0x05
	__IO uint8_t ASCTA;				//!< 0x06
	uint8_t RESERVED0;
} R9A02_UARTA_Unit_Type;

typedef struct {
	R9A02_UARTA_Unit_Type UNIT[2];	//!< 0x00, 0x08
	__IO uint8_t UTA0CK;			//!< 0x10
	__IO uint8_t UTA1CK;			//!< 0x11
} R9A02_UARTA_Type;

#define R9A02_UARTA						((R9A02_UARTA_Type *)R9A02_UARTA_BASE)

#define R9A02_ASIMA0_ISRMA				(1U << 0)		//!< 1: receive error on RX_ENDI
#define R9A02_ASIMA0_ISSMA				(1U << 1)		//!< 1: TX_ENDI on buffer empty
#define R9A02_ASIMA0_RXEA				(1U << 5)
#define R9A02_ASIMA0_TXEA				(1U << 6)
#define R9A02_ASIMA0_EN					(1U << 7)		//!< Operation clock on

#define R9A02_ASIMA1_ALV				(1U << 0)		//!< Inverted levels
#define R9A02_ASIMA1_DIR_LSB			(1U << 1)
#define R9A02_ASIMA1_SL_2BIT			(1U << 2)		//!< Transmit 2 stop bits
#define R9A02_ASIMA1_CL_5BIT			(0U << 3)
#define R9A02_ASIMA1_CL_7BIT			(2U << 3)
#define R9A02_ASIMA1_CL_8BIT			(3U << 3)
#define R9A02_ASIMA1_PS_ZERO			(1U << 5)		//!< Parity bit 0, not checked
#define R9A02_ASIMA1_PS_ODD				(2U << 5)
#define R9A02_ASIMA1_PS_EVEN			(3U << 5)

#define R9A02_ASISA_OVEA				(1U << 0)
#define R9A02_ASISA_FEA					(1U << 1)
#define R9A02_ASISA_PEA					(1U << 2)
#define R9A02_ASISA_ERR_Msk				(R9A02_ASISA_OVEA | R9A02_ASISA_FEA | R9A02_ASISA_PEA)
#define R9A02_ASISA_TXSFA				(1U << 4)		//!< Shift register transmitting
#define R9A02_ASISA_TXBFA				(1U << 5)		//!< TXBA holds data

#define R9A02_UTACK_CK_Msk				0x0FU			//!< fSEL / 2^CK, CK 0 to 6
#define R9A02_UTACK_CK_SUBCLK			0x08U			//!< LOCO or SOSC, 32768 Hz
#define R9A02_UTACK_CK_MAX				6U
#define R9A02_UTA0CK_SEL_Pos			4U				//!< fSEL of both channels
#define R9A02_UTA0CK_SEL_Msk			(3U << R9A02_UTA0CK_SEL_Pos)
#define R9A02_UTA0CK_SEL_EXTAL			(1U << R9A02_UTA0CK_SEL_Pos)
#define R9A02_UTA0CK_SEL_HOCO			(2U << R9A02_UTA0CK_SEL_Pos)
#define R9A02_UTA0CK_SEL_MOCO			(3U << R9A02_UTA0CK_SEL_Pos)

#define R9A02_UARTA_BRGCA_MIN			2U
#define R9A02_UARTA_RATE_MIN			200U			//!< Table 37.36
#define R9A02_UARTA_RATE_MAX			153600U

// 12-bit A/D converter ADC12, manual 29.2

typedef struct {
	__IO uint8_t ADM0;				//!< 0x00 Mode 0
	__IO uint8_t ADS;				//!< 0x01 Channel select
	__IO uint8_t ADM1;				//!< 0x02 Mode 1
	uint8_t RESERVED0[3];
	__I uint16_t ADCR;				//!< 0x06 Result, same as ADCR0
	uint8_t RESERVED1[0x88];
	__IO uint8_t ADM2;				//!< 0x90 Mode 2
	__IO uint8_t ADUL;				//!< 0x91 Compare upper limit
	__IO uint8_t ADLL;				//!< 0x92 Compare lower limit
	__IO uint8_t ADTES;				//!< 0x93 Test
	uint8_t RESERVED2[0x0C];
	__I uint16_t ADCRN[4];			//!< 0xA0 Results of scan 0 to 3, 16-bit access
} R9A02_ADC12_Type;

#define R9A02_ADC12						((R9A02_ADC12_Type *)R9A02_ADC12_BASE)

#define R9A02_ADM0_ADCE					(1U << 0)		//!< Comparator on
#define R9A02_ADM0_LV_Pos				1U				//!< Operation voltage mode
#define R9A02_ADM0_LV_NORMAL1			(0U << 1)		//!< Sampling 43 fAD
#define R9A02_ADM0_LV_NORMAL2			(1U << 1)		//!< Sampling 160 fAD
#define R9A02_ADM0_FR_Pos				3U				//!< fAD = PCLKB / 2^(5 - FR)
#define R9A02_ADM0_FR_Msk				(7U << 3)
#define R9A02_ADM0_FR_DIV1				5U
#define R9A02_ADM0_ADMD					(1U << 6)		//!< Scan mode
#define R9A02_ADM0_ADCS					(1U << 7)		//!< Conversion start, clears in one-shot mode

#define R9A02_ADS_ADS_Msk				0x1FU
#define R9A02_ADS_ADISS					(1U << 7)		//!< Internal input
#define R9A02_ADS_TEMP					(R9A02_ADS_ADISS | 0U)
#define R9A02_ADS_VREF					(R9A02_ADS_ADISS | 1U)

#define R9A02_ADM1_ADTRS_TAU0_ENDI1		0U				//!< Hardware trigger source
#define R9A02_ADM1_ADTRS_RTC			2U
#define R9A02_ADM1_ADTRS_TML32			3U
#define R9A02_ADM1_ADTRS_ELC			4U				//!< Event of ELSR8
#define R9A02_ADM1_ADTRS_Msk			7U
#define R9A02_ADM1_ADLSP				(1U << 3)		//!< PCLKB 1 to 4 MHz
#define R9A02_ADM1_ADSCM				(1U << 5)		//!< One-shot conversion
#define R9A02_ADM1_ADTMD_HW_NOWAIT		(2U << 6)		//!< Hardware trigger, comparator on
#define R9A02_ADM1_ADTMD_HW_WAIT		(3U << 6)		//!< Hardware trigger, comparator powered at the trigger
#define R9A02_ADM1_ADTMD_Msk			(3U << 6)

#define R9A02_ADM2_ADTYP_10BIT			0U
#define R9A02_ADM2_ADTYP_8BIT			1U
#define R9A02_ADM2_ADTYP_12BIT			2U
#define R9A02_ADM2_AWC					(1U << 2)		//!< Snooze function
#define R9A02_ADM2_ADRCK				(1U << 3)		//!< ADC_ENDI outside ADLL to ADUL
#define R9A02_ADM2_ADREFM				(1U << 5)		//!< '-' reference AVREFM (P003)
#define R9A02_ADM2_ADREFP_VCC			(0U << 6)
#define R9A02_ADM2_ADREFP_AVREFP		(1U << 6)		//!< P002
#define R9A02_ADM2_ADREFP_INTERNAL		(2U << 6)
#define R9A02_ADM2_ADREFP_DISCHARGE		(3U << 6)

#define R9A02_ADTES_ZERO_SCALE			2U				//!< Convert the '-' reference
#define R9A02_ADTES_FULL_SCALE			3U				//!< Convert the '+' reference

#define R9A02_ADC_LV_NORMAL1_SAMPLING	43U				//!< fAD cycles, table 29.3
#define R9A02_ADC_LV_NORMAL2_SAMPLING	160U

// Timer array unit, manual 18.2

typedef struct {
	__IO uint16_t TDR[8];			//!< 0x00 Compare or capture data
	uint8_t RESERVED0[0x70];
	__I uint16_t TCR[8];			//!< 0x80 Counter
	__IO uint16_t TMR[8];			//!< 0x90 Mode
	__I uint16_t TSR[8];			//!< 0xA0 Status
	__I uint16_t TE;				//!< 0xB0 Channel enable status
	__IO uint16_t TS;				//!< 0xB2 Channel start trigger
	__IO uint16_t TT;				//!< 0xB4 Channel stop trigger
	__IO uint16_t TPS;				//!< 0xB6 Prescaler
	__IO uint16_t TO;				//!< 0xB8 Output
	__IO uint16_t TOE;				//!< 0xBA Output enable
	__IO uint16_t TOL;				//!< 0xBC Output level
	__IO uint16_t TOM;				//!< 0xBE Output mode
} R9A02_TAU_Type;

#define R9A02_TAU						((R9A02_TAU_Type *)R9A02_TAU_BASE)
#define R9A02_TAU_CHAN_COUNT			8U

#define R9A02_TAU_TPS_PRS0_Pos			0U				//!< CK00 = PCLKB / 2^PRS0
#define R9A02_TAU_TPS_PRS1_Pos			4U				//!< CK01 = PCLKB / 2^PRS1
#define R9A02_TAU_TPS_PRS_Msk			0xFU

#define R9A02_TAU_TMR_OPIRQ				(1U << 0)		//!< Interrupt at count start
#define R9A02_TAU_TMR_MD_Pos			1U
#define R9A02_TAU_TMR_MD_INTERVAL		(0U << R9A02_TAU_TMR_MD_Pos)
#define R9A02_TAU_TMR_MD_CAPTURE		(2U << R9A02_TAU_TMR_MD_Pos)
#define R9A02_TAU_TMR_MD_EVENT			(3U << R9A02_TAU_TMR_MD_Pos)
#define R9A02_TAU_TMR_MD_ONECOUNT		(4U << R9A02_TAU_TMR_MD_Pos)
#define R9A02_TAU_TMR_MASTER			(1U << 11)		//!< Channels 2, 4, 6
#define R9A02_TAU_TMR_SPLIT				(1U << 11)		//!< Channels 1, 3
#define R9A02_TAU_TMR_CCS				(1U << 12)
#define R9A02_TAU_TMR_CKS_CK00			(0U << 14)
#define R9A02_TAU_TMR_CKS_CK02			(1U << 14)		//!< Channels 1, 3
#define R9A02_TAU_TMR_CKS_CK01			(2U << 14)
#define R9A02_TAU_TMR_CKS_CK03			(3U << 14)		//!< Channels 1, 3
#define R9A02_TAU_TMR_CKS_Msk			(3U << 14)

// 32-bit interval timer, manual 19.2

typedef struct {
	__IO uint16_t ITLCMP00;			//!< 0x00 Compare, channels 0 and 1
	__IO uint16_t ITLCMP01;			//!< 0x02 Compare, channels 2 and 3
	__I uint16_t ITLCAP00;			//!< 0x04 Capture
	__IO uint8_t ITLCTL0;			//!< 0x06 Control
	__IO uint8_t ITLCSEL0;			//!< 0x07 Clock select
	__IO uint8_t ITLFDIV00;			//!< 0x08 Divider, channels 0 and 1
	__IO uint8_t ITLFDIV01;			//!< 0x09 Divider, channels 2 and 3
	__IO uint8_t ITLCC0;			//!< 0x0A Capture control
	__IO uint8_t ITLS0;				//!< 0x0B Status, write 0 to clear a flag
	__IO uint8_t ITLMKF0;			//!< 0x0C Flag mask
} R9A02_TML32_Type;

#define R9A02_TML32						((R9A02_TML32_Type *)R9A02_TML32_BASE)

#define R9A02_ITLCTL0_EN0				(1U << 0)
#define R9A02_ITLCTL0_EN1				(1U << 1)
#define R9A02_ITLCTL0_EN2				(1U << 2)
#define R9A02_ITLCTL0_EN3				(1U << 3)
#define R9A02_ITLCTL0_MD_8BIT			(0U << 6)
#define R9A02_ITLCTL0_MD_16BIT			(1U << 6)
#define R9A02_ITLCTL0_MD_32BIT			(2U << 6)
#define R9A02_ITLCSEL0_ISEL_HOCO		1U
#define R9A02_ITLCSEL0_ISEL_MOCO		2U
#define R9A02_ITLCSEL0_ISEL_EXTAL		3U
#define R9A02_ITLCSEL0_ISEL_LOCO_SOSC	4U				//!< Selected by OSMCR.WUTMMCK0
#define R9A02_ITLCSEL0_ISEL_ELC			5U
#define R9A02_ITLCSEL0_ISEL_Msk			7U
#define R9A02_ITLFDIV_FDIV0_Pos			0U				//!< fITL0 / 2^FDIV, 0 to 7
#define R9A02_ITLFDIV_FDIV1_Pos			4U
#define R9A02_ITLFDIV_MAX				7U
#define R9A02_ITLS0_ITF00				(1U << 0)
#define R9A02_ITLS0_ITF01				(1U << 1)
#define R9A02_ITLS0_ITF02				(1U << 2)
#define R9A02_ITLS0_ITF03				(1U << 3)
#define R9A02_ITLS0_ITF0C				(1U << 4)
#define R9A02_ITLS0_Msk					0x1FU
#define R9A02_ITLMKF0_Msk				0x1FU			//!< 1: flag never set

// Data transfer controller, manual 14.2. The transfer information records
// and the vector table are in SRAM, see dtc_r9a02.h.

typedef struct {
	__IO uint8_t DTCCR;				//!< 0x00 Control
	uint8_t RESERVED0[3];
	__IO uint32_t DTCVBR;			//!< 0x04 Vector table base, 1 KB aligned
	uint8_t RESERVED1[4];
	__IO uint8_t DTCST;				//!< 0x0C Module start
	uint8_t RESERVED2;
	__I uint16_t DTCSTS;			//!< 0x0E Status
} R9A02_DTC_Type;

#define R9A02_DTC						((R9A02_DTC_Type *)R9A02_DTC_BASE)

#define R9A02_DTCCR_FIXED				(1U << 3)		//!< Reads 1, write 1
#define R9A02_DTCCR_RRS					(1U << 4)		//!< Skip the record read for the same vector
#define R9A02_DTCST_DTCST				(1U << 0)
#define R9A02_DTCSTS_VECN_Msk			0xFFU			//!< Vector of the active transfer
#define R9A02_DTCSTS_ACT				(1U << 15)		//!< Transfer in progress

// Event link controller, manual 15.2. ELSRn are 16-bit at 0x10 + 4n.

#define R9A02_ELC_ELCR					R9A02_REG8(R9A02_ELC_BASE + 0x00)
#define R9A02_ELC_ELSR(n)				R9A02_REG16(R9A02_ELC_BASE + 0x10 + 4U * (n))
#define R9A02_ELCR_ELCON				(1U << 7)		//!< All event links on

// 8-bit D/A converter DAC8, manual 30.2. Module stop MSTPCRC.MSTPC22 and
// MSTPCRD.MSTPD20, both set to the same value.

typedef struct {
	__IO uint16_t DADR[2];			//!< 0x00 Data, channels 0 and 1
	__IO uint8_t DACR;				//!< 0x04 Control, bits [4:0] write 1
	__IO uint8_t DADPR;				//!< 0x05 Data placement
	uint8_t RESERVED0[0x6FA];
	__IO uint16_t DAEXOUT;			//!< 0x700 Output to the pins
} R9A02_DAC8_Type;

#define R9A02_DAC8						((R9A02_DAC8_Type *)R9A02_DAC8_BASE)

#define R9A02_DACR_FIXED				0x1FU			//!< Read 1, write 1
#define R9A02_DACR_DAOE(n)				(1U << (6U + (n)))	//!< Conversion and output of channel n
#define R9A02_DADPR_DPSEL				(1U << 7)		//!< Data in DADRn [15:8]
#define R9A02_DAEXOUT_DAEXO(n)			(1U << (6U + (n)))	//!< DAn to the DACOUTn pin

// Comparators CMP, manual 31.2. All registers 8-bit.

typedef struct {
	__IO uint8_t COMPMDR;			//!< 0x00 Mode
	__IO uint8_t COMPFIR;			//!< 0x01 Filter and edge
	__IO uint8_t COMPOCR;			//!< 0x02 Output control
} R9A02_CMP_Type;

#define R9A02_CMP						((R9A02_CMP_Type *)R9A02_CMP_BASE)

#define R9A02_COMPMDR_CENB(n)			(1U << (4U * (n)))			//!< Comparator n on
#define R9A02_COMPMDR_CLVL(n)			(1U << (4U * (n) + 1U))		//!< Reference 1.4 V to VCC
#define R9A02_COMPMDR_CVRF(n)			(1U << (4U * (n) + 2U))		//!< Internal reference (CMP0), DA0 (CMP1)
#define R9A02_COMPMDR_CMON(n)			(1U << (4U * (n) + 3U))		//!< IVCMPn above the reference
#define R9A02_COMPFIR_CFCK_Pos(n)		(4U * (n))					//!< Filter sampling, 00 off
#define R9A02_COMPFIR_CFCK_Msk(n)		(3U << (4U * (n)))
#define R9A02_COMPFIR_CEPO(n)			(1U << (4U * (n) + 2U))		//!< Falling edge
#define R9A02_COMPFIR_CEDG(n)			(1U << (4U * (n) + 3U))		//!< Both edges
#define R9A02_COMPOCR_CIE(n)			(1U << (4U * (n)))			//!< Interrupt request on
#define R9A02_COMPOCR_COE(n)			(1U << (4U * (n) + 1U))		//!< VCOUTn pin output
#define R9A02_COMPOCR_COP(n)			(1U << (4U * (n) + 2U))		//!< VCOUTn inverted
#define R9A02_COMPOCR_SPDMD				(1U << 7)					//!< High-speed mode, both comparators

#endif
