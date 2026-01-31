/**-------------------------------------------------------------------------
@file	iopincfg.h

@brief	Generic I/O pin configuration

This file contains only generic I/O pin configuration.

The I/O pin control is to be implemented per MCU device in iopinctrl.h

@author	Hoang Nguyen Hoan
@date	Nov. 20, 2011

@license

Copyright (c) 2011, I-SYST inc., all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : hnhoan at i-syst dot com

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------*/
#ifndef __IOPINCFG_H__
#define __IOPINCFG_H__

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

/// Defining port name
/// Many MCU name its GPIO port by letter starting from A instead of numerical
#define IOPORTA				0
#define IOPORTB				1
#define IOPORTC				2
#define IOPORTD				3
#define IOPORTE				4
#define IOPORTF				5
#define IOPORTG				6
#define IOPORTH				7
#define IOPORTI				8
#define IOPORTJ				9

/// PINOP indicates pin operating mode as GPIO or alternative functionalities
/// each MCU brand defines its own function code & naming. Implementation must
/// map this function value to the appropriate function code. For example
/// Microchip alternate function as Peripheral A, B, C...
/// STM32 name it as AF0...AF15
/// NXP as pin function
/// Nordic does not use pin function
#define IOPINOP_GPIO		0		//!< Normal GPIO
#define IOPINOP_FUNC0		1		//!< Alternate function or Peripheral function
#define IOPINOP_FUNC1		2
#define IOPINOP_FUNC2		3
#define IOPINOP_FUNC3		4
#define IOPINOP_FUNC4		5
#define IOPINOP_FUNC5		6
#define IOPINOP_FUNC6		7
#define IOPINOP_FUNC7		8
#define IOPINOP_FUNC8		9
#define IOPINOP_FUNC9		10
#define IOPINOP_FUNC10		11
#define IOPINOP_FUNC11		12
#define IOPINOP_FUNC12		13
#define IOPINOP_FUNC13		14
#define IOPINOP_FUNC14		15
#define IOPINOP_FUNC15		16
#define IOPINOP_FUNC16		17
#define IOPINOP_FUNC17		18
#define IOPINOP_FUNC18		19
#define IOPINOP_FUNC19		20
#define IOPINOP_FUNC20		21
#define IOPINOP_FUNC21		22
#define IOPINOP_FUNC22		23
#define IOPINOP_FUNC23		24
#define IOPINOP_FUNC24		25
#define IOPINOP_FUNC25		26
#define IOPINOP_FUNC26		27
#define IOPINOP_FUNC27		28
#define IOPINOP_FUNC28		29
#define IOPINOP_FUNC29		30
#define IOPINOP_FUNC30		31
#define IOPINOP_FUNC31		32

// MCU such as Microchip name it peripheral A-X
#define IOPINOP_PERIPHA		IOPINOP_FUNC0
#define IOPINOP_PERIPHB		IOPINOP_FUNC1
#define IOPINOP_PERIPHC		IOPINOP_FUNC2
#define IOPINOP_PERIPHD		IOPINOP_FUNC3
#define IOPINOP_PERIPHE		IOPINOP_FUNC4
#define IOPINOP_PERIPHF		IOPINOP_FUNC5
#define IOPINOP_PERIPHG		IOPINOP_FUNC6
#define IOPINOP_PERIPHH		IOPINOP_FUNC7
#define IOPINOP_PERIPHI		IOPINOP_FUNC8
#define IOPINOP_PERIPHJ		IOPINOP_FUNC9
#define IOPINOP_PERIPHK		IOPINOP_FUNC10
#define IOPINOP_PERIPHL		IOPINOP_FUNC11
#define IOPINOP_PERIPHM		IOPINOP_FUNC12
#define IOPINOP_PERIPHN		IOPINOP_FUNC13
#define IOPINOP_PERIPHO		IOPINOP_FUNC14
#define IOPINOP_PERIPHP		IOPINOP_FUNC15
#define IOPINOP_PERIPHQ		IOPINOP_FUNC16
#define IOPINOP_PERIPHR		IOPINOP_FUNC17
#define IOPINOP_PERIPHS		IOPINOP_FUNC18
#define IOPINOP_PERIPHT		IOPINOP_FUNC19
#define IOPINOP_PERIPHU		IOPINOP_FUNC20
#define IOPINOP_PERIPHV		IOPINOP_FUNC21
#define IOPINOP_PERIPHW		IOPINOP_FUNC22
#define IOPINOP_PERIPHX		IOPINOP_FUNC23
#define IOPINOP_PERIPHY		IOPINOP_FUNC24
#define IOPINOP_PERIPHZ		IOPINOP_FUNC25


/**
 * @brief Pin capabilities (32-bit bitmask)
 *
 * Each bit indicates whether a pin supports a particular function.
 * For RAG parsing: look for PINMUX_ prefix defines.
 */
typedef enum __IOPin_Mux_Function {
    IOPINMUX_GPIO			= (1 << 0),		//!< General purpose I/O
    IOPINMUX_ADC			= (1 << 1),  	//!< Analog-to-digital
    IOPINMUX_CAN			= (1 << 2),		//!< CAN bus
    IOPINMUX_CLOCK_OUT		= (1 << 3),		//!< Clock output
    IOPINMUX_COMP_IN		= (1 << 4),		//!< Comparator input
    IOPINMUX_DAC			= (1 << 5),  	//!< Digital-to-analog
	IOPINMUX_DCMI			= (1 << 6),  	//!< Digital camera interface
	IOPINMUX_DISPLAY		= (1 << 7),		//!< Display interface (lcd, segmented,...)
	IOPINMUX_ETH			= (1 << 8),  	//!< Ethernet (RMII/MII/RGMII)
    IOPINMUX_EXTREF			= (1 << 9),		//!< ADC External reference
	IOPINMUX_FMC			= (1 << 10),  	//!< External memory controller
    IOPINMUX_I2C			= (1 << 11),   	//!< I2C/TWI
    IOPINMUX_I2S			= (1 << 12),  	//!< I2S
    IOPINMUX_I3C			= (1 << 13),   	//!< I3C
	IOPINMUX_JTAG_SWD       = (1 << 14),  	//!< JTAG/SWD debug
    IOPINMUX_NFC			= (1 << 15),  	//!< NFC antenna
    IOPINMUX_PDM			= (1 << 16),  	//!< PDM
    IOPINMUX_PWM			= (1 << 17),	//!< PWM output
    IOPINMUX_QDEC			= (1 << 18),	//!< Quadrature decoder
    IOPINMUX_RESET			= (1 << 19),	//!< Reset
	IOPINMUX_SPI			= (1 << 20),	//!< SPI
    IOPINMUX_DSPI			= (1 << 21),  	//!< Dual SPI
    IOPINMUX_QSPI			= (1 << 22),  	//!< Quad SPI
    IOPINMUX_OSPI			= (1 << 23),  	//!< Octo SPI
	IOPINMUX_SAI			= (1 << 24),	//!< Serial Audio Interface
	IOPINMUX_SDMMC			= (1 << 25),	//!< SD/MMC/SDIO
    IOPINMUX_TDM			= (1 << 26),	//!< Time division multiplex
    IOPINMUX_TIMER_CAP		= (1 << 27),  	//!< Timer capture input
    IOPINMUX_TRACE			= (1 << 28),	//!< Debug trace
	IOPINMUX_UART			= (1 << 29),   	//!< UART
    IOPINMUX_USB			= (1 << 30),  	//!< USB
    IOPINMUX_XTAL			= (1 << 31)		//!< Xtal oscillator
} IOPINMUX;

#pragma pack(push,1)
/**
 * @brief Pin map entry - one per physical pin
 *
 * For RAG: Parse arrays of this struct to build pin capability database.
 * Flag serves to indicate indicate whether the pin has been assigned or not
 */
typedef struct __Pin_Map_Entry {
    int8_t   PortNo;		//!< Port number (-1 = entry unused)
    int8_t   PinNo;       	//!< Pin number within port
    int8_t   Reserved[2];  	//!< Alignment padding
    IOPINMUX Caps;        	//!< Capability bitmask (PINMUX)
    IOPINMUX Conn;			//!< Pin connected to PINMUX function, 0 - unused
    uint32_t Id;			//!< Pin id, device dependent
} PinMapEntry_t;
#pragma pack(pop)

// Number of entries macro for pin map array sizing
#define IOPIN_MAP_COUNT(x) (sizeof(x) / sizeof(PinMapEntry_t))

/// I/O pin resistor configuration
typedef enum __iopin_resistor {
	IOPINRES_NONE,				//!< No pullup or pulldown
	IOPINRES_PULLUP,			//!< Pullup resistor
	IOPINRES_PULLDOWN,			//!< Pulldown resistor
	IOPINRES_FOLLOW				//!< Few MCUs support this mode
} IOPINRES;

/// I/O pin direction configuration
typedef enum __iopin_dir {
    IOPINDIR_INPUT  = 0,		//!< I/O pin as input
    IOPINDIR_OUTPUT = 1,		//!< I/O pin as output
    IOPINDIR_BI     = 2,		//!< Bidirectional, few MCUs support this mode
} IOPINDIR;

/// I/O pin type
typedef enum __iopin_type {
	IOPINTYPE_NORMAL    = 0,	//!< I/O pin normal type
	IOPINTYPE_OPENDRAIN = 1		//!< I/O pin open drain type
} IOPINTYPE;

/// I/O pin sense type
typedef enum __iopin_sense {
	IOPINSENSE_DISABLE,			//!< Disable pin sense
	IOPINSENSE_LOW_TRANSITION,	//!< Event on falling edge
	IOPINSENSE_HIGH_TRANSITION,	//!< Event on raising edge
	IOPINSENSE_TOGGLE,			//!< Event on state change
} IOPINSENSE;

/// I/O pin drive strength
typedef enum __iopin_drive_strength {
	IOPINSTRENGTH_REGULAR,		//!< Regular driver strength (normal default)
	IOPINSTRENGTH_STRONG,		//!< Stronger drive strength
} IOPINSTRENGTH;

/// I/O pin speed. This setting only available on some MCU
typedef enum __iopin_speed {
	IOPINSPEED_LOW,
	IOPINSPEED_MEDIUM,
	IOPINSPEED_HIGH,
	IOPINSPEED_TURBO
} IOPINSPEED;

#pragma pack(push,4)

/// I/O pin configuration data
typedef struct __iopin_cfg {
	int 		PortNo;			//!< Port number
	int 		PinNo;			//!< Pin number
	int 		PinOp;			//!< Pin function select index from 0, MCU dependent
	IOPINDIR	PinDir;			//!< Pin direction
	IOPINRES 	Res;			//!< Pin resistor setting
	IOPINTYPE	Type;			//!< I/O type
} IOPinCfg_t;

typedef IOPinCfg_t		IOPINCFG;

#pragma pack(pop)

/**
 * @brief	I/O pin event callback
 *
 * @param	IntNo : Interrupt number to which the I/O pin sense was assigned to.
 */
typedef void (*IOPinEvtHandler_t)(int IntNo, void *pCtx);

#ifdef 	__cplusplus
extern "C" {
#endif

/**
 * @brief Configure individual I/O pin.
 *
 * This function is MCU dependent. Needs to be implemented per MCU
 *
 * @param 	PortNo	: Port number
 * @param	PinNo  	: Pin number
 * @param	PinOp	: Pin function. MCU dependent, see implementation for details
 * @param	Dir     : I/O direction
 * @param	Resistor : Resistor config
 * @param	Type 	: I/O type
 */
void IOPinConfig(int PortNo, int PinNo, int PinOp, IOPINDIR Dir, IOPINRES Resistor, IOPINTYPE Type);

/**
 * @brief	Configure I/O pin with IOPIN_CFG data structure. Can be used for batch configuration
 *
 * @param	pCfg   : Pointer to an array gpio pin configuration
 * @param	NbPins : Number of gpio pins to configure
 */
static inline void IOPinCfg(const IOPinCfg_t *pCfg, int NbPins) {
	if (pCfg == NULL || NbPins <= 0)
		return;

	for (int i = 0; i < NbPins; i++) {
		IOPinConfig(pCfg[i].PortNo, pCfg[i].PinNo, pCfg[i].PinOp, pCfg[i].PinDir,
					pCfg[i].Res, pCfg[i].Type);
	}
}

/**
 * @brief	Disable I/O pin
 *
 * Some hardware such as low power mcu allow I/O pin to be disconnected
 * in order to save power. There is no enable function. Reconfigure the
 * I/O pin to re-enable it.
 *
 * @param	PortNo 	: Port number
 * @param	PinNo	: Pin Number
 */
void IOPinDisable(int PortNo, int PinNo);

/**
 * @brief	Batch disable I/O pin
 *
 * Some hardware such as low power mcu allow I/O pin to be disconnected
 * in order to save power. There is no enable function. Reconfigure the
 * I/O pin to re-enable it.
 *
 * @param	pCfg   : Pointer to an array gpio pin to disable
 * @param	NbPins : Number of gpio pins to disable
 */
static inline void IOPinDis(const IOPinCfg_t *pCfg, int NbPins) {
	for (int i = 0; i < NbPins; i++) {
		IOPinDisable(pCfg[i].PortNo, pCfg[i].PinNo);
	}
}

/**
 * @brief	Disable I/O pin sense interrupt
 *
 * @param	IntNo : Interrupt number to disable
 */
void IOPinDisableInterrupt(int IntNo);

/**
 * @brief Enable I/O pin sensing interrupt event
 *
 * Generate an interrupt when I/O sense a state change.
 * The IntNo (interrupt number) parameter is processor dependent. Some is
 * directly the hardware interrupt number other is just an index in an array
 *
 *
 * @param	IntNo	: Interrupt number.
 * @param	IntPrio : Interrupt priority
 * @param	PortNo  : Port number (up to 32 ports)
 * @param	PinNo   : Pin number (up to 32 pins). In port interrupt mode, this
 * 					  parameter contains pins mask.
 * 					  NOTE: Port interrupt support is MCU dependent
 * @param	Sense   : Sense type of event on the I/O pin
 * @param	pEvtCB	: Pointer to callback function when event occurs
 * @param	pCtx	: Pointer to context data to be pass to the handler function
 *
 * @return	true - success
 */
bool IOPinEnableInterrupt(int IntNo, int IntPrio, uint32_t PortNo, uint32_t PinNo, IOPINSENSE Sense, IOPinEvtHandler_t pEvtCB, void *pCtx);

/**
 * @brief	Allocate I/O pin sensing interrupt event
 *
 * Generate an interrupt when I/O sense a state change. This function will automatically
 * allocate available interrupt number to use for the pin.
 * The IntNo (interrupt number) parameter is processor dependent. Some is
 * directly the hardware interrupt number other is just an index in an array
 *
 *
 * @param	IntPrio : Interrupt priority
 * @param	PortNo  : Port number (up to 32 ports)
 * @param	PinNo   : Pin number (up to 32 pins)
 * @param	Sense   : Sense type of event on the I/O pin
 * @param	pEvtCB	: Pointer to callback function when event occurs
 * @param	pCtx	: Pointer to context data to be pass to the handler function
 *
 * @return	Interrupt number on success
 * 			-1 on failure.
 */
int IOPinAllocateInterrupt(int IntPrio, int PortNo, int PinNo, IOPINSENSE Sense, IOPinEvtHandler_t pEvtCB, void *pCtx);

/**
 * @brief Set I/O pin sensing option
 *
 * Some hardware allow pin sensing to wake up or active other subsystem without
 * requiring enabling interrupts. This requires the I/O already configured
 *
 * @param	PortNo : Port number (up to 32 ports)
 * @param	PinNo   : Pin number (up to 32 pins)
 * @param	Sense   : Sense type of event on the I/O pin
 */
void IOPinSetSense(int PortNo, int PinNo, IOPINSENSE Sense);

/**
 * @brief Set I/O pin drive strength option
 *
 * Some hardware allow setting pin drive strength. This requires the I/O already configured
 *
 * @param	PortNo 	: Port number (up to 32 ports)
 * @param	PinNo  	: Pin number (up to 32 pins)
 * @param	Strength  : Pin drive strength
 */
void IOPinSetStrength(int PortNo, int PinNo, IOPINSTRENGTH Strength);

/**
 * @brief Set I/O pin speed option
 *
 * Some hardware allow setting pin speed. This requires the I/O already configured
 *
 * @param	PortNo 	: Port number (up to 32 ports)
 * @param	PinNo  	: Pin number (up to 32 pins)
 * @param	Speed	: Pin speed
 */
void IOPinSetSpeed(int PortNo, int PinNo, IOPINSPEED Speed);

/**
 * @brief	Get pin capability
 *
 * This function return the capability (function) of a pin.
 *
 * @param	PortNo 	: Port number (up to 32 ports)
 * @param	PinNo  	: Pin number (up to 32 pins)
 *
 * @return	PINMUX - orable bit field of the capability list.
 */
IOPINMUX IOPinGetCaps(uint8_t PortNo, uint8_t PinNo);

/**
 * @brief	Allocate pin usage
 *
 * Allocate pin for assignment to a function
 *
 * @param	PortNo 	: Port number (up to 32 ports)
 * @param	PinNo  	: Pin number (up to 32 pins)
 * @param 	Fct		: Pin mux selected
 * @param	Id		: Pin identifier specific to each function
 *
 * @return	-1 - Pin in use by other function
 * 			index value in the pin assignment array.
 */
int IOPinAlloc(uint8_t PortNo, uint8_t PinNo, IOPINMUX Fct, uint8_t Id);
void IOPinRelease(uint8_t PortNo, uint8_t PinNo);
int IOPinFind(IOPINMUX Fct);

#ifdef __cplusplus
}
#endif

#endif	// __IOPINCFG_H__
