/**-------------------------------------------------------------------------
@file	semihost_intrf.h

@brief	Semihosting console device interface

Sends data to the debugger console through semihosting. The debug probe
software (OpenOCD, pyOCD, J-Link GDB server,...) prints what the firmware
writes. No UART pins and no USB serial bridge are needed, so this gives a log
output to boards that only have a debug connector.

The interface is transmit only. It can be used as a SysLog output :

	static Semihost s_Semihost;

	s_Semihost.Init();
	SysLogGetInstance()->Init(s_LogCfg, &s_Semihost);
	SysLogGetInstance()->Printf("Hello\r\n");

Each transmit is one semihosting call. The CPU stays halted while the debugger
reads the data from the target memory, so log complete lines rather than
single characters.

Semihosting must be enabled in the debug probe software, for example
"arm semihosting enable" in OpenOCD. A semihosting call made without a
debugger that handles it stops the CPU in a debug halt or takes a fault.
The architecture port decides when a call is safe :

	- ARMv7-M and ARMv8-M Mainline (Cortex-M3, M4, M7, M33, M55) : the port
	  reads DHCSR.C_DEBUGEN. With no debugger connected, nothing is sent and
	  the transmit returns 0, the caller keeps its data.
	- ARMv6-M (Cortex-M0, M0+) and ARMv8-M Baseline (Cortex-M23) : no check,
	  the call is always made. Only use it with a debugger connected.
	- RISC-V : no check, the call is always made. Only use it with a debugger
	  connected.

The host console is opened on the first transmit, not in SemihostInit, so
initializing the interface never makes a semihosting call.

@author	Hoang Nguyen Hoan
@date	Nov. 8, 2026

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
#ifndef __SEMIHOST_INTRF_H__
#define __SEMIHOST_INTRF_H__

#include <stdint.h>
#include <stdbool.h>

#include "device_intrf.h"

/** @addtogroup device_intrf
  * @{
  */

// Semihosting operation numbers used by this interface. The values are
// defined by the Arm semihosting specification, RISC-V uses the same ones.
#define SEMIHOST_SYS_OPEN			0x01	//!< Open a host file, ":tt" is the console
#define SEMIHOST_SYS_CLOSE			0x02	//!< Close a host file
#define SEMIHOST_SYS_WRITE			0x05	//!< Write to a host file

// SYS_OPEN mode for ":tt". Mode 4 ("w") selects the console output.
#define SEMIHOST_OPEN_MODE_W		4

typedef struct __Semihost_Device {
	DevIntrf_t DevIntrf;		//!< This interface
	int Handle;					//!< Host console handle, -1 when not open
} SemihostDev_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Initialize the semihosting console interface.
 *
 * Fills the DevIntrf_t function table. No semihosting call is made, the host
 * console is opened on the first transmit.
 *
 * @param	pDev : Pointer to the interface instance to initialize
 *
 * @return	true - Success\n
 * 			false - pDev is NULL
 */
bool SemihostInit(SemihostDev_t * const pDev);

/**
 * @brief	Send data to the debugger console.
 *
 * @param	pDev	: Pointer to an initialized interface instance
 * @param	pData	: Data to send
 * @param	DataLen	: Number of bytes to send
 *
 * @return	Number of bytes sent. 0 when no debugger is connected (ports that
 * 			can detect it) or the debugger did not take the data.
 */
static inline int SemihostTx(SemihostDev_t * const pDev, const uint8_t *pData, int DataLen) {
	return DeviceIntrfTx(&pDev->DevIntrf, 0, pData, DataLen);
}

/**
 * @brief	Architecture semihosting call.
 *
 * Implemented once per architecture, ARM/src/semihost_arm.c (BKPT 0xAB) and
 * RISCV/src/semihost_riscv.c (slli, ebreak, srai sequence).
 *
 * @param	Op		: Semihosting operation number (SEMIHOST_SYS_xxx)
 * @param	pArg	: Operation parameter, normally a pointer to a block of
 * 					  machine words
 *
 * @return	Value returned by the debugger for the operation
 */
int SemihostCall(int Op, void *pArg);

/**
 * @brief	Check whether a semihosting call can be made.
 *
 * Implemented with SemihostCall in the architecture port. Ports that cannot
 * detect a debugger from software return true.
 *
 * @return	true - a debugger is connected or the port cannot tell\n
 * 			false - no debugger connected
 */
bool SemihostDebuggerAttached(void);

#ifdef __cplusplus
}

/// @brief	Semihosting console interface class
///
/// C++ wrapper of SemihostDev_t, same implementation as the C functions.
class Semihost : public DeviceIntrf {
public:
	bool Init(void) { return SemihostInit(&vDevData); }

	/**
	 * @brief	Operator to convert this class to device interface handle to be
	 * 			used with C functions.
	 *
	 * @return	Pointer to internal DEVINTRF to be used with C interface functions
	 */
	operator DevIntrf_t * () { return &vDevData.DevIntrf; }
	operator SemihostDev_t * () { return &vDevData; }

	/**
	 * @brief	Set data rate of the interface.
	 *
	 * The semihosting console has no data rate setting.
	 *
	 * @param	DataRate : Ignored
	 *
	 * @return 	0
	 */
	uint32_t Rate(uint32_t DataRate) { return DeviceIntrfSetRate(*this, DataRate); }

	/**
	 * @brief	Get data rate of the interface.
	 *
	 * @return	0, the semihosting console has no data rate setting
	 */
	uint32_t Rate(void) { return DeviceIntrfGetRate(*this); }

private:
	SemihostDev_t vDevData;
};

#endif

/** @} end group device_intrf */

#endif	// __SEMIHOST_INTRF_H__
