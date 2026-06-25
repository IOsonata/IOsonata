/**-------------------------------------------------------------------------
@file	icli.h

@brief	Generic command line interface.

Line editor and command dispatcher over a DeviceIntrf. The core touches no
hardware. It reads and writes through any DevIntrf object: UART, BLE through
BtIntrf, or any other interface implementation. The same interface instance
can be shared with the system logger, so records and the command prompt ride
one wire. No dynamic allocation, caller owns all buffers.

Two input paths use the same editor :
  - Poll : call CliProcess from the main loop. It drains the interface
    receive side with DeviceIntrfRx and feeds the editor.
  - Event : call CliInput per received byte from a DEVINTRF_EVT_RX_DATA
    handler, for an interrupt or rtos driven design with no polling.

Commands are grouped into sets. A set is a table of CliCmd_t in flash plus
one CliCmdSet_t node in ram. The base set is supplied at init. Each module
can add its own set with CliCmdRegister, so commands live next to the code
they drive instead of in one central table. CliCmdHelp lists every command
across all registered sets.

@author	Hoang Nguyen Hoan
@date	Jun. 25, 2026

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
#ifndef __ICLI_H__
#define __ICLI_H__

#include <stdint.h>
#include <stdbool.h>

#include "device_intrf.h"

/** @addtogroup Utilities
  * @{
  */

// Receive buffer size used by CliProcess. DeviceIntrfRx returns one packet per
// call for a packetized interface such as BtIntrf, so size this at or above the
// interface packet size to read a packet in full, as UartBleBridge does with a
// 128 byte read. A byte stream interface such as UART has no minimum.
#ifndef CLI_RX_CHUNK
#define CLI_RX_CHUNK	128
#endif

/// Forward declaration of the command line device.
typedef struct __Cli_Dev CliDev_t;

/// @brief	Command executor.
///
/// argv[0] is the command token. Returns 0 on success, non zero on error.
/// The strings in argv point into the line buffer and stay valid only for the
/// duration of the call.
typedef int (*CliCmdHandler_t)(CliDev_t * const pCli, int argc, char *argv[]);

/// @brief	Command table entry. Place the table in flash.
typedef struct __Cli_Cmd {
	const char *pName;				//!< Command token to match argv[0]
	CliCmdHandler_t Handler;		//!< Executor, must not be NULL
	const char *pHelp;				//!< One line help text, may be NULL
} CliCmd_t;

/// @brief	A registered command set. One node in ram per table.
///
/// pNext is managed by the core, the application leaves it alone. Allocate the
/// node statically, it must outlive the cli device.
typedef struct __Cli_Cmd_Set CliCmdSet_t;
struct __Cli_Cmd_Set {
	const CliCmd_t *pCmd;			//!< Command table in flash
	int NbCmd;						//!< Number of entries in the table
	CliCmdSet_t *pNext;				//!< Next set in the chain, set by the core
};

/// @brief	Init parameters.
typedef struct __Cli_Cfg {
	const CliCmd_t *pCmd;			//!< Base command table, may be NULL
	int NbCmd;						//!< Number of entries in the base table
	char *pLineBuf;					//!< Line buffer supplied by caller
	int LineBufSize;				//!< Line buffer size in bytes, includes terminator
	char **pArgv;					//!< argv slot array supplied by caller
	int ArgvMax;					//!< Number of argv slots
	const char *pPrompt;			//!< Prompt string, may be NULL
	bool bEcho;						//!< Echo received characters back to the interface
	DevIntrf_t *pIntrf;				//!< I/O interface, UART, BtIntrf, any DeviceIntrf
	uint32_t IntrfAddr;				//!< Device select id for DeviceIntrf Rx and Tx, 0 for UART and BLE
} CliCfg_t;

/// @brief	Command line device state. Application does not access members directly.
struct __Cli_Dev {
	DevIntrf_t *pIntrf;				//!< I/O interface
	uint32_t IntrfAddr;				//!< Device select id for the interface
	CliCmdSet_t BaseSet;			//!< Base command set built from cfg, head of the chain
	char *pLineBuf;					//!< Line buffer
	int LineBufSize;				//!< Line buffer size in bytes
	int LineLen;					//!< Number of characters held in the line buffer
	char **pArgv;					//!< argv slot array
	int ArgvMax;					//!< Number of argv slots
	const char *pPrompt;			//!< Prompt string, may be NULL
	bool bEcho;						//!< Echo flag
	uint8_t LastCh;					//!< Previous byte, used to fold CR LF into one line end
};

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Initialize a command line device.
 *
 * @param	pCli : Pointer to a CliDev_t instance
 * @param	pCfg : Init parameters. The interface and the buffers referenced by
 *				   pCfg must stay valid for the lifetime of pCli.
 *
 * @return	true on success, false on invalid parameters
 */
bool CliInit(CliDev_t * const pCli, const CliCfg_t *pCfg);

/**
 * @brief	Add a command set to the device.
 *
 * The set node and the command table must stay valid for the lifetime of the
 * device. Sets are searched in registration order, base set first. The first
 * matching command name wins.
 *
 * @param	pCli  : Pointer to a CliDev_t instance
 * @param	pSet  : Caller owned set node, fields filled by this call
 * @param	pCmd  : Command table in flash
 * @param	NbCmd : Number of entries in the table
 */
void CliCmdRegister(CliDev_t * const pCli, CliCmdSet_t * const pSet, const CliCmd_t *pCmd, int NbCmd);

/**
 * @brief	Feed one received byte through the line editor.
 *
 * Handles printable characters, backspace and delete, and the CR or LF line
 * end. On a completed line the command sets are searched and the matching
 * handler is run. Output, including echo and prompt, goes to the interface.
 * Use this from a DEVINTRF_EVT_RX_DATA handler for an event driven design.
 *
 * @param	pCli : Pointer to a CliDev_t instance
 * @param	Ch	 : Received byte
 *
 * @return	1 when a command line was dispatched, 0 otherwise
 */
int CliInput(CliDev_t * const pCli, uint8_t Ch);

/**
 * @brief	Drain the interface receive side and feed any available bytes.
 *
 * Call from the main loop. Reads with DeviceIntrfRx in chunks of CLI_RX_CHUNK
 * bytes, draining all queued packets.
 *
 * @param	pCli : Pointer to a CliDev_t instance
 *
 * @return	Number of bytes consumed from the interface
 */
int CliProcess(CliDev_t * const pCli);

/**
 * @brief	Write a null terminated string to the interface.
 *
 * @param	pCli : Pointer to a CliDev_t instance
 * @param	pStr : Null terminated string
 *
 * @return	Number of bytes written
 */
int CliPuts(CliDev_t * const pCli, const char *pStr);

/**
 * @brief	Print the prompt to the interface.
 *
 * @param	pCli : Pointer to a CliDev_t instance
 */
void CliPrompt(CliDev_t * const pCli);

/**
 * @brief	Library command that lists every command in every registered set.
 *
 * Register it like any other command, for example
 * { "help", CliCmdHelp, "List commands" }. Never goes stale.
 *
 * @param	pCli : Pointer to a CliDev_t instance
 * @param	argc : Argument count
 * @param	argv : Argument vector
 *
 * @return	0
 */
int CliCmdHelp(CliDev_t * const pCli, int argc, char *argv[]);

#ifdef __cplusplus
}

/// @brief	C++ wrapper for the command line device.
class Cli {
public:
	bool Init(const CliCfg_t &Cfg) { return CliInit(&vDev, &Cfg); }
	void Register(CliCmdSet_t &Set, const CliCmd_t *pCmd, int NbCmd) { CliCmdRegister(&vDev, &Set, pCmd, NbCmd); }
	int Input(uint8_t Ch) { return CliInput(&vDev, Ch); }
	int Process(void) { return CliProcess(&vDev); }
	int Puts(const char *pStr) { return CliPuts(&vDev, pStr); }
	void Prompt(void) { CliPrompt(&vDev); }
	operator CliDev_t * () { return &vDev; }

private:
	CliDev_t vDev;
};

#endif

/** @} End of group Utilities */

#endif // __ICLI_H__
