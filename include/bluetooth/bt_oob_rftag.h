/**-------------------------------------------------------------------------
@file	bt_oob_rftag.h

@brief	Bluetooth LE Secure Connections OOB over an NFC tag

Bluetooth library module. Publishes the LESC OOB pairing material through
the rftag NDEF layer, which is the out of band carrier.

Builds the standard NFC record a phone reads to obtain the LE Secure
Connections OOB pairing material. The record media type is
application/vnd.bluetooth.le.oob and the payload is a sequence of AD
structures per the Bluetooth Core Specification Supplement:

	LE Bluetooth Device Address, AD type 0x1B, 6 byte address plus 1 byte
	address type
	LE Role, AD type 0x1C
	LE Secure Connections Confirmation Value, AD type 0x22, 16 bytes
	LE Secure Connections Random Value, AD type 0x23, 16 bytes
	Local Name, AD type 0x09, optional

The confirm and random come from BtSmpOobLocalDataGen and the address from
BtSmpLocalAddrGet, both already in SMP byte order, low octet first, which is
the order this record requires.

@author	Hoang Nguyen Hoan
@date	Jul. 5, 2026

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
#ifndef __BT_OOB_RFTAG_H__
#define __BT_OOB_RFTAG_H__

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "rftag/rftag_ndef.h"

// LE Role AD values
#define BT_OOB_LEROLE_PERIPH			0	//!< Peripheral only
#define BT_OOB_LEROLE_CENTRAL			1	//!< Central only
#define BT_OOB_LEROLE_BOTH_PERIPH		2	//!< Both, peripheral preferred
#define BT_OOB_LEROLE_BOTH_CENTRAL	3	//!< Both, central preferred

// Payload size without the name: addr AD 9, role AD 3, confirm AD 18,
// random AD 18
#define BT_OOB_LE_MIN_PAYLOAD		48

#pragma pack(push, 4)

typedef struct __Bt_OobLe {
	uint8_t Addr[6];			//!< Device address, low octet first
	uint8_t AddrType;			//!< 0 public, 1 random
	uint8_t Role;				//!< BT_OOB_LEROLE_ value
	uint8_t Confirm[16];		//!< LE SC confirm C, low octet first
	uint8_t Rand[16];			//!< LE SC random r, low octet first
	const char *pName;			//!< Optional local name, null for none. Longest
								//!< accepted name is 64 bytes, longer fails the build
} BtOobLe_t;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Build the LE OOB AD structure payload.
 *
 * Fills pBuf with the AD structure sequence of the le.oob record. Useful on
 * its own for tests or a different carrier.
 *
 * @param	pOob	OOB data set
 * @param	pBuf	Output buffer
 * @param	Size	Output buffer size in bytes
 *
 * @return	Payload length in bytes, 0 on failure
 */
int BtOobLePayload(const BtOobLe_t * const pOob, uint8_t *pBuf, size_t Size);

/**
 * @brief	Append the application/vnd.bluetooth.le.oob record to a message.
 *
 * @param	pMsg	NDEF message under construction
 * @param	pOob	OOB data set
 *
 * @return	true on success
 */
bool BtOobLeNdefAdd(RFNdefMsg_t * const pMsg, const BtOobLe_t * const pOob);

#ifdef __cplusplus
}
#endif

#endif	// __BT_OOB_RFTAG_H__
