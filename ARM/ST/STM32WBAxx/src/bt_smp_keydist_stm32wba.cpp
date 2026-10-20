/**-------------------------------------------------------------------------
@file	bt_smp_keydist_stm32wba.cpp

@brief	STM32WBA pairing response: keep the EncKey bit for Secure Connections.

With LE Secure Connections the Cube BLE stack masks the EncKey bit out of the
initiator key distribution field of its Pairing Response. The long term key is
generated on both sides (f5), so no key is sent and the bit has no effect on
the link. The bit does matter to some centrals. A SoftDevice based central,
nRF Connect Desktop for one, keeps the long term key of its own side only when
that bit is part of the negotiated distribution. Without it the central stores
no key for the peer and presents a stale or empty key on the next connection.
The link layer then ends the connection with reason 0x3D (MIC failure) about
60 ms after the Security Request.

The Cube stack has no setting for this field, so the two stack entry points
that see the pairing PDUs are wrapped at link time:

  -Wl,--wrap=SMP_L2cap_Data_Handler   incoming SMP PDUs, to note the Pairing
                                      Request key distribution
  -Wl,--wrap=L2C_Send_Data            outgoing L2CAP PDUs, to set the bit in the
                                      Pairing Response

Both options must be on the link line of a project that uses this port with
the stm32wba_ble_stack_full library. A project without the options links and
runs as before, this file is then not called.

The bit is set only when all of these hold:
- the PDU is an SMP Pairing Response,
- Secure Connections is in use (SC bit of the AuthReq),
- the Pairing Request of the same pairing had the EncKey bit in its initiator
  key distribution, so the response stays a subset of the request.

The responder key distribution and the other bits are left as the stack set
them. The pairing is one at a time in the stack, so a single remembered request
is enough.

The packet layout used here is the one of stm32wba_ble_stack_full.a as shipped
with the Cube package this port is built on: the packet descriptor holds the
length at offset 4, the direction flag at offset 6 (0 for received) and the
pointer to the buffer at offset 32, and the buffer starts with the 4 octets of
the L2CAP header, followed by the SMP PDU.

@author	Hoang Nguyen Hoan
@date	Oct. 18, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is furnished
to do so, subject to the following conditions:

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
#include <stdint.h>
#include <string.h>

// L2CAP fixed channel of the Security Manager
#define BT_WBA_SMP_CID				0x0006U

// SMP opcodes
#define BT_WBA_SMP_PAIRING_REQ		0x01U
#define BT_WBA_SMP_PAIRING_RSP		0x02U

// Packet descriptor of the Cube stack
#define BT_WBA_PKT_LEN_OFS			4U		//!< Length including the L2CAP header
#define BT_WBA_PKT_DIR_OFS			6U		//!< 0 for a received packet
#define BT_WBA_PKT_BUF_OFS			32U		//!< Pointer to the buffer

// Buffer: L2CAP header, then the SMP PDU
#define BT_WBA_BUF_OPCODE			4U
#define BT_WBA_BUF_AUTHREQ			7U
#define BT_WBA_BUF_INIT_KEYS		9U

// Pairing Request and Response: opcode and 6 parameters, 4 octets of header
#define BT_WBA_PAIRING_PDU_LEN		11U

#define BT_WBA_AUTHREQ_SC			0x08U
#define BT_WBA_KEYDIST_ENC			0x01U

extern "C" {

uint8_t __real_SMP_L2cap_Data_Handler(void *pPkt);
uint8_t __real_L2C_Send_Data(uint16_t Cid, void *pPkt);

}

// Initiator EncKey bit of the last Pairing Request
static uint8_t s_ReqInitEnc = 0;

// Buffer of a packet, or 0 when the packet is not a pairing PDU of the length
// the stack builds. The opcode is checked by the caller.
static uint8_t *BtWbaPairingPdu(void *pPkt)
{
	if (pPkt == 0)
	{
		return 0;
	}

	uint8_t *p = (uint8_t *)pPkt;
	uint16_t len;
	uint8_t *pBuf;

	memcpy(&len, p + BT_WBA_PKT_LEN_OFS, sizeof(len));
	if (len != BT_WBA_PAIRING_PDU_LEN)
	{
		return 0;
	}

	memcpy(&pBuf, p + BT_WBA_PKT_BUF_OFS, sizeof(pBuf));

	return pBuf;
}

extern "C" uint8_t __wrap_SMP_L2cap_Data_Handler(void *pPkt)
{
	// Received SMP PDU. Take the key distribution of a Pairing Request before
	// the stack queues the packet, the response is built in the same buffer.
	if (pPkt != 0 && ((uint8_t *)pPkt)[BT_WBA_PKT_DIR_OFS] == 0)
	{
		uint8_t *pBuf = BtWbaPairingPdu(pPkt);

		if (pBuf != 0 && pBuf[BT_WBA_BUF_OPCODE] == BT_WBA_SMP_PAIRING_REQ)
		{
			s_ReqInitEnc = pBuf[BT_WBA_BUF_INIT_KEYS] & BT_WBA_KEYDIST_ENC;
		}
	}

	return __real_SMP_L2cap_Data_Handler(pPkt);
}

extern "C" uint8_t __wrap_L2C_Send_Data(uint16_t Cid, void *pPkt)
{
	if (Cid == BT_WBA_SMP_CID && s_ReqInitEnc != 0)
	{
		uint8_t *pBuf = BtWbaPairingPdu(pPkt);

		if (pBuf != 0 && pBuf[BT_WBA_BUF_OPCODE] == BT_WBA_SMP_PAIRING_RSP &&
			(pBuf[BT_WBA_BUF_AUTHREQ] & BT_WBA_AUTHREQ_SC) != 0)
		{
			pBuf[BT_WBA_BUF_INIT_KEYS] |= BT_WBA_KEYDIST_ENC;
		}
	}

	return __real_L2C_Send_Data(Cid, pPkt);
}
