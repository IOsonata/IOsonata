/**-------------------------------------------------------------------------
@file	bt_attrsp.cpp

@brief	Generic Bluetooth ATT protocol
Implementation of ATT command response

@author	Thinh Tran
@date	Apr. 03, 2024

@license

MIT License

Copyright (c) 2024, I-SYST inc., all rights reserved

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
#include <stddef.h>
#include <memory.h>

#include "istddef.h"
#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_l2cap.h"
#include "bluetooth/bt_att.h"

#include "bluetooth/bt_dev.h"
#include "bluetooth/bt_app.h"
#include "bluetooth/bt_uuid.h"
#include "bluetooth/bt_gatt.h"

/******** For DEBUG ************/
//#define UART_DEBUG_ENABLE

#ifdef UART_DEBUG_ENABLE
#include "coredev/uart.h"
extern UART g_Uart;
#define DEBUG_PRINTF(...)		g_Uart.printf(__VA_ARGS__)
#else
#define DEBUG_PRINTF(...)
#endif
/*******************************/

uint32_t BtAttProcessError(uint16_t ConnHdl, BtAttReqRsp_t * const pRspAtt, int RspLen)
{
	uint32_t retval = 0;

	// Find the peer record for this connection. The state machine writes
	// services/chars into pPeer->Services and fires next requests via
	// pPeer->pHciDev / pPeer->ConnHdl. The discovery cursor lives on
	// pPeer->Discovery so concurrent links do not clobber each other.
	BtDevice_t *pPeer = BtAppPeerFindByHdl(ConnHdl);
	if (pPeer == NULL)
	{
		return retval;
	}

	switch (pRspAtt->ErrorRsp.ReqOpCode)
	{
	case BT_ATT_OPCODE_ATT_READ_BY_GROUP_TYPE_REQ:
	{
		if (pRspAtt->ErrorRsp.Error == BT_ATT_ERROR_ATT_NOT_FOUND)
		{
			DEBUG_PRINTF("ATT not found at StartHdl = %d\r\n", pRspAtt->ErrorRsp.Hdl);
			DEBUG_PRINTF("Total found BLE services = %d \r\n", pPeer->NbSrvc);

			for (int i=0; i < pPeer->NbSrvc; i++)
			{
				DEBUG_PRINTF("Service #%d: ", i);
				BtGattDBSrvc_t *p = &pPeer->Services[i];
				DEBUG_PRINTF("StartHdl = %d, EndHdl = %d\r\n", p->handle_range.StartHdl, p->handle_range.EndHdl);
			}

			if (pPeer->NbSrvc == 0)
			{
				DEBUG_PRINTF("No services discovered, nothing to scan\r\n");
				break;
			}

			// Look into the first BLE Srvc
			pPeer->Discovery.SrvIdx = 0;
			BtGattDBSrvc_t *pSrvc = &pPeer->Services[pPeer->Discovery.SrvIdx];
			pPeer->Discovery.CharIdx = 0;
			pPeer->Discovery.Hdl = pSrvc->handle_range.StartHdl;
			if (pPeer->Discovery.Hdl > pSrvc->handle_range.EndHdl)
			{
				DEBUG_PRINTF("Parsing Hdl (%d) larger than EndHdl (%d)\r\n", pPeer->Discovery.Hdl, pSrvc->handle_range.EndHdl);
			}
			else
			{
				uint16_t sHdl = pSrvc->handle_range.StartHdl;
				uint16_t eHdl = pSrvc->handle_range.EndHdl;

				pPeer->Discovery.UuidType.BaseIdx = 0;
				pPeer->Discovery.UuidType.Type    = BT_UUID_TYPE_16;
				pPeer->Discovery.UuidType.Uuid16  = BT_UUID_DECLARATIONS_CHARACTERISTIC;

				DEBUG_PRINTF(
						"Parse the characteristic of the first service ConnHdl %d, sHdl %d, eHdl %d, uuid 0x%X, baseIdx %d\r\n",
						pPeer->ConnHdl, sHdl, eHdl,
						pPeer->Discovery.UuidType.Uuid16,
						pPeer->Discovery.UuidType.BaseIdx);

				BtAttReadByTypeRequest((BtHciDevice_t*) pPeer->pHciDev, pPeer->ConnHdl, sHdl, eHdl, &pPeer->Discovery.UuidType);
			}
		}
	}
		break;
	case BT_ATT_OPCODE_ATT_READ_BY_TYPE_REQ:
	{
		if (pRspAtt->ErrorRsp.Error == BT_ATT_ERROR_ATT_NOT_FOUND)
		{
			DEBUG_PRINTF("ATT with sHdl %d of SrvcIdx %d not found \r\n", pPeer->Discovery.Hdl, pPeer->Discovery.SrvIdx);

			if (pPeer->Discovery.SrvIdx >= pPeer->NbSrvc - 1)
			{
				DEBUG_PRINTF("Out of number of services. Restart to scan for different Uuid16 Type\r\n");
				pPeer->Discovery.SrvIdx = 0;
				pPeer->Discovery.CharIdx = 0;
				BtGattDBSrvc_t *pSrvc = &pPeer->Services[pPeer->Discovery.SrvIdx];
				pPeer->Discovery.Hdl = pSrvc->handle_range.StartHdl;
				uint16_t sHdl = pSrvc->handle_range.StartHdl;
				uint16_t eHdl = pSrvc->handle_range.EndHdl;

				switch (pPeer->Discovery.UuidType.Uuid16)
				{
				case BT_UUID_DECLARATIONS_CHARACTERISTIC:
				{
					DEBUG_PRINTF("Characteristic scan done. Summary:\r\n");
					for (int i = 0; i < pPeer->NbSrvc; i++)
						DEBUG_PRINTF("    SrvcIdx %d has %d characteristics\r\n", i, pPeer->Services[i].char_count);

					DEBUG_PRINTF("Start Scanning for BT_UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION 0x2902\r\n");
					pPeer->Discovery.UuidType.Uuid16 = BT_UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION;
				}
					break;
				case BT_UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION:
				{
					DEBUG_PRINTF("Start Scanning for BT_UUID_DESCRIPTOR_CHARACTERISTIC_USER_DESCRIPTION 0x2901\r\n");
					pPeer->Discovery.UuidType.Uuid16 = BT_UUID_DESCRIPTOR_CHARACTERISTIC_USER_DESCRIPTION;
				}
					break;
				default:
					DEBUG_PRINTF("Unprocess Uuid16 Type (code 0x%X)\r\n", pPeer->Discovery.UuidType.Uuid16);
				}

				DEBUG_PRINTF("SrvcIdx %d, CharIdx %d, sHdl %d, eHdl %d \r\n", pPeer->Discovery.SrvIdx, pPeer->Discovery.CharIdx, sHdl, eHdl);
				BtAttReadByTypeRequest((BtHciDevice_t*) pPeer->pHciDev, pPeer->ConnHdl, sHdl, eHdl, &pPeer->Discovery.UuidType);
			}
			else
			{
				DEBUG_PRINTF("Parse the next BLE SrvcIdx %d\r\n", pPeer->Discovery.SrvIdx + 1);
				pPeer->Discovery.SrvIdx++;
				pPeer->Discovery.CharIdx = 0;
				BtGattDBSrvc_t *pSrvc = &pPeer->Services[pPeer->Discovery.SrvIdx];
				pPeer->Discovery.Hdl = pSrvc->handle_range.StartHdl;
				uint16_t sHdl = pSrvc->handle_range.StartHdl;
				uint16_t eHdl = pSrvc->handle_range.EndHdl;
				DEBUG_PRINTF("sHdl %d, eHdl %d \r\n", sHdl, eHdl);

				BtAttReadByTypeRequest((BtHciDevice_t*) pPeer->pHciDev, pPeer->ConnHdl, sHdl, eHdl, &pPeer->Discovery.UuidType);
			}
		}
	}
		break;
	case BT_ATT_OPCODE_ATT_READ_REQ:
	{
		if (pRspAtt->ErrorRsp.Error == BT_ATT_ERROR_INVALID_HANDLE)
		{
			DEBUG_PRINTF("BT_ATT_OPCODE_ATT_READ_REQ (0x0A) error: invalid handle\r\n");
		}
	}
		break;
	default:
		break;
	}

	return retval;
}

void BtAttProcessRsp(uint16_t ConnHdl, BtAttReqRsp_t * const pRspAtt, int RspLen)
{
//	DEBUG_PRINTF("BtAttProcessRsp: Opcode 0x%x, RspLen = %d \r\n",
//			pRspAtt->OpCode, RspLen);

	// Route the response to the connected peer's record. The state machine
	// reads/writes pPeer->Services and fires next requests via pPeer->pHciDev
	// / pPeer->ConnHdl. The cursor lives on pPeer->Discovery.
	BtDevice_t *pPeer = BtAppPeerFindByHdl(ConnHdl);
	if (pPeer == NULL)
	{
		return;
	}

	switch (pRspAtt->OpCode)
	{
	case BT_ATT_OPCODE_ATT_ERROR_RSP:
	{
		DEBUG_PRINTF(
				"BT_ATT_OPCODE_ATT_ERROR_RSP (0x01): OpCode 0x%x, ErrCode 0x%x \r\n",
				pRspAtt->ErrorRsp.ReqOpCode, pRspAtt->ErrorRsp.Error);

		// Error code processing
		BtAttProcessError(ConnHdl, pRspAtt, RspLen);
	}
		break;
	case BT_ATT_OPCODE_ATT_EXCHANGE_MTU_RSP:
	{
		uint16_t mtu = min(BtAttGetMtu(), pRspAtt->ExchgMtuReqRsp.RxMtu);
		DEBUG_PRINTF("BT_ATT_OPCODE_ATT_EXCHANGE_MTU_RSP (0x03): %d %d\r\n", pRspAtt->ExchgMtuReqRsp.RxMtu, mtu);
		BtAttSetMtu(mtu);
	}
		break;
	case BT_ATT_OPCODE_ATT_FIND_INFORMATION_RSP:
	{
		DEBUG_PRINTF("BT_ATT_OPCODE_ATT_FIND_INFORMATION_RSP (0x05)\r\n");
		BtAttFindInfoRsp_t *p = (BtAttFindInfoRsp_t*) &pRspAtt->FindInfoRsp;
		DEBUG_PRINTF("Format %d\r\n", p->Fmt);

		// Payload after opcode + format byte. Fmt = 1: uuid16 entries (4
		// bytes each = handle + uuid16). Fmt = 2: uuid128 entries (18
		// bytes each = handle + uuid128). Previously this used RspLen<=3
		// to pick format which was wrong - RspLen <= 3 means an empty
		// response payload.
		int payloadLen = RspLen - 2;

		if (p->Fmt == 1)
		{
			int entrySize = sizeof(BtAttHdlUuid16_t);
			for (int off = 0; off + entrySize <= payloadLen; off += entrySize)
			{
				BtAttHdlUuid16_t *e =
					(BtAttHdlUuid16_t*) ((uint8_t*) p->HdlUuid16 + off);
				DEBUG_PRINTF("Hdl %d Uuid16 0x%04X\r\n", e->Hdl, e->Uuid);
			}
		}
		else if (p->Fmt == 2)
		{
			int entrySize = sizeof(BtAttHdlUuid128_t);
			for (int off = 0; off + entrySize <= payloadLen; off += entrySize)
			{
				BtAttHdlUuid128_t *e =
					(BtAttHdlUuid128_t*) ((uint8_t*) p->HdlUuid128 + off);
				DEBUG_PRINTF("Hdl %d Uuid128 (hex): ", e->Hdl);
				for (int i = 0; i < 16; i++)
					DEBUG_PRINTF("%02X ", e->Uuid[i]);
				DEBUG_PRINTF("\r\n");
			}
		}
		else
		{
			DEBUG_PRINTF("FIND_INFO_RSP: unknown format %d\r\n", p->Fmt);
		}
	}
		break;
	case BT_ATT_OPCODE_ATT_FIND_BY_TYPE_VALUE_RSP:
	{
		DEBUG_PRINTF("BT_ATT_OPCODE_ATT_FIND_BY_TYPE_VALUE_RSP (0x07)\r\n");
	}
		break;
	case BT_ATT_OPCODE_ATT_READ_BY_TYPE_RSP:
	{
		DEBUG_PRINTF("BT_ATT_OPCODE_ATT_READ_BY_TYPE_RSP (0x09)\r\n");
		BtAttReadByTypeRsp_t *p = (BtAttReadByTypeRsp_t *) &pRspAtt->ReadByTypeRsp;
		DEBUG_PRINTF("Per-tuple Len %d, RspLen %d\r\n", p->Len, RspLen);

		// RspLen counts opcode + length-byte + payload, so the payload
		// available for tuples is RspLen - 2. Each tuple is exactly p->Len
		// bytes; a response packs as many as fit in the ATT_MTU.
		int payloadLen = RspLen - 2;
		if (p->Len == 0 || payloadLen < p->Len)
		{
			DEBUG_PRINTF("READ_BY_TYPE_RSP: malformed (Len=%d, payload=%d)\r\n",
			             p->Len, payloadLen);
			break;
		}

		// Cache the cursor for state-machine advancement after the loop.
		// Advance the start handle past the LAST tuple's decl handle,
		// not the first - a previous bug here meant repeat-queries on the
		// same range whenever multiple tuples were returned.
		uint16_t lastDeclHdl = 0;

		switch (pPeer->Discovery.UuidType.Uuid16)
		{
		case BT_UUID_DECLARATIONS_CHARACTERISTIC:
		{
			DEBUG_PRINTF("Response BT_UUID_DECLARATIONS_CHARACTERISTIC 0x2803\r\n");

			for (int off = 0; off + p->Len <= payloadLen; off += p->Len)
			{
				// Bound the per-service char index against the static array
				// in BtGattDBSrvc_t. Without this, a peer advertising more
				// than BT_GATT_DB_MAX_CHARS chars in a single service would
				// overrun pPeer->Services[].characteristics[].
				if (pPeer->Discovery.CharIdx >= BT_GATT_DB_MAX_CHARS)
				{
					DEBUG_PRINTF("Char table full for SrvcIdx %d (cap %d), dropping rest\r\n",
					             pPeer->Discovery.SrvIdx, BT_GATT_DB_MAX_CHARS);
					break;
				}

				uint16_t Hdl        = p->Data[off+0] | (p->Data[off+1] << 8);
				uint8_t  CharProp   = p->Data[off+2];
				uint16_t CharHdlVal = p->Data[off+3] | (p->Data[off+4] << 8);

				BtGattDBChar_t *pChar =
					&pPeer->Services[pPeer->Discovery.SrvIdx]
					     .characteristics[pPeer->Discovery.CharIdx];
				memcpy((uint8_t*) &pChar->characteristic.char_props, &CharProp,
				       sizeof(BtGattCharProps_t));
				pChar->characteristic.handle_value = CharHdlVal;

				if (p->Len <= 7) // 16-bit UUID tuple: 5 fixed + 2 uuid = 7
				{
					pChar->characteristic.uuid.Uuid = p->Data[off+5]
					                                 | (p->Data[off+6] << 8);
					pChar->characteristic.uuid.BaseIdx = 0;
					pChar->characteristic.uuid.Type = BT_UUID_TYPE_16;
				}
				else if (p->Len <= 9) // 32-bit UUID? not standard - ignored
				{
				}
				else // 128-bit UUID tuple: 5 fixed + 16 uuid = 21
				{
					// Work on a copy so the wire buffer is not mutated.
					uint8_t base[16];
					memcpy(base, &p->Data[off+5], 16);
					uint16_t CharUuid16 = base[12] | (base[13] << 8);
					DEBUG_PRINTF("128-bit UUID with UUID16 0x%X\r\n", CharUuid16);
					base[12] = 0;
					base[13] = 0;
					int idx = BtUuidFindBase(base);
					pChar->characteristic.uuid.BaseIdx = (idx >= 0) ? (uint8_t)idx : 0;
					pChar->characteristic.uuid.Uuid    = CharUuid16;
					pChar->characteristic.uuid.Type    = BT_UUID_TYPE_16;
				}

				DEBUG_PRINTF("SrvcIdx %d CharIdx %d: Hdl %d Prop 0x%X ValHdl %d Uuid16 0x%X\r\n",
				             pPeer->Discovery.SrvIdx, pPeer->Discovery.CharIdx, Hdl,
				             pChar->characteristic.char_props,
				             pChar->characteristic.handle_value,
				             pChar->characteristic.uuid.Uuid);

				pPeer->Discovery.CharIdx++;
				pPeer->Services[pPeer->Discovery.SrvIdx].char_count++;
				lastDeclHdl = Hdl;
			}

			// Advance the search past the last consumed declaration. If
			// there's room in the service handle range, fire another
			// READ_BY_TYPE request to get more characteristics.
			uint16_t eHdl =
				pPeer->Services[pPeer->Discovery.SrvIdx].handle_range.EndHdl;
			if (lastDeclHdl != 0 && (uint32_t)lastDeclHdl + 1 <= eHdl)
			{
				pPeer->Discovery.Hdl = lastDeclHdl + 1;
				pPeer->Discovery.UuidType.BaseIdx = 0;
				pPeer->Discovery.UuidType.Type    = BT_UUID_TYPE_16;
				BtAttReadByTypeRequest((BtHciDevice_t*) pPeer->pHciDev,
				                       pPeer->ConnHdl, pPeer->Discovery.Hdl,
				                       eHdl, &pPeer->Discovery.UuidType);
			}
		}
			break;
		case BT_UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION:
		{
			DEBUG_PRINTF("Response BT_UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION 0x2902\r\n");
			DEBUG_PRINTF("SrvcIdx %d CharIdx %d CurHdl %d\r\n",
			             pPeer->Discovery.SrvIdx, pPeer->Discovery.CharIdx, pPeer->Discovery.Hdl);

			// One CCCD tuple per response is the common case but the
			// spec permits multiple - iterate. CCCDs are uuid16 only so
			// every tuple is the same length (4 bytes: 2 handle + 2 uuid).
			uint16_t lastCccdHdl = 0;
			for (int off = 0; off + p->Len <= payloadLen; off += p->Len)
			{
				if (pPeer->Discovery.CharIdx >= BT_GATT_DB_MAX_CHARS)
				{
					DEBUG_PRINTF("CCCD scan past char cap for SrvcIdx %d\r\n",
					             pPeer->Discovery.SrvIdx);
					break;
				}
				uint16_t Hdl = p->Data[off+0] | (p->Data[off+1] << 8);
				BtGattDBChar_t *pChar =
					&pPeer->Services[pPeer->Discovery.SrvIdx]
					     .characteristics[pPeer->Discovery.CharIdx];
				if (pChar->characteristic.char_props.notify
				 || pChar->characteristic.char_props.indicate)
				{
					pChar->cccd_handle = Hdl;
					DEBUG_PRINTF("SrvcIdx %d CharIdx %d ValHdl %d CccdHdl %d\r\n",
					             pPeer->Discovery.SrvIdx, pPeer->Discovery.CharIdx,
					             pChar->characteristic.handle_value,
					             pChar->cccd_handle);
				}
				lastCccdHdl = Hdl;
			}

			bool bNextSrvc = false;
			pPeer->Discovery.CharIdx++;
			if (pPeer->Discovery.CharIdx < pPeer->Services[pPeer->Discovery.SrvIdx].char_count)
			{
				DEBUG_PRINTF("Scan Next CharIdx %d\r\n", pPeer->Discovery.CharIdx);
				uint16_t eHdl =
					pPeer->Services[pPeer->Discovery.SrvIdx].handle_range.EndHdl;
				if (lastCccdHdl != 0 && (uint32_t)lastCccdHdl + 1 <= eHdl)
				{
					pPeer->Discovery.Hdl = lastCccdHdl + 1;
					BtAttReadByTypeRequest((BtHciDevice_t*) pPeer->pHciDev,
					                       pPeer->ConnHdl, pPeer->Discovery.Hdl,
					                       eHdl, &pPeer->Discovery.UuidType);
				}
				else
				{
					bNextSrvc = true;
				}
			}
			else
			{
				bNextSrvc = true;
			}

			// Look into the next BLE service
			if (bNextSrvc)
			{
				pPeer->Discovery.SrvIdx++;
				if (pPeer->Discovery.SrvIdx < pPeer->NbSrvc)
				{
					DEBUG_PRINTF("Scan Next SrvcIdx %d\r\n", pPeer->Discovery.SrvIdx);
					pPeer->Discovery.Hdl =
						pPeer->Services[pPeer->Discovery.SrvIdx].handle_range.StartHdl;
					uint16_t eHdl =
						pPeer->Services[pPeer->Discovery.SrvIdx].handle_range.EndHdl;
					BtAttReadByTypeRequest((BtHciDevice_t*) pPeer->pHciDev,
					                       pPeer->ConnHdl, pPeer->Discovery.Hdl,
					                       eHdl, &pPeer->Discovery.UuidType);
				}
				else
				{
					DEBUG_PRINTF("Out of number of service. Start parsing other UUID type\r\n");
				}
			}
		}
			break;
		case BT_UUID_DESCRIPTOR_CHARACTERISTIC_USER_DESCRIPTION:
		{
			DEBUG_PRINTF("Response BT_UUID_DESCRIPTOR_CHARACTERISTIC_USER_DESCRIPTION 0x2901\r\n");
			uint16_t hdl = p->Data[0] | (p->Data[1] << 8);
			DEBUG_PRINTF("Hdl = %d\r\n", hdl);
			DEBUG_PRINTF("Data (hex): ");
			for (int i = 2; i < p->Len; i++)
				DEBUG_PRINTF("%X ", p->Data[i]);
			DEBUG_PRINTF("\r\n");
		}
			break;
		default:
			DEBUG_PRINTF("Unprocess Uuid16 Type (code 0x%X)\r\n", pPeer->Discovery.UuidType.Uuid16);
		}
	}
	break;
	case BT_ATT_OPCODE_ATT_READ_RSP:
	{
		DEBUG_PRINTF("BT_ATT_OPCODE_ATT_READ_RSP (0x0B), RspLen = %d \r\n", RspLen);
		BtAttReadRsp_t *p = (BtAttReadRsp_t *) &pRspAtt->ReadRsp;

		DEBUG_PRINTF("Raw Data (hex): ");
		for (int i = 0; i < RspLen; i++)
		{
			DEBUG_PRINTF("%x ", p->Data[i]);
		}
		DEBUG_PRINTF("\r\n");

		BtGattDBSrvc_t *pSrvc = &pPeer->Services[pPeer->Discovery.SrvIdx];

		// Parse the next handle
		pPeer->Discovery.Hdl++;
		if (pPeer->Discovery.Hdl > pSrvc->handle_range.EndHdl)
		{
			// Jump to the next BLE service handle range
			if (pPeer->Discovery.SrvIdx < (pPeer->NbSrvc - 1))
			{
				pPeer->Discovery.SrvIdx++;
				DEBUG_PRINTF("Next service idx = %d, UUID16 = 0x%X\r\n",
						pPeer->Discovery.SrvIdx, pSrvc->srv_uuid.Uuid);
				pSrvc = &pPeer->Services[pPeer->Discovery.SrvIdx];
				pPeer->Discovery.Hdl = pSrvc->handle_range.StartHdl + 1; // ignore the first handle, which corresponds to service handle

				BtAttReadRequest((BtHciDevice_t *)pPeer->pHciDev, pPeer->ConnHdl, pPeer->Discovery.Hdl);
			}
			else
			{
				DEBUG_PRINTF("All handles were scanned\r\n");
			}
		}
		else
		{
			DEBUG_PRINTF("Next Discovery.Hdl = %d \r\n", pPeer->Discovery.Hdl);
			BtAttReadRequest((BtHciDevice_t *)pPeer->pHciDev, pPeer->ConnHdl, pPeer->Discovery.Hdl);
		}
	}
		break;
	case BT_ATT_OPCODE_ATT_READ_BLOB_RSP:
	{
		DEBUG_PRINTF("BT_ATT_OPCODE_ATT_READ_BLOB_RSP (0x0D) \r\n");
	}
		break;
	case BT_ATT_OPCODE_ATT_READ_MULTIPLE_RSP:
	{
		DEBUG_PRINTF("BT_ATT_OPCODE_ATT_READ_MULTIPLE_RSP (0x0F) \r\n");
	}
		break;
	case BT_ATT_OPCODE_ATT_READ_BY_GROUP_TYPE_RSP:
	{
		DEBUG_PRINTF("BT_ATT_OPCODE_ATT_READ_BY_GROUP_TYPE_RSP (0x11)\r\n");
		BtAttReadByGroupTypeRsp_t *p =
				(BtAttReadByGroupTypeRsp_t*) &pRspAtt->ReadByGroupTypeRsp;
		DEBUG_PRINTF("Per-tuple Len %d, RspLen %d\r\n", p->Len, RspLen);

		// Payload available for tuples = RspLen - 2 (opcode + length byte).
		// Each tuple is exactly p->Len bytes; the response packs as many
		// services as fit within the ATT_MTU.
		int payloadLen = RspLen - 2;
		if (p->Len == 0 || payloadLen < p->Len)
		{
			DEBUG_PRINTF("READ_BY_GROUP_TYPE_RSP: malformed\r\n");
			break;
		}

		uint16_t lastEndHdl = 0;

		// p->Len discriminates UUID width: 6 = uuid16 tuple, 20 = uuid128.
		for (int off = 0; off + p->Len <= payloadLen; off += p->Len)
		{
			if (pPeer->NbSrvc >= BT_DEV_SERVICE_MAXCNT)
			{
				DEBUG_PRINTF("Service table full (cap %d), dropping rest\r\n",
				             BT_DEV_SERVICE_MAXCNT);
				break;
			}
			uint8_t SrvcIdx = pPeer->NbSrvc++;
			BtGattDBSrvc_t *pSrvc = &pPeer->Services[SrvcIdx];

			if (p->Len <= 6) // UUID16 tuple: 2+2+2 = 6
			{
				BtAttReadByGroupTypeRspUuid16_t *g =
					(BtAttReadByGroupTypeRspUuid16_t*) &p->Data[off];

				pSrvc->handle_range.StartHdl = g->HdlStart;
				pSrvc->handle_range.EndHdl   = g->HdlEnd;
				pSrvc->srv_uuid.Uuid    = g->Uuid;
				pSrvc->srv_uuid.BaseIdx = 0;
				pSrvc->srv_uuid.Type    = BT_UUID_TYPE_16;
				lastEndHdl              = g->HdlEnd;

				DEBUG_PRINTF("Srvc[%d] uuid16 0x%04X hdl %d..%d\r\n",
				             SrvcIdx, g->Uuid, g->HdlStart, g->HdlEnd);
			}
			else // UUID128 tuple: 2+2+16 = 20
			{
				BtAttReadByGroupTypeRspUuid128_t *g =
					(BtAttReadByGroupTypeRspUuid128_t*) &p->Data[off];

				pSrvc->handle_range.StartHdl = g->HdlStart;
				pSrvc->handle_range.EndHdl   = g->HdlEnd;
				// BtUuid128To16 no longer mutates its input (see bt_uuid.cpp),
				// so the previous g->Uuid[12]=0; g->Uuid[13]=0; belt-and-suspenders
				// is gone.
				int idx = BtUuid128To16(&pSrvc->srv_uuid, g->Uuid);
				lastEndHdl = g->HdlEnd;

				DEBUG_PRINTF("Srvc[%d] uuid128 (base idx %d) uuid16 0x%X hdl %d..%d\r\n",
				             SrvcIdx, idx, pSrvc->srv_uuid.Uuid,
				             g->HdlStart, g->HdlEnd);
			}
		}

		// Continue reading the next group if not yet at the end of the
		// attribute table. Uses the LAST consumed service's end handle,
		// not the first - a previous bug here would re-query the same
		// range forever when the response carried multiple services.
		if (lastEndHdl != 0 && lastEndHdl != 0xFFFF)
		{
			uint16_t NextStartHdl = lastEndHdl + 1;
			DEBUG_PRINTF("Read next group from sHdl = %d\r\n", NextStartHdl);
			BtUuid_t Uuid = {
				.BaseIdx = 0,
				.Type    = BT_UUID_TYPE_16,
				.Uuid16  = BT_UUID_DECLARATIONS_PRIMARY_SERVICE,
			};
			BtAttReadByGroupTypeRequest((BtHciDevice_t*) pPeer->pHciDev,
			                            pPeer->ConnHdl, NextStartHdl,
			                            0xFFFF, &Uuid);
		}
	}
		break;
	case BT_ATT_OPCODE_ATT_WRITE_RSP:
	{
		DEBUG_PRINTF("BT_ATT_OPCODE_ATT_WRITE_RSP (0x13) \r\n");
	}
		break;
	case BT_ATT_OPCODE_ATT_EXECUTE_WRITE_RSP:
	{
		DEBUG_PRINTF("BT_ATT_OPCODE_ATT_EXECUTE_WRITE_RSP (0x19) \r\n");
	}
		break;
	case BT_ATT_OPCODE_ATT_READ_MULTIPLE_VARIABLE_RSP:
	{
		DEBUG_PRINTF("BT_ATT_OPCODE_ATT_READ_MULTIPLE_VARIABLE_RSP (0x21) \r\n");
	}
		break;
	}
}
