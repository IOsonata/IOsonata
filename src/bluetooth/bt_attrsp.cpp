/**-------------------------------------------------------------------------
@file	bt_attresp.cpp

@brief	Generic Bluetooth ATT protocol
Implementation of ATT command response

@author	Thinh Tran
@date	Apr. 03, 2024

@license

MIT License

Copyright (c) 2024, I-SYST, all rights reserved

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
#include <inttypes.h>
#include <stddef.h>
#include <memory.h>

#include "istddef.h"
#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_l2cap.h"
#include "bluetooth/bt_att.h"

#include "bluetooth/bt_dev.h"
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

extern BtDev_t g_BtDevSdc;
extern BtUuid_t g_UuidType;
CurParseInf_t g_CurIdx;

uint32_t BtAttProcessError(uint16_t ConnHdl, BtAttReqRsp_t * const pRspAtt, int RspLen)
{
	uint32_t retval = 0;

	switch (pRspAtt->ErrorRsp.ReqOpCode)
	{
	case BT_ATT_OPCODE_ATT_READ_BY_GROUP_TYPE_REQ:
	{
		if (pRspAtt->ErrorRsp.Error == BT_ATT_ERROR_ATT_NOT_FOUND)
		{
			DEBUG_PRINTF("ATT not found at StartHdl = %d\r\n", pRspAtt->ErrorRsp.Hdl);
			DEBUG_PRINTF("Total found BLE services = %d \r\n", g_BtDevSdc.NbSrvc);

			for (int i=0; i < g_BtDevSdc.NbSrvc; i++)
			{
				DEBUG_PRINTF("Service #%d: ", i);
				BtGattDBSrvc_t *p = (BtGattDBSrvc_t *) &g_BtDevSdc.Services[i];
				DEBUG_PRINTF("StartHdl = %d, EndHdl = %d\r\n", p->handle_range.StartHdl, p->handle_range.EndHdl);
			}

			// Look into the first BLE Srvc
			g_CurIdx.SrvIdx = 0;
			BtGattDBSrvc_t *pSrvc = (BtGattDBSrvc_t*) &g_BtDevSdc.Services[g_CurIdx.SrvIdx];
			g_CurIdx.CharIdx = 0;
			g_CurIdx.Hdl = pSrvc->handle_range.StartHdl;
			if (g_CurIdx.Hdl > pSrvc->handle_range.EndHdl)
			{
				DEBUG_PRINTF("Parsing Hdl (%d) larger than EndHdl (%d)\r\n", g_CurIdx.Hdl, pSrvc->handle_range.EndHdl);
			}
			else
			{
				uint16_t sHdl = pSrvc->handle_range.StartHdl;
				uint16_t eHdl = pSrvc->handle_range.EndHdl;

				//g_UuidType.BaseIdx = 0;
				//g_UuidType.Type = BT_UUID_TYPE_16;
				g_UuidType.Uuid16 = BT_UUID_DECLARATIONS_CHARACTERISTIC;

				DEBUG_PRINTF(
						"Parse the characteristic of the first service ConnHdl %d, sHdl %d, eHdl %d, uuid 0x%X, baseIdx %d\r\n",
						g_BtDevSdc.ConnHdl, sHdl, eHdl, g_UuidType.Uuid16, g_UuidType.BaseIdx);

				BtAttReadByTypeRequest((BtHciDevice_t*) g_BtDevSdc.pHciDev, g_BtDevSdc.ConnHdl, sHdl, eHdl, &g_UuidType);
			}
		}
	}
		break;
	case BT_ATT_OPCODE_ATT_READ_BY_TYPE_REQ:
	{
		if (pRspAtt->ErrorRsp.Error == BT_ATT_ERROR_ATT_NOT_FOUND)
		{
			DEBUG_PRINTF("ATT with sHdl %d of SrvcIdx %d not found \r\n", g_CurIdx.Hdl, g_CurIdx.SrvIdx);

			if (g_CurIdx.SrvIdx >= g_BtDevSdc.NbSrvc - 1)
			{
				DEBUG_PRINTF("Out of number of services. Restart to scan for different Uuid16 Type\r\n");
				g_CurIdx.SrvIdx = 0;
				g_CurIdx.CharIdx = 0;
				BtGattDBSrvc_t *pSrvc = (BtGattDBSrvc_t*) &g_BtDevSdc.Services[g_CurIdx.SrvIdx];
				g_CurIdx.Hdl = pSrvc->handle_range.StartHdl;
				uint16_t sHdl = pSrvc->handle_range.StartHdl;
				uint16_t eHdl = pSrvc->handle_range.EndHdl;

				//g_UuidType.BaseIdx = 0;
				//g_UuidType.Type = BT_UUID_TYPE_16;

				switch (g_UuidType.Uuid16)
				{
				case BT_UUID_DECLARATIONS_CHARACTERISTIC:
				{
					DEBUG_PRINTF("Characteristic scan done. Summary:\r\n");
					for (int i = 0; i < g_BtDevSdc.NbSrvc; i++)
						DEBUG_PRINTF("    SrvcIdx %d has %d characteristics\r\n", i, g_BtDevSdc.Services[i].char_count);

					DEBUG_PRINTF("Start Scanning for BT_UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION 0x2902\r\n");
					g_UuidType.Uuid16 = BT_UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION;
				}
					break;
				case BT_UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION:
				{
					DEBUG_PRINTF("Start Scanning for BT_UUID_DESCRIPTOR_CHARACTERISTIC_USER_DESCRIPTION 0x2901\r\n");
					g_UuidType.Uuid16 = BT_UUID_DESCRIPTOR_CHARACTERISTIC_USER_DESCRIPTION;
				}
					break;
				default:
					DEBUG_PRINTF("Unprocess Uuid16 Type (code 0x%X)\r\n", g_UuidType.Uuid16);
				}

				DEBUG_PRINTF("SrvcIdx %d, CharIdx %d, sHdl %d, eHdl %d \r\n", g_CurIdx.SrvIdx, g_CurIdx.CharIdx, sHdl, eHdl);
				BtAttReadByTypeRequest((BtHciDevice_t*) g_BtDevSdc.pHciDev, g_BtDevSdc.ConnHdl, sHdl, eHdl, &g_UuidType);
			}
			else
			{
				DEBUG_PRINTF("Parse the next BLE SrvcIdx %d\r\n", g_CurIdx.CharIdx + 1);
				g_CurIdx.SrvIdx++;
				g_CurIdx.CharIdx = 0;
				BtGattDBSrvc_t *pSrvc = (BtGattDBSrvc_t*) &g_BtDevSdc.Services[g_CurIdx.SrvIdx];
				g_CurIdx.Hdl = pSrvc->handle_range.StartHdl;
				uint16_t sHdl = pSrvc->handle_range.StartHdl;
				uint16_t eHdl = pSrvc->handle_range.EndHdl;
				DEBUG_PRINTF("sHdl %d, eHdl %d \r\n", sHdl, eHdl);

				//g_UuidType.BaseIdx = 0;
				//g_UuidType.Type = BT_UUID_TYPE_16;
				BtAttReadByTypeRequest((BtHciDevice_t*) g_BtDevSdc.pHciDev, g_BtDevSdc.ConnHdl, sHdl, eHdl, &g_UuidType);
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
		// We advance the start handle past the LAST tuple's decl handle,
		// not the first - a previous bug here meant repeat-queries on the
		// same range whenever multiple tuples were returned.
		uint16_t lastDeclHdl = 0;

		switch (g_UuidType.Uuid16)
		{
		case BT_UUID_DECLARATIONS_CHARACTERISTIC:
		{
			DEBUG_PRINTF("Response BT_UUID_DECLARATIONS_CHARACTERISTIC 0x2803\r\n");

			for (int off = 0; off + p->Len <= payloadLen; off += p->Len)
			{
				uint16_t Hdl        = p->Data[off+0] | (p->Data[off+1] << 8);
				uint8_t  CharProp   = p->Data[off+2];
				uint16_t CharHdlVal = p->Data[off+3] | (p->Data[off+4] << 8);

				BtGattDBChar_t *pChar =
					(BtGattDBChar_t*) &g_BtDevSdc.Services[g_CurIdx.SrvIdx]
					                    .charateristics[g_CurIdx.CharIdx];
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
					uint8_t *pUuid128 = (uint8_t*) &p->Data[off+5];
					uint16_t CharUuid16 = pUuid128[12] | (pUuid128[13] << 8);
					DEBUG_PRINTF("128-bit UUID with UUID16 0x%X\r\n", CharUuid16);
					pUuid128[12] = 0;
					pUuid128[13] = 0;
					int idx = BtUuidFindBase(pUuid128);
					pChar->characteristic.uuid.BaseIdx = idx;
					pChar->characteristic.uuid.Uuid   = CharUuid16;
					pChar->characteristic.uuid.Type   = BT_UUID_TYPE_128;
				}

				DEBUG_PRINTF("SrvcIdx %d CharIdx %d: Hdl %d Prop 0x%X ValHdl %d Uuid16 0x%X\r\n",
				             g_CurIdx.SrvIdx, g_CurIdx.CharIdx, Hdl,
				             pChar->characteristic.char_props,
				             pChar->characteristic.handle_value,
				             pChar->characteristic.uuid.Uuid);

				g_CurIdx.CharIdx++;
				g_BtDevSdc.Services[g_CurIdx.SrvIdx].char_count++;
				lastDeclHdl = Hdl;
			}

			// Advance the search past the last consumed declaration. If
			// there's room in the service handle range, fire another
			// READ_BY_TYPE request to get more characteristics.
			uint16_t eHdl =
				g_BtDevSdc.Services[g_CurIdx.SrvIdx].handle_range.EndHdl;
			if (lastDeclHdl != 0 && (uint32_t)lastDeclHdl + 1 <= eHdl)
			{
				g_CurIdx.Hdl = lastDeclHdl + 1;
				g_UuidType.BaseIdx = 0;
				g_UuidType.Type    = BT_UUID_TYPE_16;
				BtAttReadByTypeRequest((BtHciDevice_t*) g_BtDevSdc.pHciDev,
				                       g_BtDevSdc.ConnHdl, g_CurIdx.Hdl,
				                       eHdl, &g_UuidType);
			}
		}
			break;
		case BT_UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION:
		{
			DEBUG_PRINTF("Response BT_UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION 0x2902\r\n");
			DEBUG_PRINTF("SrvcIdx %d CharIdx %d CurHdl %d\r\n",
			             g_CurIdx.SrvIdx, g_CurIdx.CharIdx, g_CurIdx.Hdl);

			// One CCCD tuple per response is the common case but the
			// spec permits multiple - iterate. CCCDs are uuid16 only so
			// every tuple is the same length (4 bytes: 2 handle + 2 uuid).
			uint16_t lastCccdHdl = 0;
			for (int off = 0; off + p->Len <= payloadLen; off += p->Len)
			{
				uint16_t Hdl = p->Data[off+0] | (p->Data[off+1] << 8);
				BtGattDBChar_t *pChar =
					(BtGattDBChar_t*) &g_BtDevSdc.Services[g_CurIdx.SrvIdx]
					                    .charateristics[g_CurIdx.CharIdx];
				if (pChar->characteristic.char_props.notify
				 || pChar->characteristic.char_props.indicate)
				{
					pChar->cccd_handle = Hdl;
					DEBUG_PRINTF("SrvcIdx %d CharIdx %d ValHdl %d CccdHdl %d\r\n",
					             g_CurIdx.SrvIdx, g_CurIdx.CharIdx,
					             pChar->characteristic.handle_value,
					             pChar->cccd_handle);
				}
				lastCccdHdl = Hdl;
			}

			bool bNextSrvc = false;
			g_CurIdx.CharIdx++;
			if (g_CurIdx.CharIdx < g_BtDevSdc.Services[g_CurIdx.SrvIdx].char_count)
			{
				DEBUG_PRINTF("Scan Next CharIdx %d\r\n", g_CurIdx.CharIdx);
				uint16_t eHdl =
					g_BtDevSdc.Services[g_CurIdx.SrvIdx].handle_range.EndHdl;
				if (lastCccdHdl != 0 && (uint32_t)lastCccdHdl + 1 <= eHdl)
				{
					g_CurIdx.Hdl = lastCccdHdl + 1;
					BtAttReadByTypeRequest((BtHciDevice_t*) g_BtDevSdc.pHciDev,
					                       g_BtDevSdc.ConnHdl, g_CurIdx.Hdl,
					                       eHdl, &g_UuidType);
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
				g_CurIdx.SrvIdx++;
				if (g_CurIdx.SrvIdx < g_BtDevSdc.NbSrvc)
				{
					DEBUG_PRINTF("Scan Next SrvcIdx %d\r\n", g_CurIdx.SrvIdx);
					g_CurIdx.Hdl =
						g_BtDevSdc.Services[g_CurIdx.SrvIdx].handle_range.StartHdl;
					uint16_t eHdl =
						g_BtDevSdc.Services[g_CurIdx.SrvIdx].handle_range.EndHdl;
					BtAttReadByTypeRequest((BtHciDevice_t*) g_BtDevSdc.pHciDev,
					                       g_BtDevSdc.ConnHdl, g_CurIdx.Hdl,
					                       eHdl, &g_UuidType);
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
			DEBUG_PRINTF("Unprocess Uuid16 Type (code 0x%X)\r\n", g_UuidType.Uuid16);
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

		BtGattDBSrvc_t *pSrvc = (BtGattDBSrvc_t*) &g_BtDevSdc.Services[g_CurIdx.SrvIdx];

		// Parse the next handle
		g_CurIdx.Hdl++;
		if (g_CurIdx.Hdl > pSrvc->handle_range.EndHdl)
		{
			// Jump to the next BLE service handle range
			if (g_CurIdx.SrvIdx < (g_BtDevSdc.NbSrvc - 1))
			{
				g_CurIdx.SrvIdx++;
				DEBUG_PRINTF("Next service idx = %d, UUID16 = 0x%X\r\n",
						g_CurIdx.SrvIdx, pSrvc->srv_uuid.Uuid);
				pSrvc = (BtGattDBSrvc_t*) &g_BtDevSdc.Services[g_CurIdx.SrvIdx];
				g_CurIdx.Hdl = pSrvc->handle_range.StartHdl + 1; // ignore the first handle, which corresponds to service handle

				BtAttReadRequest((BtHciDevice_t *)g_BtDevSdc.pHciDev, g_BtDevSdc.ConnHdl, g_CurIdx.Hdl);
			}
			else
			{
				DEBUG_PRINTF("All handles were scanned\r\n");
			}
		}
		else
		{
			DEBUG_PRINTF("Next g_CurIdx.Hdl = %d \r\n", g_CurIdx.Hdl);
			BtAttReadRequest((BtHciDevice_t *)g_BtDevSdc.pHciDev, g_BtDevSdc.ConnHdl, g_CurIdx.Hdl);
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
			if (g_BtDevSdc.NbSrvc >= BLEPERIPH_DEV_SERVICE_MAXCNT)
			{
				DEBUG_PRINTF("Service table full (cap %d), dropping rest\r\n",
				             BLEPERIPH_DEV_SERVICE_MAXCNT);
				break;
			}
			uint8_t SrvcIdx = g_BtDevSdc.NbSrvc++;
			BtGattDBSrvc_t *pSrvc =
				(BtGattDBSrvc_t*) &g_BtDevSdc.Services[SrvcIdx];

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
				int idx = BtUuid128To16(&pSrvc->srv_uuid, g->Uuid);
				g->Uuid[12] = 0;
				g->Uuid[13] = 0;
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
			BtAttReadByGroupTypeRequest((BtHciDevice_t*) g_BtDevSdc.pHciDev,
			                            g_BtDevSdc.ConnHdl, NextStartHdl,
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
