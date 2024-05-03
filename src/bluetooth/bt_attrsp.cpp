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
#define UART_DEBUG_ENABLE

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
extern CurParseInf_t g_CurIdx;

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
		DEBUG_PRINTF("BT_ATT_OPCODE_ATT_FIND_INFORMATION_RSP (0x05) \r\n");
		BtAttFindInfoRsp_t *p = (BtAttFindInfoRsp_t*) &pRspAtt->FindInfoRsp;
		DEBUG_PRINTF("Format %d\r\n", p->Fmt);

		if (RspLen <= 3)
		{
			DEBUG_PRINTF("Hdl = %d\r\n", p->HdlUuid16->Hdl);
			DEBUG_PRINTF("Uuid16 = (0x)%02x \r\n", p->HdlUuid16->Uuid);
		}
		else
		{
			DEBUG_PRINTF("Hdl = %d\r\n", p->HdlUuid128->Hdl);
			DEBUG_PRINTF("Uuid128 = (0x) ");
			for (int i = 0; i < RspLen - 2; i++)
			{
				DEBUG_PRINTF("%02x ", p->HdlUuid128->Uuid[i]);
			}
			DEBUG_PRINTF("\r\n");
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
		DEBUG_PRINTF("Len %d, Raw Data (hex): ", p->Len);
		for (int i = 0; i < p->Len; i++)
			DEBUG_PRINTF("%X ", p->Data[i]);
		DEBUG_PRINTF("\r\n");

		uint16_t Hdl = p->Data[0] | (p->Data[1] << 8);
		uint8_t CharProp = p->Data[2];
		uint16_t CharHdlVal = p->Data[3] | (p->Data[4] << 8);

		switch (g_UuidType.Uuid16)
		{
		case BT_UUID_DECLARATIONS_CHARACTERISTIC:
		{
			DEBUG_PRINTF("Response BT_UUID_DECLARATIONS_CHARACTERISTIC 0x2803\r\n");
			BtGattDBChar_t *pChar =
					(BtGattDBChar_t*) &g_BtDevSdc.Services[g_CurIdx.SrvIdx].charateristics[g_CurIdx.CharIdx];
			memcpy((uint8_t*) &pChar->characteristic.char_props, &CharProp,
					sizeof(BtGattCharProps_t));
			pChar->characteristic.handle_value = CharHdlVal;

			if (p->Len <= 7) //16-bit UUID
			{
				pChar->characteristic.uuid.Uuid = p->Data[5]
						| (p->Data[6] << 8);
				pChar->characteristic.uuid.BaseIdx = 0;
				pChar->characteristic.uuid.Type = BT_UUID_TYPE_16;
			}
			else if (p->Len > 7 && p->Len <= 9) // 32-bit UUID?
			{
				uint32_t CharUuid32 = p->Data[5] | (p->Data[6] << 8)
						| (p->Data[7] << 16) | (p->Data[8] << 24);
			}
			else // 128-bit UUID
			{
				uint8_t *pUuid128 = (uint8_t*) &p->Data[5];
				uint16_t CharUuid16 = pUuid128[12] | (pUuid128[13] << 8);
				DEBUG_PRINTF("128-bit UUID with UUID16 0x%X \r\n", CharUuid16);
				pUuid128[12] = 0;
				pUuid128[13] = 0;
				int idx = BtUuidFindBase(pUuid128);
				DEBUG_PRINTF("BaseUUID128 (index %d) (hex): ", idx);
				for (int i = 0; i < 16; i++)
					DEBUG_PRINTF("%X ", pUuid128[i]);
				DEBUG_PRINTF("\r\n");

				pChar->characteristic.uuid.BaseIdx = idx;
				pChar->characteristic.uuid.Uuid = CharUuid16;
				pChar->characteristic.uuid.Type = BT_UUID_TYPE_128;
			}

			DEBUG_PRINTF("SrvcIdx %d, CharIdx %d: ", g_CurIdx.SrvIdx,
					g_CurIdx.CharIdx);
			DEBUG_PRINTF(
					"Hdl %d, CharProp 0x%X, CharHdlVal %d, CharUuid16 0x%X\r\n",
					Hdl, pChar->characteristic.char_props,
					pChar->characteristic.handle_value,
					pChar->characteristic.uuid.Uuid);

			g_CurIdx.CharIdx++;
			g_BtDevSdc.Services[g_CurIdx.SrvIdx].char_count++;

			DEBUG_PRINTF("New char_count = %d\r\n",
					g_BtDevSdc.Services[g_CurIdx.SrvIdx].char_count);

			// Search for the next BLE characteristic
			uint8_t eHdl =
					g_BtDevSdc.Services[g_CurIdx.SrvIdx].handle_range.EndHdl;
			if (Hdl + 1 <= eHdl)
			{
				g_CurIdx.Hdl = Hdl + 1;
				DEBUG_PRINTF(
						"Search for the next characteristic with sHdl = %d\r\n",
						g_CurIdx.Hdl);
				g_UuidType.BaseIdx = 0;
				g_UuidType.Type = BT_UUID_TYPE_16;
				//g_UuidType.Uuid16 = BT_UUID_DECLARATIONS_CHARACTERISTIC;
				BtAttReadByTypeRequest((BtHciDevice_t*) g_BtDevSdc.pHciDev,
						g_BtDevSdc.ConnHdl, g_CurIdx.Hdl, eHdl, &g_UuidType);
			}
		}
			break;
		case BT_UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION:
		{
			DEBUG_PRINTF("Respone BT_UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION 0x2902\r\n");
			DEBUG_PRINTF("SrvcIdx %d CharIdx %d CurHdl %d\r\n", g_CurIdx.SrvIdx, g_CurIdx.CharIdx, g_CurIdx.Hdl);
			BtGattDBChar_t *pChar =
					(BtGattDBChar_t*) &g_BtDevSdc.Services[g_CurIdx.SrvIdx].charateristics[g_CurIdx.CharIdx];

//			if (pChar->characteristic.char_props.notify)
//				DEBUG_PRINTF("Notification Characteristic \r\n");
//			else if (pChar->characteristic.char_props.indicate)
//				DEBUG_PRINTF("Indication Characteristic\r\n");
//			else
//				DEBUG_PRINTF("Neither notification nor indication char\r\n");

			if (pChar->characteristic.char_props.notify
					|| pChar->characteristic.char_props.indicate)
			{
				pChar->cccd_handle = Hdl;
				DEBUG_PRINTF(
						"SrvcIdx %d CharIdx %d with Hdl %d has CCC_handle %d\r\n",
						g_CurIdx.SrvIdx, g_CurIdx.CharIdx,
						pChar->characteristic.handle_value, pChar->cccd_handle);
			}

			bool bNextSrvc = false;
			g_CurIdx.CharIdx++;
			if (g_CurIdx.CharIdx < g_BtDevSdc.Services[g_CurIdx.SrvIdx].char_count)
			{
				DEBUG_PRINTF("Scan Next CharIdx %d\r\n", g_CurIdx.CharIdx);
				pChar = (BtGattDBChar_t*) &g_BtDevSdc.Services[g_CurIdx.SrvIdx].charateristics[g_CurIdx.CharIdx];
				uint8_t eHdl = g_BtDevSdc.Services[g_CurIdx.SrvIdx].handle_range.EndHdl;
				if (Hdl + 1 <= eHdl)
				{
					g_CurIdx.Hdl = Hdl + 1;
					BtAttReadByTypeRequest((BtHciDevice_t*) g_BtDevSdc.pHciDev,
							g_BtDevSdc.ConnHdl, g_CurIdx.Hdl, eHdl, &g_UuidType);
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
					uint8_t eHdl =
							g_BtDevSdc.Services[g_CurIdx.SrvIdx].handle_range.EndHdl;
					BtAttReadByTypeRequest((BtHciDevice_t*) g_BtDevSdc.pHciDev,
							g_BtDevSdc.ConnHdl, g_CurIdx.Hdl, eHdl, &g_UuidType);
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
		DEBUG_PRINTF("BT_ATT_OPCODE_ATT_READ_BY_GROUP_TYPE_RSP (0x11) \r\n");
		BtAttReadByGroupTypeRsp_t *p =
				(BtAttReadByGroupTypeRsp_t*) &pRspAtt->ReadByGroupTypeRsp;
		DEBUG_PRINTF("Len = %d, Raw data (hex): ", p->Len);
		for (int i = 0; i < p->Len; i++)
			DEBUG_PRINTF("%X ", p->Data[i]);
		DEBUG_PRINTF("\r\n");

		bool bScanNext = false;
		uint16_t NextStartHdl;

		g_BtDevSdc.NbSrvc++;
		uint8_t SrvcIdx = g_BtDevSdc.NbSrvc - 1;

		// Process the received data
		if (p->Len <= 6) // Process UUID 16
		{
			BtAttReadByGroupTypeRspUuid16_t *g =
					(BtAttReadByGroupTypeRspUuid16_t*) p->Data;
			DEBUG_PRINTF("StartHdl = %d, EndHdl = %d, Service UUID16 0x%X \r\n", g->HdlStart, g->HdlEnd, g->Uuid);

			BtGattDBSrvc_t *pSrvc =
					(BtGattDBSrvc_t*) &g_BtDevSdc.Services[SrvcIdx];
			pSrvc->handle_range.StartHdl = g->HdlStart;
			pSrvc->handle_range.EndHdl = g->HdlEnd;
			pSrvc->srv_uuid.Uuid = g->Uuid;
			pSrvc->srv_uuid.BaseIdx = 0; // Standard Bluetooth service
			pSrvc->srv_uuid.Type = BT_UUID_TYPE_16;

			bScanNext = (g->HdlEnd != 0xFFFF) ? true : false;
			NextStartHdl = (g->HdlEnd != 0xFFFF) ? (g->HdlEnd + 1) : 0xFFFF;
		}
		else // Process UUID 128
		{
			BtAttReadByGroupTypeRspUuid128_t *g =
					(BtAttReadByGroupTypeRspUuid128_t*) p->Data;
			DEBUG_PRINTF("StartHdl = %d, EndHdl = %d \r\n", g->HdlStart, g->HdlEnd);
			DEBUG_PRINTF("Custom service UUID128 (hex): ");
			for (int i = 0; i < 16; i++)
				DEBUG_PRINTF("%X ", g->Uuid[i]);
			DEBUG_PRINTF("\r\n");

			BtGattDBSrvc_t *pSrvc =
					(BtGattDBSrvc_t*) &g_BtDevSdc.Services[SrvcIdx];
			pSrvc->handle_range.StartHdl = g->HdlStart;
			pSrvc->handle_range.EndHdl = g->HdlEnd;
			int idx = BtUuid128To16(&pSrvc->srv_uuid, g->Uuid);
			g->Uuid[12] = 0;
			g->Uuid[13] = 0;
			DEBUG_PRINTF("BLE service UUID16 0x%X \r\n", pSrvc->srv_uuid.Uuid);
			DEBUG_PRINTF("BaseUUID128 (hex) = ");
			for (int i = 0; i < 16; i++)
				DEBUG_PRINTF("%X ", g->Uuid[i]);
			DEBUG_PRINTF("was added at the internal table with index %d\r\n", idx);

			bScanNext = (g->HdlEnd != 0xFFFF) ? true : false;
			NextStartHdl = (g->HdlEnd != 0xFFFF) ? (g->HdlEnd + 1) : 0xFFFF;
		}

		// Read the next group type
		if (bScanNext)
		{
			// Continue to read the next group
			DEBUG_PRINTF("Read the next group type with StartHdl = %d\r\n", NextStartHdl);
			BtUuid_t Uuid = {
					.BaseIdx = 0, // Standard bluetooth
					.Type = BT_UUID_TYPE_16,
					.Uuid16 = BT_UUID_DECLARATIONS_PRIMARY_SERVICE,
			};
			BtAttReadByGroupTypeRequest((BtHciDevice_t*) g_BtDevSdc.pHciDev,
					g_BtDevSdc.ConnHdl, NextStartHdl, 0xFFFF, &Uuid);
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
