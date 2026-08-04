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

/******** For DEBUG Trace ************/
//#define DEBUG_ENABLE

#if !defined(NDEBUG) && defined(DEBUG_ENABLE)
#include "syslog.h"
#define DEBUG_PRINTF(...)		SysLogPrintf(SysLogGet(), __VA_ARGS__)
#else
#define DEBUG_PRINTF(...)
#endif
/*******************************/

static inline uint16_t BtAttRspLe16(const uint8_t *p)
{
	return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

static bool BtAttReadMultipleVarRspValid(const uint8_t *pData, int Len)
{
	int pos = 0;

	while (pos < Len)
	{
		if (Len - pos < 2)
		{
			return false;
		}

		uint16_t valueLen = BtAttRspLe16(pData + pos);
		pos += 2;
		if (valueLen > (uint16_t)(Len - pos))
		{
			return false;
		}
		pos += valueLen;
	}

	return pos == Len;
}

// Validate the complete response before releasing the one ATT transaction slot
// or reading any response fields. A matched opcode alone is not enough: tuple
// lengths and payload strides are supplied by the peer and must describe only
// bytes present in this PDU.
static bool BtAttRspValid(BtDevice_t *pPeer, const BtAttReqRsp_t *pRspAtt,
							 int RspLen)
{
	if (pPeer == nullptr || pRspAtt == nullptr || RspLen < 1)
	{
		return false;
	}

	const uint8_t *raw = (const uint8_t *)pRspAtt;

	switch (pRspAtt->OpCode)
	{
		case BT_ATT_OPCODE_ATT_ERROR_RSP:
			return RspLen == 5 &&
				   pRspAtt->ErrorRsp.ReqOpCode == pPeer->AttReqOpcode;

		case BT_ATT_OPCODE_ATT_EXCHANGE_MTU_RSP:
			return RspLen == 3 && pRspAtt->ExchgMtuReqRsp.RxMtu >= BT_ATT_MTU_MIN;

		case BT_ATT_OPCODE_ATT_FIND_INFORMATION_RSP:
		{
			if (RspLen < 2)
			{
				return false;
			}
			uint8_t fmt = raw[1];
			int entryLen = fmt == 1 ? 4 : fmt == 2 ? 18 : 0;
			int payloadLen = RspLen - 2;
			return entryLen != 0 && payloadLen >= entryLen &&
				   (payloadLen % entryLen) == 0;
		}

		case BT_ATT_OPCODE_ATT_FIND_BY_TYPE_VALUE_RSP:
		{
			int payloadLen = RspLen - 1;
			return payloadLen >= 4 && (payloadLen % 4) == 0;
		}

		case BT_ATT_OPCODE_ATT_READ_BY_TYPE_RSP:
		{
			if (RspLen < 2)
			{
				return false;
			}

			uint8_t elemLen = raw[1];
			int payloadLen = RspLen - 2;
			if (elemLen < 2 || payloadLen < elemLen ||
				(payloadLen % elemLen) != 0)
			{
				return false;
			}

			switch (pPeer->Discovery.UuidType.Uuid16)
			{
				case BT_UUID_DECLARATIONS_CHARACTERISTIC:
					return elemLen == 7 || elemLen == 21;

				case BT_UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION:
					return elemLen == 4;

				case BT_UUID_DESCRIPTOR_CHARACTERISTIC_USER_DESCRIPTION:
					return elemLen >= 3;

				default:
					return true;
			}
		}

		case BT_ATT_OPCODE_ATT_READ_RSP:
		case BT_ATT_OPCODE_ATT_READ_BLOB_RSP:
		case BT_ATT_OPCODE_ATT_READ_MULTIPLE_RSP:
			return RspLen >= 1;

		case BT_ATT_OPCODE_ATT_READ_BY_GROUP_TYPE_RSP:
		{
			if (RspLen < 2)
			{
				return false;
			}
			uint8_t elemLen = raw[1];
			int payloadLen = RspLen - 2;
			return (elemLen == 6 || elemLen == 20) &&
				   payloadLen >= elemLen && (payloadLen % elemLen) == 0;
		}

		case BT_ATT_OPCODE_ATT_WRITE_RSP:
		case BT_ATT_OPCODE_ATT_EXECUTE_WRITE_RSP:
			return RspLen == 1;

		case BT_ATT_OPCODE_ATT_PREPARE_WRITE_RSP:
			return RspLen >= 5;

		case BT_ATT_OPCODE_ATT_READ_MULTIPLE_VARIABLE_RSP:
			return RspLen >= 3 &&
				   BtAttReadMultipleVarRspValid(raw + 1, RspLen - 1);

		default:
			return false;
	}
}

static bool BtAttDiscoveryServiceValid(const BtDevice_t *pPeer)
{
	return pPeer != nullptr && pPeer->NbSrvc > 0 &&
		   pPeer->Discovery.SrvIdx < pPeer->NbSrvc &&
		   pPeer->Discovery.SrvIdx < BT_DEV_SERVICE_MAXCNT;
}

static bool BtAttDiscoveryReadByType(BtDevice_t *pPeer)
{
	if (!BtAttDiscoveryServiceValid(pPeer) || pPeer->pHciDev == nullptr)
	{
		return false;
	}

	BtGattDBSrvc_t *pSrvc = &pPeer->Services[pPeer->Discovery.SrvIdx];
	pPeer->Discovery.Hdl = pSrvc->handle_range.StartHdl;
	return BtAttReadByTypeRequest((BtHciDevice_t *)pPeer->pHciDev,
								pPeer->Conn.Hdl,
								pSrvc->handle_range.StartHdl,
								pSrvc->handle_range.EndHdl,
								&pPeer->Discovery.UuidType);
}

static void BtAttDiscoveryNextType(BtDevice_t *pPeer)
{
	if (pPeer == nullptr)
	{
		return;
	}

	switch (pPeer->Discovery.UuidType.Uuid16)
	{
		case BT_UUID_DECLARATIONS_CHARACTERISTIC:
			pPeer->Discovery.UuidType.Uuid16 =
					BT_UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION;
			break;

		case BT_UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION:
			pPeer->Discovery.UuidType.Uuid16 =
					BT_UUID_DESCRIPTOR_CHARACTERISTIC_USER_DESCRIPTION;
			break;

		case BT_UUID_DESCRIPTOR_CHARACTERISTIC_USER_DESCRIPTION:
			BtDeviceDiscovered(pPeer);
			return;

		default:
			return;
	}

	pPeer->Discovery.SrvIdx = 0;
	pPeer->Discovery.CharIdx = 0;
	(void)BtAttDiscoveryReadByType(pPeer);
}

uint32_t BtAttProcessError(uint16_t ConnHdl, BtAttReqRsp_t * const pRspAtt,
						   int RspLen)
{
	(void)RspLen;
	BtDevice_t *pPeer = BtPeerFindByHdl(ConnHdl);
	if (pPeer == nullptr || pRspAtt == nullptr)
	{
		return 0;
	}

	if (pRspAtt->ErrorRsp.Error != BT_ATT_ERROR_ATT_NOT_FOUND)
	{
		return 0;
	}

	switch (pRspAtt->ErrorRsp.ReqOpCode)
	{
		case BT_ATT_OPCODE_ATT_READ_BY_GROUP_TYPE_REQ:
			if (pPeer->NbSrvc == 0)
			{
				BtDeviceDiscovered(pPeer);
				return 0;
			}

			pPeer->Discovery.SrvIdx = 0;
			pPeer->Discovery.CharIdx = 0;
			pPeer->Discovery.UuidType.BaseIdx = 0;
			pPeer->Discovery.UuidType.Type = BT_UUID_TYPE_16;
			pPeer->Discovery.UuidType.Uuid16 =
					BT_UUID_DECLARATIONS_CHARACTERISTIC;
			(void)BtAttDiscoveryReadByType(pPeer);
			break;

		case BT_ATT_OPCODE_ATT_READ_BY_TYPE_REQ:
			if (!BtAttDiscoveryServiceValid(pPeer))
			{
				BtDeviceDiscovered(pPeer);
				return 0;
			}

			if (pPeer->Discovery.SrvIdx + 1 < pPeer->NbSrvc)
			{
				pPeer->Discovery.SrvIdx++;
				pPeer->Discovery.CharIdx = 0;
				(void)BtAttDiscoveryReadByType(pPeer);
			}
			else
			{
				BtAttDiscoveryNextType(pPeer);
			}
			break;

		default:
			break;
	}

	return 0;
}

static void BtAttProcessReadByTypeRsp(BtDevice_t *pPeer,
									 BtAttReqRsp_t *pRspAtt, int RspLen)
{
	BtAttReadByTypeRsp_t *p = &pRspAtt->ReadByTypeRsp;
	int payloadLen = RspLen - 2;
	uint8_t elemLen = p->Len;

	if (!BtAttDiscoveryServiceValid(pPeer))
	{
		return;
	}

	BtGattDBSrvc_t *pSrvc = &pPeer->Services[pPeer->Discovery.SrvIdx];
	uint16_t lastHdl = 0;

	switch (pPeer->Discovery.UuidType.Uuid16)
	{
		case BT_UUID_DECLARATIONS_CHARACTERISTIC:
		{
			for (int off = 0; off < payloadLen; off += elemLen)
			{
				if (pSrvc->char_count >= BT_GATT_DB_MAX_CHARS)
				{
					break;
				}

				const uint8_t *d = p->Data + off;
				uint16_t declHdl = BtAttRspLe16(d);
				uint16_t valueHdl = BtAttRspLe16(d + 3);
				if (declHdl == 0 || valueHdl == 0 || valueHdl <= declHdl)
				{
					return;
				}

				BtGattDBChar_t *pChar = &pSrvc->characteristics[pSrvc->char_count];
				memset(pChar, 0, sizeof(*pChar));
				pChar->characteristic.handle_decl = declHdl;
				pChar->characteristic.handle_value = valueHdl;
				memcpy(&pChar->characteristic.char_props, d + 2,
					   sizeof(BtGattCharProps_t));
				pChar->cccd_handle = BT_ATT_HANDLE_INVALID;
				pChar->ext_prop_handle = BT_ATT_HANDLE_INVALID;
				pChar->user_desc_handle = BT_ATT_HANDLE_INVALID;
				pChar->report_ref_handle = BT_ATT_HANDLE_INVALID;

				if (elemLen == 7)
				{
					pChar->characteristic.uuid.BaseIdx = 0;
					pChar->characteristic.uuid.Type = BT_UUID_TYPE_16;
					pChar->characteristic.uuid.Uuid = BtAttRspLe16(d + 5);
				}
				else
				{
					uint8_t base[16];
					memcpy(base, d + 5, sizeof(base));
					uint16_t uuid = BtAttRspLe16(base + 12);
					base[12] = 0;
					base[13] = 0;
					int idx = BtUuidFindBase(base);
					pChar->characteristic.uuid.BaseIdx =
							idx >= 0 ? (uint8_t)idx : 0;
					pChar->characteristic.uuid.Type = BT_UUID_TYPE_16;
					pChar->characteristic.uuid.Uuid = uuid;
				}

				pSrvc->char_count++;
				pPeer->Discovery.CharIdx = pSrvc->char_count;
				lastHdl = declHdl;
			}
		}
		break;

		case BT_UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION:
		{
			uint8_t charIdx = pPeer->Discovery.CharIdx;
			for (int off = 0; off < payloadLen; off += elemLen)
			{
				uint16_t hdl = BtAttRspLe16(p->Data + off);
				while (charIdx < pSrvc->char_count)
				{
					BtGattDBChar_t *pChar = &pSrvc->characteristics[charIdx];
					if (pChar->characteristic.char_props.notify ||
						pChar->characteristic.char_props.indicate)
					{
						pChar->cccd_handle = hdl;
						charIdx++;
						break;
					}
					charIdx++;
				}
				lastHdl = hdl;
			}
			pPeer->Discovery.CharIdx = charIdx;
		}
		break;

		case BT_UUID_DESCRIPTOR_CHARACTERISTIC_USER_DESCRIPTION:
			lastHdl = BtAttRspLe16(p->Data + payloadLen - elemLen);
			break;

		default:
			return;
	}

	if (lastHdl != 0 && lastHdl < pSrvc->handle_range.EndHdl)
	{
		pPeer->Discovery.Hdl = (uint16_t)(lastHdl + 1U);
		(void)BtAttReadByTypeRequest((BtHciDevice_t *)pPeer->pHciDev,
									pPeer->Conn.Hdl, pPeer->Discovery.Hdl,
									pSrvc->handle_range.EndHdl,
									&pPeer->Discovery.UuidType);
		return;
	}

	if (pPeer->Discovery.SrvIdx + 1 < pPeer->NbSrvc)
	{
		pPeer->Discovery.SrvIdx++;
		pPeer->Discovery.CharIdx = 0;
		(void)BtAttDiscoveryReadByType(pPeer);
	}
	else
	{
		BtAttDiscoveryNextType(pPeer);
	}
}

static void BtAttProcessReadByGroupRsp(BtDevice_t *pPeer,
									  BtAttReqRsp_t *pRspAtt, int RspLen)
{
	BtAttReadByGroupTypeRsp_t *p = &pRspAtt->ReadByGroupTypeRsp;
	uint8_t elemLen = p->Len;
	int payloadLen = RspLen - 2;
	uint16_t lastEndHdl = 0;

	for (int off = 0; off < payloadLen; off += elemLen)
	{
		if (pPeer->NbSrvc >= BT_DEV_SERVICE_MAXCNT)
		{
			break;
		}

		const uint8_t *d = p->Data + off;
		uint16_t startHdl = BtAttRspLe16(d);
		uint16_t endHdl = BtAttRspLe16(d + 2);
		if (startHdl == 0 || endHdl < startHdl ||
			(lastEndHdl != 0 && startHdl <= lastEndHdl))
		{
			return;
		}

		BtGattDBSrvc_t *pSrvc = &pPeer->Services[pPeer->NbSrvc];
		memset(pSrvc, 0, sizeof(*pSrvc));
		pSrvc->handle_range.StartHdl = startHdl;
		pSrvc->handle_range.EndHdl = endHdl;

		if (elemLen == 6)
		{
			pSrvc->srv_uuid.BaseIdx = 0;
			pSrvc->srv_uuid.Type = BT_UUID_TYPE_16;
			pSrvc->srv_uuid.Uuid = BtAttRspLe16(d + 4);
		}
		else
		{
			uint8_t uuid128[16];
			memcpy(uuid128, d + 4, sizeof(uuid128));
			(void)BtUuid128To16(&pSrvc->srv_uuid, uuid128);
		}

		pPeer->NbSrvc++;
		lastEndHdl = endHdl;
	}

	if (lastEndHdl != 0 && lastEndHdl != 0xFFFF)
	{
		BtUuid_t uuid = {
			.BaseIdx = 0,
			.Type = BT_UUID_TYPE_16,
			.Uuid16 = BT_UUID_DECLARATIONS_PRIMARY_SERVICE,
		};
		(void)BtAttReadByGroupTypeRequest((BtHciDevice_t *)pPeer->pHciDev,
										pPeer->Conn.Hdl,
										(uint16_t)(lastEndHdl + 1U),
										0xFFFF, &uuid);
	}
	else if (pPeer->NbSrvc > 0)
	{
		pPeer->Discovery.SrvIdx = 0;
		pPeer->Discovery.CharIdx = 0;
		pPeer->Discovery.UuidType.BaseIdx = 0;
		pPeer->Discovery.UuidType.Type = BT_UUID_TYPE_16;
		pPeer->Discovery.UuidType.Uuid16 =
				BT_UUID_DECLARATIONS_CHARACTERISTIC;
		(void)BtAttDiscoveryReadByType(pPeer);
	}
}

void BtAttProcessRsp(uint16_t ConnHdl, BtAttReqRsp_t * const pRspAtt, int RspLen)
{
	BtDevice_t *pPeer = BtPeerFindByHdl(ConnHdl);
	if (pPeer == nullptr || pRspAtt == nullptr || RspLen <= 0 ||
		pPeer->bAttReqPending == false || pPeer->bAttTimedOut)
	{
		return;
	}

	bool matched = false;
	if (pRspAtt->OpCode == BT_ATT_OPCODE_ATT_ERROR_RSP)
	{
		if (RspLen < 2)
		{
			return;
		}
		matched = pRspAtt->ErrorRsp.ReqOpCode == pPeer->AttReqOpcode;
	}
	else
	{
		matched = pRspAtt->OpCode == pPeer->AttRspOpcode;
	}

	if (!matched || !BtAttRspValid(pPeer, pRspAtt, RspLen))
	{
		return;
	}

	// A structurally valid matching response completes the request. Release
	// before dispatch so discovery can issue the next request immediately.
	pPeer->bAttReqPending = false;
	pPeer->AttReqOpcode = 0;
	pPeer->AttRspOpcode = 0;
	pPeer->AttReqTime = 0;

	switch (pRspAtt->OpCode)
	{
		case BT_ATT_OPCODE_ATT_ERROR_RSP:
			(void)BtAttProcessError(ConnHdl, pRspAtt, RspLen);
			break;

		case BT_ATT_OPCODE_ATT_EXCHANGE_MTU_RSP:
			pPeer->Conn.MaxMtu = min(BtAttGetMtu(),
									 pRspAtt->ExchgMtuReqRsp.RxMtu);
			break;

		case BT_ATT_OPCODE_ATT_READ_BY_TYPE_RSP:
			BtAttProcessReadByTypeRsp(pPeer, pRspAtt, RspLen);
			break;

		case BT_ATT_OPCODE_ATT_READ_BY_GROUP_TYPE_RSP:
			BtAttProcessReadByGroupRsp(pPeer, pRspAtt, RspLen);
			break;

		case BT_ATT_OPCODE_ATT_READ_RSP:
			if (BtAttDiscoveryServiceValid(pPeer))
			{
				BtGattDBSrvc_t *pSrvc =
						&pPeer->Services[pPeer->Discovery.SrvIdx];
				pPeer->Discovery.Hdl++;
				if (pPeer->Discovery.Hdl > pSrvc->handle_range.EndHdl)
				{
					if (pPeer->Discovery.SrvIdx + 1 < pPeer->NbSrvc)
					{
						pPeer->Discovery.SrvIdx++;
						pSrvc = &pPeer->Services[pPeer->Discovery.SrvIdx];
						pPeer->Discovery.Hdl =
								(uint16_t)(pSrvc->handle_range.StartHdl + 1U);
						(void)BtAttReadRequest((BtHciDevice_t *)pPeer->pHciDev,
											pPeer->Conn.Hdl,
											pPeer->Discovery.Hdl);
					}
				}
				else
				{
					(void)BtAttReadRequest((BtHciDevice_t *)pPeer->pHciDev,
										pPeer->Conn.Hdl,
										pPeer->Discovery.Hdl);
				}
			}
			break;

		case BT_ATT_OPCODE_ATT_FIND_INFORMATION_RSP:
		case BT_ATT_OPCODE_ATT_FIND_BY_TYPE_VALUE_RSP:
		case BT_ATT_OPCODE_ATT_READ_BLOB_RSP:
		case BT_ATT_OPCODE_ATT_READ_MULTIPLE_RSP:
		case BT_ATT_OPCODE_ATT_WRITE_RSP:
		case BT_ATT_OPCODE_ATT_PREPARE_WRITE_RSP:
		case BT_ATT_OPCODE_ATT_EXECUTE_WRITE_RSP:
		case BT_ATT_OPCODE_ATT_READ_MULTIPLE_VARIABLE_RSP:
		default:
			break;
	}
}
