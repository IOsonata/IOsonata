/**-------------------------------------------------------------------------
@file	bt_gatt_sdc.cpp

@brief	Bluetooth GATT  

Generic implementation Bluetooth Generic Attribute Profile with Nordic SDC

@author	Hoang Nguyen Hoan
@date	Oct. 22, 2022

@license

MIT License

Copyright (c) 2022, I-SYST, all rights reserved

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
#include <memory.h>

#include "sdc_hci.h"

#include "istddef.h"
#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_att.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_l2cap.h"

#include "coredev/uart.h"

extern UART g_Uart;

bool BtGattCharNotify(uint16_t ConnHdl, BtGattChar_t *pChar, void * const pData, size_t Len)
{
	if (BtGattCharSetValue(pChar, pData, Len) == false)
	{
		return false;
	}

	if (isBtGattCharNotifyEnabled(pChar))
	{
		//BtCtlrDev_t *p = (BtCtlrDev_t *)pDev->pDevData;
		uint8_t buf[BT_HCI_BUFFER_MAX_SIZE];
		BtHciACLDataPacket_t *acl = (BtHciACLDataPacket_t*)buf;
		BtL2CapPdu_t *l2pdu = (BtL2CapPdu_t*)acl->Data;

		g_Uart.printf("BtHciMotify : %d %d \r\n", ConnHdl, pChar->ValHdl);

		acl->Hdr.ConnHdl = ConnHdl;
		acl->Hdr.PBFlag = BT_HCI_PBFLAG_COMPLETE_L2CAP_PDU;
		acl->Hdr.BCFlag = 0;

		l2pdu->Hdr.Cid = BT_L2CAP_CID_ATT;

		l2pdu->Att.OpCode = BT_ATT_OPCODE_ATT_HANDLE_VALUE_NTF;

//		uint8_t *param = l2pdu->Att.Param;
		//BtGattCharNotify_t *p = (BtGattCharNotify_t*)l2pdu->Att.Param;

//		*(uint16_t*)param = pChar->ValHdl;
//		param += 2;
		l2pdu->Att.HandleValueNtf.ValHdl = pChar->ValHdl;
		memcpy(l2pdu->Att.HandleValueNtf.Data, pData, Len);
		l2pdu->Hdr.Len = sizeof(BtGattCharNotify_t) + Len;
		acl->Hdr.Len = l2pdu->Hdr.Len + sizeof(BtL2CapHdr_t);

		return sdc_hci_data_put((uint8_t*)acl) == 0 ? acl->Hdr.Len + sizeof(acl->Hdr) : 0;
	}

	return false;
}

