/**-------------------------------------------------------------------------
@file	ble_app_sdc.cpp

@brief	Nordic SDK based BLE application creation helper using softdevice controller


@author	Hoang Nguyen Hoan
@date	Mar. 8, 2022

@license

MIT License

Copyright (c) 2022, I-SYST inc., all rights reserved

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
#include <stdio.h>
#include <inttypes.h>
#include <atomic>
#include <stdlib.h>

#include "mpsl.h"
#include "mpsl_coex.h"
#include "sdc.h"
#include "sdc_soc.h"
#include "sdc_hci_cmd_le.h"
#include "sdc_hci.h"
#include "sdc_hci_vs.h"
#include "sdc_hci_cmd_controller_baseband.h"

#include "istddef.h"
#include "convutil.h"
#include "coredev/uart.h"
#include "coredev/system_core_clock.h"
#include "coredev/timer.h"
#include "iopinctrl.h"
#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_hcievt.h"
#include "bluetooth/bt_l2cap.h"
#include "bluetooth/bt_att.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_gap.h"
#include "bluetooth/ble_appearance.h"
#include "bluetooth/bt_dev.h"
#include "bluetooth/bt_app.h"
#include "app_evt_handler.h"

//BleConn_t g_BleConn = {0,};
extern UART g_Uart;

//void TimerHandler(TimerDev_t * const pTimer, uint32_t Evt);

static inline uint32_t BtDevSendData(void *pData, uint32_t Len) {
	return sdc_hci_data_put((uint8_t*)pData) == 0 ? Len : 0;
}
void BtDevEvtHandler(uint32_t Evt, void * const pCtx);
void BtDevConnected(uint16_t ConnHdl, uint8_t Role, uint8_t AddrType, uint8_t PerrAddr[6]);
void BtDevDisconnected(uint16_t ConnHdl, uint8_t Reason);

#pragma pack(push, 4)

/*
typedef struct _BleAppData {
	BLEAPP_STATE State;
	BLEAPP_ROLE Role;
	uint8_t AdvHdl;		// Advertisement handle
	uint16_t ConnHdl;	// BLE connection handle
	int ConnLedPort;
	int ConnLedPin;
	uint8_t ConnLedActLevel;
	uint16_t VendorId;
	int PeriphDevCnt;
	uint32_t (*SDEvtHandler)(void) ;
	int MaxMtu;
	bool bSecure;
	bool bAdvertising;
	bool bExtAdv;
	bool bScan;
//    bool BleInitialized;
	BLEAPP_COEXMODE CoexMode;
} BleAppData_t;
*/
#pragma pack(pop)

// S132 tx_power values: -40dBm, -20dBm, -16dBm, -12dBm, -8dBm, -4dBm, 0dBm, +3dBm and +4dBm
// S140 tx_power values: -40dBm, -20dBm, -16dBm, -12dBm, -8dBm, -4dBm, 0dBm, +2dBm, +3dBm, +4dBm, +5dBm, +6dBm, +7dBm and +8dBm.

static const int8_t s_TxPowerdBm[] = {
	-40, -20, -16, -12, -8, -4, 0,
#if defined(S132) || defined(NRF52832_XXAA)
	3, 4, 4
#else
	2, 3, 4, 5, 6, 7, 8
#endif
};

alignas(4) static const int s_NbTxPowerdBm = sizeof(s_TxPowerdBm) / sizeof(int8_t);

static BtDev_t s_BtDevSdc = {
	BTDEV_ROLE_PERIPHERAL,
};

/**@brief Bluetooth SIG debug mode Private Key */
__ALIGN(4) __WEAK extern const uint8_t g_lesc_private_key[32] = {
    0xbd,0x1a,0x3c,0xcd,0xa6,0xb8,0x99,0x58,0x99,0xb7,0x40,0xeb,0x7b,0x60,0xff,0x4a,
    0x50,0x3f,0x10,0xd2,0xe3,0xb3,0xc9,0x74,0x38,0x5f,0xc5,0xa3,0xd4,0xf6,0x49,0x3f,
};

//__ALIGN(4) static ble_gap_lesc_p256_pk_t    s_lesc_public_key;      /**< LESC ECC Public Key */
//__ALIGN(4) static ble_gap_lesc_dhkey_t      s_lesc_dh_key;          /**< LESC ECC DH Key*/
#if 1
alignas(4) static uint8_t s_BtDevAdvBuff[260];
alignas(4) static sdc_hci_cmd_le_set_adv_data_t &s_BtDevAdvData = *(sdc_hci_cmd_le_set_adv_data_t*)s_BtDevAdvBuff;
alignas(4) static BleAdvPacket_t s_BtDevAdvPkt = { sizeof(s_BtDevAdvData.adv_data), 0, s_BtDevAdvData.adv_data};

alignas(4) static sdc_hci_cmd_le_set_ext_adv_data_t &s_BtDevExtAdvData = *(sdc_hci_cmd_le_set_ext_adv_data_t*)s_BtDevAdvBuff;
alignas(4) static BleAdvPacket_t s_BtDevExtAdvPkt = { 255, 0, s_BtDevExtAdvData.adv_data};

alignas(4) static uint8_t s_BtDevSrBuff[260];

alignas(4) static sdc_hci_cmd_le_set_scan_response_data_t &s_BtDevSrData = *(sdc_hci_cmd_le_set_scan_response_data_t*)s_BtDevSrBuff;
alignas(4) static BleAdvPacket_t s_BtDevSrPkt = { sizeof(s_BtDevSrData.scan_response_data), 0, s_BtDevSrData.scan_response_data};

alignas(4) static sdc_hci_cmd_le_set_ext_scan_response_data_t &s_BtDevExtSrData = *(sdc_hci_cmd_le_set_ext_scan_response_data_t*)s_BtDevSrBuff;
alignas(4) static BleAdvPacket_t s_BtDevExtSrPkt = { 255, 0, s_BtDevExtSrData.scan_response_data};
#endif

alignas(4) static uint8_t s_BtStackSdcMemPool[10000];
/*
alignas(4) static BtHciDevice_t s_HciDevice = {
		251, 251,
		.SendData = HciSdcSendData,
		.EvtHandler = BleAppEvtHandler,
		.Connected = BleAppConnected,
		.Disconnected = BleAppDisconnected,
};

const static TimerCfg_t s_TimerCfg = {
    .DevNo = 1,
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 0,			// 0 => Default frequency
	.IntPrio = 6,
	.EvtHandler = TimerHandler
};

Timer g_BleAppTimer;
*/

BtDev_t * const BtDevGetInstance()
{
	return &s_BtDevSdc;
}

__WEAK void BtDevInitCustomSrvc()
{

}
/*
__WEAK void BtDevConnected(unsigned short, unsigned char, unsigned char, unsigned char*)
{

}

__WEAK void BtDevDisconnected(unsigned short, unsigned char)
{

}

__WEAK void BtDevEvtHandler(unsigned long, void*)
{

}
*/
#if 0
static void BtStackMpslAssert(const char * const file, const uint32_t line)
{
//	printf("MPSL Fault: %s, %d\n", file, line);
	while(1);
}

static void BtStackSdcAssert(const char * file, const uint32_t line)
{
//	printf("SDC Fault: %s, %d\n", file, line);
	while(1);
}


static void BtStackSdcCB()
{
	//printf("BtHciSdcCB\n");

	uint8_t buf[HCI_MSG_BUFFER_MAX_SIZE];
	int32_t res = 0;
	sdc_hci_msg_type_t mtype;

	res = sdc_hci_get(buf, &mtype);
	if (res == 0)
	{
		switch (mtype)
		{
			case SDC_HCI_MSG_TYPE_EVT:
				// Event available
				BtHciProcessEvent(s_pBtDevSdc, (BtHciEvtPacket_t*)buf);
				break;
			case SDC_HCI_MSG_TYPE_DATA:
				BtHciProcessData(s_pBtDevSdc, (BtHciACLDataPacket_t*)buf);
				break;
		}
	}
}

void TimerHandler(TimerDev_t *pTimer, uint32_t Evt)
{
    if (Evt & TIMER_EVT_TRIGGER(0))
    {

    }
}
#endif

void BtDevSetDevName(const char *pName)
{
	BleAdvPacket_t *advpkt;

	if (s_BtDevSdc.bExtAdv == true)
	{
		advpkt = &s_BtDevExtAdvPkt;
	}
	else
	{
		advpkt = &s_BtDevAdvPkt;
	}

	BleAdvDataSetDevName(advpkt, pName);

	BtGapSetDevName(pName);
}
/*
char * const BleAppGetDevName()
{
	//return s_BtGapCharDevName;
}

*/
#if 0
bool isBtDevConnected()
{
	return s_pBtDevSdc->ConnHdl != 0;
}

static void BtDevConnLedOff()
{
	if (s_pBtDevSdc->ConnLedPort < 0 || s_pBtDevSdc->ConnLedPin < 0)
		return;

	if (s_pBtDevSdc->ConnLedActLevel)
	{
	    IOPinClear(s_pBtDevSdc->ConnLedPort, s_pBtDevSdc->ConnLedPin);
	}
	else
	{
	    IOPinSet(s_pBtDevSdc->ConnLedPort, s_pBtDevSdc->ConnLedPin);
	}
}

static void BtDevConnLedOn()
{
	if (s_pBtDevSdc->ConnLedPort < 0 || s_pBtDevSdc->ConnLedPin < 0)
		return;

    if (s_pBtDevSdc->ConnLedActLevel)
    {
        IOPinSet(s_pBtDevSdc->ConnLedPort, s_pBtDevSdc->ConnLedPin);
    }
    else
    {
        IOPinClear(s_pBtDevSdc->ConnLedPort, s_pBtDevSdc->ConnLedPin);
    }
}
#endif

void BtDevEvtHandler(uint32_t Evt, void * const pCtx)
{

}

void BtDevConnected(uint16_t ConnHdl, uint8_t Role, uint8_t PeerAddrType, uint8_t PeerAddr[6])
{
	s_BtDevSdc.ConnHdl = ConnHdl;

	BtGapAddConnection(ConnHdl, Role, PeerAddrType, PeerAddr);
//	s_BtGapSrvc.ConnHdl = ConnHdl;
//	s_BtGattSrvc.ConnHdl = ConnHdl;

	//BtAppEvtConnected(ConnHdl);
}

void BtDevDisconnected(uint16_t ConnHdl, uint8_t Reason)
{
//	s_BtGapSrvc.ConnHdl = BT_GATT_HANDLE_INVALID;
//	s_BtGattSrvc.ConnHdl = BT_GATT_HANDLE_INVALID;

	BtGapDeleteConnection(ConnHdl);

	if (isBtGapConnected() == false)
	{
//		BtGattSrvcDisconnected(&s_BtGapSrvc);
//		BtGattSrvcDisconnected(&s_BtGattSrvc);

		//BtAppEvtDisconnected(ConnHdl);
	}

	s_BtDevSdc.ConnHdl = BtGapGetConnection();

}

void BtDevSendCompleted(uint16_t ConnHdl, uint16_t NbPktSent)
{

}

void BtDevDisconnect()
{
}
/*
void BleAppGapDeviceNameSet(const char* pDeviceName)
{
	BleAdvPacket_t *advpkt;

	if (g_BleAppData.bExtAdv == true)
	{
		advpkt = &s_BleAppExtAdvPkt;
	}
	else
	{
		advpkt = &s_BleAppAdvPkt;
	}

	size_t l = strlen(pDeviceName);
	uint8_t type = BT_GAP_DATA_TYPE_COMPLETE_LOCAL_NAME;

	if (l < 14)
	{
		// Short name
		type = BT_GAP_DATA_TYPE_SHORT_LOCAL_NAME;
	}

	BleAdvDataAdd(advpkt, type, (uint8_t*)pDeviceName, l);

	BtGattCharSetValue(&s_BtGapChar[0], (void*)pDeviceName, l);
}
*/

bool BtDevAdvManDataSet(uint8_t * const pAdvData, int AdvLen, uint8_t * const pSrData, int SrLen)
{
	if (s_BtDevSdc.State != BTDEV_STATE_ADVERTISING && s_BtDevSdc.State != BTDEV_STATE_IDLE)
	{
		return false;
	}

	BleAdvPacket_t *advpkt;
	BleAdvPacket_t *srpkt;

	if (s_BtDevSdc.bExtAdv == true)
	{
		advpkt = &s_BtDevExtAdvPkt;
		srpkt = &s_BtDevExtSrPkt;
	}
	else
	{
		advpkt = &s_BtDevAdvPkt;
		srpkt = &s_BtDevSrPkt;
	}

	if (pAdvData)
	{
		int l = AdvLen + 2;
		BleAdvData_t *p = BleAdvDataAllocate(advpkt, BT_GAP_DATA_TYPE_MANUF_SPECIFIC_DATA, l);

		if (p == NULL)
		{
			return false;
		}
		*(uint16_t *)p->Data = s_BtDevSdc.VendorId;
		memcpy(&p->Data[2], pAdvData, AdvLen);

    	if (s_BtDevSdc.bExtAdv == true)
    	{
    		s_BtDevExtAdvData.adv_handle = 0;
    		s_BtDevExtAdvData.operation = 3;
    		s_BtDevExtAdvData.fragment_preference = 1;
    		s_BtDevExtAdvData.adv_data_length = advpkt->Len;

    		int res = sdc_hci_cmd_le_set_ext_adv_data(&s_BtDevExtAdvData);
    		if (res != 0)
    		{
    			return false;
    		}
    	}
    	else
    	{
			s_BtDevAdvData.adv_data_length = s_BtDevAdvPkt.Len;

			int res = sdc_hci_cmd_le_set_adv_data(&s_BtDevAdvData);
    		if (res != 0)
    		{
    			return false;
    		}
    	}
	}

	if (pSrData)
	{
		int l = SrLen + 2;
		BleAdvData_t *p = BleAdvDataAllocate(srpkt, BT_GAP_DATA_TYPE_MANUF_SPECIFIC_DATA, l);

		if (p == NULL)
		{
			return false;
		}
		*(uint16_t *)p->Data = s_BtDevSdc.VendorId;
		memcpy(&p->Data[2], pAdvData, AdvLen);

    	if (s_BtDevSdc.bExtAdv == false)
    	{
			s_BtDevSrData.scan_response_data_length = s_BtDevSrPkt.Len;

			int res = sdc_hci_cmd_le_set_scan_response_data(&s_BtDevSrData);
    		if (res != 0)
    		{
    			return false;
    		}
    	}
	}

	return true;
}

void BtDevAdvStart()
{
	if (s_BtDevSdc.State == BTDEV_STATE_ADVERTISING)// || g_BleAppData.ConnHdl != BLE_CONN_HANDLE_INVALID)
		return;

	int res = 0;

	if (s_BtDevSdc.bExtAdv == true)
	{
		uint8_t buff[100];

		sdc_hci_cmd_le_set_ext_adv_enable_t *x = (sdc_hci_cmd_le_set_ext_adv_enable_t*)buff;

		x->enable = 1;
		x->num_sets = 1;
		x->array_params[0].adv_handle = 0;
		x->array_params[0].duration = 0;
		x->array_params[0].max_ext_adv_events = 0;

		res = sdc_hci_cmd_le_set_ext_adv_enable(x);
	}
	else
	{
		sdc_hci_cmd_le_set_adv_enable_t x = { 1 };

		res = sdc_hci_cmd_le_set_adv_enable(&x);
	}

	if (res == 0)
	{
		//g_BleAppData.bAdvertising = true;
		s_BtDevSdc.State = BTDEV_STATE_ADVERTISING;
	}
}

void BtDevAdvStop()
{
	int res = 0;

	if (s_BtDevSdc.bExtAdv == true)
	{
		uint8_t buff[100];

		sdc_hci_cmd_le_set_ext_adv_enable_t *x = (sdc_hci_cmd_le_set_ext_adv_enable_t*)buff;

		x->enable = 0;
		x->num_sets = 1;
		x->array_params[0].adv_handle = 0;
		x->array_params[0].duration = 0;
		x->array_params[0].max_ext_adv_events = 0;

		res = sdc_hci_cmd_le_set_ext_adv_enable(x);
	}
	else
	{
		sdc_hci_cmd_le_set_adv_enable_t x = { 0 };

		res = sdc_hci_cmd_le_set_adv_enable(&x);
	}

	s_BtDevSdc.State = BTDEV_STATE_IDLE;
}

/**@brief Overloadable function for initializing the Advertising functionality.
 */
bool BtDevAdvInit(const BtDevCfg_t * const pCfg)
{
	uint8_t flags = BT_GAP_DATA_TYPE_FLAGS_NO_BREDR;
	uint16_t extprop = 0;//BLE_EXT_ADV_EVT_PROP_LEGACY;
	BleAdvPacket_t *advpkt;
	BleAdvPacket_t *srpkt;

	//g_BleAppTimer.Init(s_TimerCfg);

	if (s_BtDevSdc.bExtAdv == true)
	{
		advpkt = &s_BtDevExtAdvPkt;
		srpkt = &s_BtDevExtSrPkt;
	}
	else
	{
		advpkt = &s_BtDevAdvPkt;
		srpkt = &s_BtDevSrPkt;
	}

	if (pCfg->Role & BTDEV_ROLE_PERIPHERAL)
	{
		if (pCfg->AdvTimeout != 0)
		{
			flags |= BT_GAP_DATA_TYPE_FLAGS_LIMITED_DISCOVERABLE;
		}
		else
		{
			flags |= BT_GAP_DATA_TYPE_FLAGS_GENERAL_DISCOVERABLE;
		}
		extprop |= BLE_EXT_ADV_EVT_PROP_CONNECTABLE;// | BLE_EXT_ADV_EVT_PROP_SCANNABLE;
	}
	else if (pCfg->Role & BTDEV_ROLE_BROADCASTER)
	{
		//extprop |= BLE_EXT_ADV_EVT_PROP_OMIT_ADDR;
		//extprop |= BLE_EXT_ADV_EVT_PROP_SCANNABLE;
		//extprop = 0;
		//flags |= GAP_DATA_TYPE_FLAGS_LIMITED_DISCOVERABLE;
	}

	if (BleAdvDataAdd(advpkt, BT_GAP_DATA_TYPE_FLAGS, &flags, 1) == false)
	{
		return false;
	}

    if (pCfg->Appearance != BLE_APPEAR_UNKNOWN_GENERIC)
    {
    	if (BleAdvDataAdd(advpkt, BT_GAP_DATA_TYPE_APPEARANCE, (uint8_t*)&pCfg->Appearance, 2) == false)
    	{
//    		return false;
    	}
    }

    BleAdvPacket_t *uidadvpkt;

    if (pCfg->pDevName != NULL)
    {
    	size_t l = strlen(pCfg->pDevName);
    	uint8_t type = BT_GAP_DATA_TYPE_COMPLETE_LOCAL_NAME;
    	size_t mxl = advpkt->MaxLen - advpkt->Len - 2;

    	if (l > 30 || l > mxl)
    	{
    		// Short name
    		type = BT_GAP_DATA_TYPE_SHORT_LOCAL_NAME;
    		l = min((size_t)30, mxl);
    	}

    	if (BleAdvDataAdd(advpkt, type, (uint8_t*)pCfg->pDevName, l) == false)
    	{
    		return false;
    	}

    	uidadvpkt = pCfg->bExtAdv ? advpkt : srpkt;
    }
    else
    {
    	uidadvpkt = advpkt;
    }

    if (pCfg->pAdvUuid != NULL && pCfg->Role & BTDEV_ROLE_PERIPHERAL)
    {
    	if (BleAdvDataAddUuid(uidadvpkt, pCfg->pAdvUuid, pCfg->bCompleteUuidList) == false)
    	{

    	}

    }

	if (pCfg->pAdvManData != NULL)
	{
		int l = pCfg->AdvManDataLen + 2;
		BleAdvData_t *p = BleAdvDataAllocate(advpkt, BT_GAP_DATA_TYPE_MANUF_SPECIFIC_DATA, l);

		if (p == NULL)
		{
			return false;
		}
		*(uint16_t *)p->Data = pCfg->VendorId;
		memcpy(&p->Data[2], pCfg->pAdvManData, pCfg->AdvManDataLen);
	}

	if (pCfg->pSrManData != NULL)
	{
		int l = pCfg->SrManDataLen + 2;
		BleAdvData_t *p = BleAdvDataAllocate(srpkt, BT_GAP_DATA_TYPE_MANUF_SPECIFIC_DATA, l);

		if (p == NULL)
		{
			return false;
		}
		*(uint16_t *)p->Data = pCfg->VendorId;
		memcpy(&p->Data[2], pCfg->pSrManData, pCfg->SrManDataLen);
	}

	if (s_BtDevSdc.bExtAdv == false)
	{
		sdc_hci_cmd_le_set_adv_params_t advparam = {
			.adv_interval_min = (uint16_t)mSecTo0_625(pCfg->AdvInterval),
			.adv_interval_max = (uint16_t)mSecTo0_625(pCfg->AdvInterval + 50),
			.adv_type = BLEADV_TYPE_ADV_NONCONN_IND,//ADV_DIRECT_IND,
			.own_address_type = BLE_ADDR_TYPE_PUBLIC,
			.peer_address_type = 0,
			.peer_address = {0,},
			.adv_channel_map = 7,
			.adv_filter_policy = 0
		};

		if (pCfg->Role & BTDEV_ROLE_PERIPHERAL)
		{
			advparam.adv_type = BLEADV_TYPE_ADV_IND;
		}

		int sdc_res = sdc_hci_cmd_le_set_adv_params(&advparam);

		if (sdc_res != 0)
		{
			return false;
		}

		s_BtDevAdvData.adv_data_length = advpkt->Len;

		sdc_res = sdc_hci_cmd_le_set_adv_data(&s_BtDevAdvData);
		if (sdc_res != 0)
		{
			return false;
		}

		if (srpkt->Len > 0)
		{
			s_BtDevSrData.scan_response_data_length = srpkt->Len;

			sdc_res = sdc_hci_cmd_le_set_scan_response_data(&s_BtDevSrData);
			if (sdc_res != 0)
			{
				return false;
			}
		}
	}
	else
	{
		// Use extended advertisement

		BleExtAdvParam_t extparam = {
			.AdvHdl = 0,
			.EvtProp = extprop,//BLE_EXT_ADV_EVT_PROP_CONNECTABLE,// | BLE_EXT_ADV_EVT_PROP_SCANNABLE,
			.PrimIntervalMin = (uint16_t)mSecTo0_625(pCfg->AdvInterval),
			.PrimIntervalMax = (uint16_t)mSecTo0_625(pCfg->AdvInterval + 50),
			.PrimChanMap = 7,
			.OwnAddrType = BLE_ADDR_TYPE_PUBLIC,
			.PrimPhy = BLE_EXT_ADV_PHY_1M,
			.SecondPhy = BLE_EXT_ADV_PHY_2M,
			.ScanNotifEnable = 0,
		};

		sdc_hci_cmd_le_set_ext_adv_params_t &exadvparm = *(sdc_hci_cmd_le_set_ext_adv_params_t*)&extparam;
		sdc_hci_cmd_le_set_ext_adv_params_return_t rexadvparm;
		int res = sdc_hci_cmd_le_set_ext_adv_params(&exadvparm, &rexadvparm);

		if (res != 0)
		{
			return false;
//			printf("sdc_hci_cmd_le_set_ext_adv_params : %x\n", res);
		}

		s_BtDevExtAdvData.adv_handle = 0;
		s_BtDevExtAdvData.operation = 3;
		s_BtDevExtAdvData.fragment_preference = 1;
		s_BtDevExtAdvData.adv_data_length = advpkt->Len;

		res = sdc_hci_cmd_le_set_ext_adv_data(&s_BtDevExtAdvData);
		if (res != 0)
		{
			return false;
//			printf("sdc_hci_cmd_le_set_ext_adv_data : %x\n", res);
		}

		if (srpkt->Len > 0)
		{
			s_BtDevExtSrData.adv_handle = 0;
			s_BtDevExtSrData.operation = 3;
			s_BtDevExtSrData.fragment_preference = 1;
			s_BtDevExtSrData.scan_response_data_length = advpkt->Len;

			res = sdc_hci_cmd_le_set_ext_scan_response_data(&s_BtDevExtSrData);
			if (res != 0)
			{
				return false;
//				printf("sdc_hci_cmd_le_set_ext_adv_data : %x\n", res);
			}
		}
	}

	return true;
}

void BtDevDisInit(BtDev_t * const pDev, const BtDevCfg_t *pCfg)
{
#if 0
    ble_dis_init_t   dis_init;
    ble_dis_pnp_id_t pnp_id;

    // Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));

	if (pBleAppCfg->pDevDesc)
	{
		ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, (char*)pBleAppCfg->pDevDesc->ManufName);
		ble_srv_ascii_to_utf8(&dis_init.model_num_str, (char*)pBleAppCfg->pDevDesc->ModelName);
		if (pBleAppCfg->pDevDesc->pSerialNoStr)
			ble_srv_ascii_to_utf8(&dis_init.serial_num_str, (char*)pBleAppCfg->pDevDesc->pSerialNoStr);
		if (pBleAppCfg->pDevDesc->pFwVerStr)
			ble_srv_ascii_to_utf8(&dis_init.fw_rev_str, (char*)pBleAppCfg->pDevDesc->pFwVerStr);
		if (pBleAppCfg->pDevDesc->pHwVerStr)
			ble_srv_ascii_to_utf8(&dis_init.hw_rev_str, (char*)pBleAppCfg->pDevDesc->pHwVerStr);
	}

    pnp_id.vendor_id_source = BLE_DIS_VENDOR_ID_SRC_BLUETOOTH_SIG;
    pnp_id.vendor_id  = pBleAppCfg->VendorID;
    pnp_id.product_id = pBleAppCfg->ProductId;
    dis_init.p_pnp_id = &pnp_id;

    //BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init.dis_attr_md.read_perm);
    //BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init.dis_attr_md.write_perm);

    switch (pBleAppCfg->SecType)
    {
    	case BLEAPP_SECTYPE_STATICKEY_NO_MITM:
    	    dis_init.dis_char_rd_sec = SEC_JUST_WORKS;
    		break;
    	case BLEAPP_SECTYPE_STATICKEY_MITM:
    	    dis_init.dis_char_rd_sec = SEC_MITM;
    		break;
    	case BLEAPP_SECTYPE_LESC_MITM:
    	    dis_init.dis_char_rd_sec = SEC_JUST_WORKS;
    		break;
    	case BLEAPP_SECTYPE_SIGNED_NO_MITM:
    	    dis_init.dis_char_rd_sec = SEC_SIGNED;
    		break;
    	case BLEAPP_SECTYPE_SIGNED_MITM:
    	    dis_init.dis_char_rd_sec = SEC_SIGNED_MITM;
    		break;
    	case BLEAPP_SECTYPE_NONE:
    	default:
    	    dis_init.dis_char_rd_sec = SEC_OPEN;
    	    break;
    }

    uint32_t err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
#endif

}

#if 0
uint16_t BtDevGetConnHandle()
{
	return s_pBtDevSdc->ConnHdl;
}
/**@brief Function for handling events from the GATT library. */
void BleGattEvtHandler(nrf_ble_gatt_t * p_gatt, const nrf_ble_gatt_evt_t * p_evt)
{
    if ((g_BleAppData.ConnHdl == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
    	//g_BleAppData.MaxMtu = p_evt->params.att_mtu_effective - 3;//OPCODE_LENGTH - HANDLE_LENGTH;
       // m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        //NRF_LOG_INFO("Data len is set to 0x%X(%d)\r\n", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
 //   printf("ATT MTU exchange completed. central 0x%x peripheral 0x%x\r\n", p_gatt->att_mtu_desired_central, p_gatt->att_mtu_desired_periph);
}
#endif
/**@brief Function for initializing the GATT library. */
void BtDevGattInit(BtDev_t * const pDev)
{
#if 0
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&s_Gatt, BleGattEvtHandler);
    APP_ERROR_CHECK(err_code);

    if (g_BleAppData.AppRole & BLEAPP_ROLE_PERIPHERAL)
    {
    	err_code = nrf_ble_gatt_att_mtu_periph_set(&s_Gatt, g_BleAppData.MaxMtu);
    	APP_ERROR_CHECK(err_code);

    	if (g_BleAppData.MaxMtu >= 27)
    	{
    		// 251 bytes is max dat length as per Bluetooth core spec 5, vol 6, part b, section 4.5.10
    		// 27 - 251 bytes is hardcoded in nrf_ble_gat of the SDK.
    		uint8_t dlen = g_BleAppData.MaxMtu > 254 ? 251: g_BleAppData.MaxMtu - 3;
    		err_code = nrf_ble_gatt_data_length_set(&s_Gatt, BLE_CONN_HANDLE_INVALID, dlen);
    		APP_ERROR_CHECK(err_code);
    	}
      	ble_opt_t opt;

      	memset(&opt, 0x00, sizeof(opt));
      	opt.common_opt.conn_evt_ext.enable = 1;

      	err_code = sd_ble_opt_set(BLE_COMMON_OPT_CONN_EVT_EXT, &opt);
      	APP_ERROR_CHECK(err_code);
    }

    if (g_BleAppData.AppRole & BLEAPP_ROLE_CENTRAL)
    {
    	err_code = nrf_ble_gatt_att_mtu_central_set(&s_Gatt, g_BleAppData.MaxMtu);
    	APP_ERROR_CHECK(err_code);
    }
#endif
}

bool BtDevConnectable(const BtDevCfg_t *pBleAppCfg, bool bEraseBond)
{

#if 0
	uint32_t err_code;

    //BleAppInitUserData();

	BleAppGapParamInit(pBleAppCfg);

	//gatt_init();

	if (pBleAppCfg->AppMode != BLEAPP_MODE_NOCONNECT)
		conn_params_init();

	BleAppInitUserServices();

	if (pBleAppCfg->bEnDevInfoService)
		BleAppDisInit(pBleAppCfg);
#endif
	return true;
}

#if 0
static uint8_t BtStackRandPrioLowGet(uint8_t *pBuff, uint8_t Len)
{
	for (int i = 0; i < Len; i++)
	{
		pBuff[i] = rand();
	}

	return Len;
}

static uint8_t BtStackRandPrioHighGet(uint8_t *pBuff, uint8_t Len)
{
	return BtStackRandPrioLowGet(pBuff, Len);
}

static void BtStackRandPrioLowGetBlocking(uint8_t *pBuff, uint8_t Len)
{
	NRF_RNG->CONFIG = RNG_CONFIG_DERCEN_Enabled;

	NRF_RNG->TASKS_START = 1;

	for (int i = 0; i < Len; i++)
	{
		while (NRF_RNG->EVENTS_VALRDY == 0);

		pBuff[i] = NRF_RNG->VALUE;
	}

	NRF_RNG->TASKS_STOP = 1;

	NRF_RNG->CONFIG = RNG_CONFIG_DERCEN_Disabled;
}

bool BtDevStackInit(const BtDevCfg_t *pCfg)
{
	// Initialize Nordic Softdevice controller
	int32_t res = sdc_init(BtStackSdcAssert);

	sdc_hci_cmd_cb_reset();

	sdc_rand_source_t rand_functions = {
		.rand_prio_low_get = BtStackRandPrioLowGet,
		.rand_prio_high_get = BtStackRandPrioHighGet,
		.rand_poll = BtStackRandPrioLowGetBlocking
	};

	res = sdc_rand_source_register(&rand_functions);

	sdc_support_le_2m_phy();
	sdc_support_le_coded_phy();
	//sdc_support_le_power_control();

	if (pCfg->Role & (BTDEV_ROLE_PERIPHERAL | BTDEV_ROLE_BROADCASTER))
	{
		// Config for peripheral role
		res = sdc_support_adv();
		res = sdc_support_ext_adv();
		sdc_support_le_periodic_adv();
		sdc_support_le_periodic_sync();
		sdc_support_peripheral();
		sdc_support_dle_peripheral();
		sdc_support_phy_update_peripheral();
		sdc_support_le_power_control_peripheral();
		sdc_support_le_conn_cte_rsp_peripheral();

		if (pCfg->CoexMode != BTDEV_COEXMODE_NONE)
		{
			sdc_coex_adv_mode_configure(true);
		}
	}
	if (pCfg->Role & (BTDEV_ROLE_CENTRAL | BTDEV_ROLE_OBSERVER))
	{
		// Config for central role
		sdc_support_scan();
		sdc_support_ext_scan();
		sdc_support_central();
		sdc_support_ext_central();
		sdc_support_dle_central();
		sdc_support_phy_update_central();
		sdc_support_le_power_control_central();
		sdc_support_le_conn_cte_rsp_central();
	}

    uint32_t ram = 0;
	sdc_cfg_t cfg;

	int l = pCfg->MaxMtu == 0 ? BTDEV_DEFAULT_MAX_DATA_LEN : pCfg->MaxMtu;
	cfg.buffer_cfg.rx_packet_size = l;
	cfg.buffer_cfg.tx_packet_size = l;
	cfg.buffer_cfg.rx_packet_count = 4;
	cfg.buffer_cfg.tx_packet_count = 4;

	ram = sdc_cfg_set(SDC_DEFAULT_RESOURCE_CFG_TAG,
				       	  SDC_CFG_TYPE_BUFFER_CFG,
						  &cfg);
	if (ram < 0)
	{
		return false;
	}

	cfg.event_length.event_length_us = 7500;
	ram = sdc_cfg_set(SDC_DEFAULT_RESOURCE_CFG_TAG,
				       	  SDC_CFG_TYPE_EVENT_LENGTH,
						  &cfg);
	if (ram < 0)
	{
		return false;
	}

	if (pCfg->Role & (BTDEV_ROLE_PERIPHERAL | BTDEV_ROLE_BROADCASTER))
	{
		// Config for peripheral role
		cfg.peripheral_count.count = pCfg->PeriLinkCount;

		ram = sdc_cfg_set(SDC_DEFAULT_RESOURCE_CFG_TAG,
					       	  SDC_CFG_TYPE_PERIPHERAL_COUNT,
							  &cfg);
		if (ram < 0)
		{
			return false;
		}

		cfg.adv_count.count = 1;

		ram = sdc_cfg_set(SDC_DEFAULT_RESOURCE_CFG_TAG,
							  SDC_CFG_TYPE_ADV_COUNT,
							  &cfg);
		if (ram < 0)
		{
			return false;
		}

		cfg.adv_buffer_cfg.max_adv_data = 255;

		ram = sdc_cfg_set(SDC_DEFAULT_RESOURCE_CFG_TAG,
							  SDC_CFG_TYPE_ADV_BUFFER_CFG,
							  &cfg);
		if (ram < 0)
		{
			return false;
		}
	}

	if (pCfg->Role & (BTDEV_ROLE_CENTRAL | BTDEV_ROLE_OBSERVER))
	{
		// Config for central role
		cfg.central_count.count = pCfg->CentLinkCount;
		ram = sdc_cfg_set(SDC_DEFAULT_RESOURCE_CFG_TAG,
					       	  SDC_CFG_TYPE_CENTRAL_COUNT,
							  &cfg);
		if (ram < 0)
		{
			return false;
		}


		cfg.scan_buffer_cfg.count = 10;

		ram = sdc_cfg_set(SDC_DEFAULT_RESOURCE_CFG_TAG,
						  SDC_CFG_TYPE_SCAN_BUFFER_CFG,
						  &cfg);
		if (ram < 0)
		{
			return false;
		}
	}

	if (sizeof(s_BtStackSdcMemPool) < ram)
	{
		return false;
	}
    // Enable BLE stack.
	res = sdc_enable(BtStackSdcCB, s_BtStackSdcMemPool);
	if (res != 0)
	{
		return false;
	}

    return true;
}
#endif

int8_t GetValidTxPower(int TxPwr)
{
	int8_t retval = s_TxPowerdBm[0];

	for (int i = 1; i < s_NbTxPowerdBm; i++)
	{
		if (s_TxPowerdBm[i] > TxPwr)
			break;

		retval = s_TxPowerdBm[i];
	}

	return retval;
}

/**
 * @brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
bool BtDevInit(const BtDevCfg_t *pCfg)
{
	int32_t res = 0;
#if 0
	mpsl_clock_lfclk_cfg_t lfclk = {MPSL_CLOCK_LF_SRC_RC, 0,};
	OscDesc_t const *lfosc = GetLowFreqOscDesc();

	s_pBtDevSdc = pDev;

	//s_BleStarted = false;
	s_pBtDevSdc->State = BTDEV_STATE_UNKNOWN;

	// Set default clock based on system oscillator settings
	if (lfosc->Type == OSC_TYPE_RC)
	{
		lfclk.source = MPSL_CLOCK_LF_SRC_RC;
		lfclk.rc_ctiv = MPSL_RECOMMENDED_RC_CTIV;
		lfclk.rc_temp_ctiv = MPSL_RECOMMENDED_RC_TEMP_CTIV;
	}
	else
	{
		lfclk.accuracy_ppm = lfosc->Accuracy;
		lfclk.source = MPSL_CLOCK_LF_SRC_XTAL;
	}

	lfclk.skip_wait_lfclk_started = MPSL_DEFAULT_SKIP_WAIT_LFCLK_STARTED;

	// Initialize Nordic multi-protocol support library (MPSL)
	res = mpsl_init(&lfclk, PendSV_IRQn, BtStackMpslAssert);
	if (res < 0)
	{
		return false;
	}

	NVIC_SetPriority(PendSV_IRQn, MPSL_HIGH_IRQ_PRIORITY + 15);
	NVIC_EnableIRQ(PendSV_IRQn);

	pDev->CoexMode = pCfg->CoexMode;

	if (pCfg->CoexMode == BTDEV_COEXMODE_1W)
	{
		mpsl_coex_support_1wire_gpiote_if();
	}
	else if (pCfg->CoexMode == BTDEV_COEXMODE_3W)
	{
		mpsl_coex_support_802152_3wire_gpiote_if();
	}
#endif
	s_BtDevSdc.State = BTDEV_STATE_UNKNOWN;
    s_BtDevSdc.Role = pCfg->Role;
	s_BtDevSdc.bExtAdv = pCfg->bExtAdv;
	s_BtDevSdc.VendorId = pCfg->VendorId;
	s_BtDevSdc.ProductId = pCfg->ProductId;
	s_BtDevSdc.ProductVer = pCfg->ProductVer;
	s_BtDevSdc.Appearance = pCfg->Appearance;
	//s_BtDevSdc.ConnLedPort = pCfg->ConnLedPort;
	//s_BtDevSdc.ConnLedPin = pCfg->ConnLedPin;
	//s_BtDevSdc.ConnLedActLevel = pCfg->ConnLedActLevel;
	s_BtDevSdc.EvtHandler = BtDevEvtHandler;
	s_BtDevSdc.Connected = BtDevConnected;
	s_BtDevSdc.Disconnected = BtDevDisconnected;
	s_BtDevSdc.SendData = BtDevSendData;
	s_BtDevSdc.SendCompleted = BtDevSendCompleted;

#if 0
	if (pCfg->ConnLedPort != -1 && pCfg->ConnLedPin != -1)
    {
		IOPinConfig(pCfg->ConnLedPort, pCfg->ConnLedPin, 0,
					IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);

		BtDevConnLedOff();
    }


    if (BtDevStackInit(pCfg) == false)
    {
    	return false;
    }

	uint8_t abuf[100];

	memset(abuf, 0, 100);

	sdc_hci_cmd_vs_zephyr_read_static_addresses_return_t *addr = (sdc_hci_cmd_vs_zephyr_read_static_addresses_return_t *)abuf;

	res = sdc_hci_cmd_vs_zephyr_read_static_addresses(addr);
	if (res == 0)
	{
		sdc_hci_cmd_vs_zephyr_write_bd_addr_t bdaddr;

		memcpy(bdaddr.bd_addr, addr->addresses->address, 6);
		sdc_hci_cmd_vs_zephyr_write_bd_addr(&bdaddr);
	}

	sdc_hci_cmd_le_rand_return_t rr;

	res = sdc_hci_cmd_le_rand(&rr);
	if (res == 0)
	{
		sdc_hci_cmd_le_set_random_address_t ranaddr;
		memcpy(ranaddr.random_address, &rr.random_number, 6);
		if (sdc_hci_cmd_le_set_random_address(&ranaddr))
			return false;
	}

	sdc_hci_cmd_le_read_max_data_length_return_t maxlen;

	res = sdc_hci_cmd_le_read_max_data_length(&maxlen);

	sdc_hci_cmd_le_write_suggested_default_data_length_t datalen = {
		(uint16_t)max(maxlen.supported_max_tx_octets, pCfg->MaxMtu),
		maxlen.supported_max_tx_time
	};

	res = sdc_hci_cmd_le_write_suggested_default_data_length(&datalen);

	sdc_hci_cmd_le_read_buffer_size_return_t rbr;

	res = sdc_hci_cmd_le_read_buffer_size(&rbr);

	sdc_default_tx_power_set(pCfg->TxPower);

	sdc_hci_cmd_le_set_event_mask_t evmask = {0, };

	//evmask.params.le_remote_connection_parameter_request_event = 1;

	memset(evmask.raw, 0xff, sizeof(evmask.raw));
	if (sdc_hci_cmd_le_set_event_mask(&evmask))
	{
		return false;
	}

	sdc_hci_cmd_cb_set_event_mask_t cbevmask;

	memset(cbevmask.raw, 0xff, sizeof(cbevmask.raw));
	if (sdc_hci_cmd_cb_set_event_mask(&cbevmask))
	{
		return false;
	}

	sdc_hci_cmd_cb_set_event_mask_page_2_t cbevmask2;
	memset(cbevmask2.raw, 0xff, sizeof(cbevmask2.raw));
	if (sdc_hci_cmd_cb_set_event_mask_page_2(&cbevmask2))
	{
		return false;
	}

//	if (pCfg->)
//    BleAppInitUserData();
	BtDevInitCustomData(pDev);
#endif

    //BtGapInit(pCfg->Role);

    if (pCfg->Role & (BTDEV_ROLE_BROADCASTER | BTDEV_ROLE_PERIPHERAL))
    {
    	if (pCfg->Role & BTDEV_ROLE_PERIPHERAL)
    	{
//    		BtGattSrvcAdd(&s_BtGattSrvc, &s_BtGattSrvcCfg);
//    		BtGattSrvcAdd(&s_BtGapSrvc, &s_BtGapSrvcCfg);

    		BtGapServiceInit();//&s_BtDevSdc.Srvc[s_BtDevSdc.NbSrvc]);
//    		s_BtDevSdc.NbSrvc++;
//    		BtGattServiceInit(&s_BtDevSdc.Srvc[s_BtDevSdc.NbSrvc]);
//    		s_BtDevSdc.NbSrvc++;

    //		BleAppInitUserServices();
    		BtDevInitCustomSrvc();
    	}

    	if (BtDevAdvInit(pCfg) == false)
    	{
    		return false;
    	}

#if 0
    	size_t count = 0;
    	BtGattListEntry_t *tbl = GetEntryTable(&count);
    	for (int i = 0; i < count; i++)
    	{
    		g_Uart.printf("tbl[%d]: Hdl: %d (0x%04x), Uuid: %04x, Data: ", i, tbl[i].Hdl, tbl[i].Hdl, tbl[i].TypeUuid.Uuid);
    		uint8_t *p = (uint8_t*)&tbl[i].Val32;
    		for (int j = 0; j < 20; j++)
    		{
    			g_Uart.printf("0x%02x ", p[j]);
    		}
    		g_Uart.printf("\r\n");
    	}
#endif
    }

	BtGapSetDevName(pCfg->pDevName);

    //BleAppGapDeviceNameSet(pBleAppCfg->pDevName);
#if 0
#if (__FPU_USED == 1)
    // Patch for softdevice & FreeRTOS to sleep properly when FPU is in used
    NVIC_SetPriority(FPU_IRQn, 6);
    NVIC_ClearPendingIRQ(FPU_IRQn);
    NVIC_EnableIRQ(FPU_IRQn);
#endif

    if (AppEvtHandlerInit(pCfg->pEvtHandlerQueMem, pCfg->EvtHandlerQueMemSize) == false)
    {
    	return false;
    }

	//BtHciInit(&s_BtDevCfg);
#endif
    s_BtDevSdc.State = BTDEV_STATE_INITIALIZED;


	return true;
}

bool BtDevEnableNotify(uint16_t ConnHandle, uint16_t CharHandle)//ble_uuid_t * const pCharUid)
{
//	BtGattCharNotify();

	return false;
#if 0
    uint32_t                 err_code;
    ble_gattc_write_params_t write_params;
    uint8_t                  buf[BLE_CCCD_VALUE_LEN];

    buf[0] = BLE_GATT_HVX_NOTIFICATION;
    buf[1] = 0;

    write_params.write_op = BLE_GATT_OP_WRITE_CMD;//BLE_GATT_OP_WRITE_REQ;
    write_params.flags = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE,
    write_params.handle   = CharHandle;
    write_params.offset   = 0;
    write_params.len      = sizeof(buf);
    write_params.p_value  = buf;

    err_code = sd_ble_gattc_write(ConnHandle, &write_params);

    return err_code == NRF_SUCCESS;
#endif
}

bool BtDevNotify(BtGattChar_t *pChar, uint8_t *pData, uint16_t DataLen)
{
	if (BtGattCharSetValue(pChar, pData, DataLen) == false)
	{
		return false;
	}

	if (isBtGattCharNotifyEnabled(pChar) == false)
	{
		return false;
	}

	BtHciNotify(&s_BtDevSdc, s_BtDevSdc.ConnHdl, pChar->ValHdl, pData, DataLen);

	return true;
}

bool BtDevWrite(BtGattChar_t * const pChar, uint8_t * const pData, uint16_t DataLen)
{
	return false;
#if 0
	if (ConnHandle == BLE_CONN_HANDLE_INVALID || CharHandle == BLE_CONN_HANDLE_INVALID)
	{
		return false;
	}

    ble_gattc_write_params_t const write_params =
    {
        .write_op = BLE_GATT_OP_WRITE_CMD,
        .flags    = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE,
        .handle   = CharHandle,
        .offset   = 0,
        .len      = DataLen,
        .p_value  = pData
    };

    return sd_ble_gattc_write(ConnHandle, &write_params) == NRF_SUCCESS;
#endif
}
#if 0
bool BtDevAddSrvc(const BtGattSrvcCfg_t *pSrvcCfg)
{
	if (pSrvcCfg == nullptr)
	{
		return false;
	}

	if (s_BtDevSdc.NbSrvc >= BTDEV_SERVICE_MAXCNT)
	{
		return false;
	}

	bool res = BtGattSrvcAdd(&s_BtDevSdc.Srvc[s_BtDevSdc.NbSrvc], pSrvcCfg);

	if (res == true)
	{
		s_BtDevSdc.NbSrvc++;
	}

	return res;
}
#endif

