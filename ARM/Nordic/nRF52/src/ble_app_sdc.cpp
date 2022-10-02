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
#include "sdc.h"
#include "sdc_soc.h"
#include "sdc_hci_cmd_le.h"
#include "sdc_hci.h"
#include "sdc_hci_vs.h"
#include "sdc_hci_cmd_controller_baseband.h"

#include "istddef.h"
#include "coredev/uart.h"
#include "custom_board.h"
#include "coredev/iopincfg.h"
#include "coredev/system_core_clock.h"
#include "bluetooth/ble_app.h"
#include "bluetooth/ble_hcidef.h"
#include "iopinctrl.h"

#pragma pack(push, 4)

typedef struct _BleAppData {
	BLEAPP_ROLE Role;
	uint16_t ConnHdl;	// BLE connection handle
	int ConnLedPort;
	int ConnLedPin;
	uint8_t ConnLedActLevel;
	int PeriphDevCnt;
//	BLEAPP_PERIPH *pPeriphDev;
	uint32_t (*SDEvtHandler)(void) ;
	uint8_t AdvData[31];
	uint8_t SrData[31];
	uint8_t ExtAdvData[255];
    //ble_advdata_manuf_data_t ManufData;
    //ble_advdata_manuf_data_t SRManufData;
	int MaxMtu;
	bool bSecure;
	bool bAdvertising;
	bool bScan;
} BleAppData_t;

#pragma pack(pop)

// S132 tx_power values: -40dBm, -20dBm, -16dBm, -12dBm, -8dBm, -4dBm, 0dBm, +3dBm and +4dBm
// S140 tx_power values: -40dBm, -20dBm, -16dBm, -12dBm, -8dBm, -4dBm, 0dBm, +2dBm, +3dBm, +4dBm, +5dBm, +6dBm, +7dBm and +8dBm.

static const int8_t s_TxPowerdBm[] = {
	-40, -20, -16, -12, -8, -4, 0,
#if defined(S132) || defined(NRF52832_XXAA)
	3, 4
#else
	2, 3, 4, 5, 6, 7, 8
#endif
};

static const int s_NbTxPowerdBm = sizeof(s_TxPowerdBm) / sizeof(int8_t);

BleAppData_t g_BleAppData = {
	BLEAPP_ROLE_PERIPHERAL, 0, -1, -1, 0,
};

//#endif

/**@brief Bluetooth SIG debug mode Private Key */
__ALIGN(4) __WEAK extern const uint8_t g_lesc_private_key[32] = {
    0xbd,0x1a,0x3c,0xcd,0xa6,0xb8,0x99,0x58,0x99,0xb7,0x40,0xeb,0x7b,0x60,0xff,0x4a,
    0x50,0x3f,0x10,0xd2,0xe3,0xb3,0xc9,0x74,0x38,0x5f,0xc5,0xa3,0xd4,0xf6,0x49,0x3f,
};

//__ALIGN(4) static ble_gap_lesc_p256_pk_t    s_lesc_public_key;      /**< LESC ECC Public Key */
//__ALIGN(4) static ble_gap_lesc_dhkey_t      s_lesc_dh_key;          /**< LESC ECC DH Key*/

alignas(4) static uint8_t s_BleStackSdcMemPool[6000];

static void BleStackMpslAssert(const char * const file, const uint32_t line)
{
	printf("MPSL Fault: %s, %d\n", file, line);
	while(1);
}

static void BleStackSdcAssert(const char * file, const uint32_t line)
{
	printf("Softdevice Controller Fault: %s, %d\n", file, line);
	while(1);
}

static void BleStackProcessEvent(uint8_t *pEvtBuf)
{
	BleHciEvtPacketHdr_t *hdr = (BleHciEvtPacketHdr_t *)pEvtBuf;

	switch (hdr->Evt)
	{
		case BLE_HCI_EVT_LE_META:
			break;
	}
}

static void BleStackSdcCB()
{
	//printf("BleHciSdcCB\n");

	uint8_t buf[512];
	int32_t res = 0;

	res = sdc_hci_evt_get(buf);
	if (res == 0)
	{
		// Event available
		BleStackProcessEvent(buf);
	}

	res = sdc_hci_data_get(buf);
	if (res == 0)
	{

	}
}

bool isConnected()
{
	return g_BleAppData.ConnHdl != 0;
}

static void BleConnLedOff() {
	if (g_BleAppData.ConnLedPort < 0 || g_BleAppData.ConnLedPin < 0)
		return;

	if (g_BleAppData.ConnLedActLevel)
	{
	    IOPinClear(g_BleAppData.ConnLedPort, g_BleAppData.ConnLedPin);
	}
	else
	{
	    IOPinSet(g_BleAppData.ConnLedPort, g_BleAppData.ConnLedPin);
	}
}

static void BleConnLedOn() {
	if (g_BleAppData.ConnLedPort < 0 || g_BleAppData.ConnLedPin < 0)
		return;

    if (g_BleAppData.ConnLedActLevel)
    {
        IOPinSet(g_BleAppData.ConnLedPort, g_BleAppData.ConnLedPin);
    }
    else
    {
        IOPinClear(g_BleAppData.ConnLedPort, g_BleAppData.ConnLedPin);
    }
}

void BleAppEnterDfu()
{
}

void BleAppDisconnect()
{
}

void BleAppGapDeviceNameSet(const char* pDeviceName)
{
    uint32_t res;

//    err_code = sd_ble_gap_device_name_set(&s_gap_conn_mode,
//                                          (const uint8_t *)pDeviceName,
//                                          strlen( pDeviceName ));
//    APP_ERROR_CHECK(err_code);
//    ble_advertising_restart_without_whitelist(&g_AdvInstance);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */

static void BleAppGapParamInit(const BleAppCfg_t *pBleAppCfg)
{
/*    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;

    switch (pBleAppCfg->SecType)
    {
    	case BLEAPP_SECTYPE_NONE:
    	    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&s_gap_conn_mode);
    	    break;
		case BLEAPP_SECTYPE_STATICKEY_NO_MITM:
		    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&s_gap_conn_mode);
    	    break;
		case BLEAPP_SECTYPE_STATICKEY_MITM:
		    BLE_GAP_CONN_SEC_MODE_SET_ENC_WITH_MITM(&s_gap_conn_mode);
    	    break;
		case BLEAPP_SECTYPE_LESC_MITM:
			BLE_GAP_CONN_SEC_MODE_SET_LESC_ENC_WITH_MITM(&s_gap_conn_mode);
    	    break;
		case BLEAPP_SECTYPE_SIGNED_NO_MITM:
			BLE_GAP_CONN_SEC_MODE_SET_SIGNED_NO_MITM(&s_gap_conn_mode);
    	    break;
		case BLEAPP_SECTYPE_SIGNED_MITM:
			BLE_GAP_CONN_SEC_MODE_SET_SIGNED_WITH_MITM(&s_gap_conn_mode);
    	    break;
    }
    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_UNKNOWN);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    if (pBleAppCfg->AppMode != BLEAPP_MODE_NOCONNECT)
    {
		gap_conn_params.min_conn_interval = pBleAppCfg->ConnIntervalMin;// MIN_CONN_INTERVAL;
		gap_conn_params.max_conn_interval = pBleAppCfg->ConnIntervalMax;//MAX_CONN_INTERVAL;
		gap_conn_params.slave_latency     = SLAVE_LATENCY;
		gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

		err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
		APP_ERROR_CHECK(err_code);
    }*/
}

bool BleAppAdvManDataSet(uint8_t *pAdvData, int AdvLen, uint8_t *pSrData, int SrLen)
{/*
	uint32_t err;

	if (pAdvData && AdvLen > 0)
	{
		int l = min(AdvLen, BLE_GAP_ADV_SET_DATA_SIZE_MAX);

		memcpy(g_BleAppData.ManufData.data.p_data, pAdvData, l);

		g_BleAppData.ManufData.data.size = l;
        g_AdvInstance.adv_data.adv_data.len = BLE_GAP_ADV_SET_DATA_SIZE_MAX;

        err = ble_advdata_encode(&g_BleAppData.AdvData, g_AdvInstance.adv_data.adv_data.p_data, &g_AdvInstance.adv_data.adv_data.len);
		APP_ERROR_CHECK(err);

	}

	if (pSrData && SrLen > 0)
	{
		int l = min(SrLen, BLE_GAP_ADV_SET_DATA_SIZE_MAX);

		memcpy(g_BleAppData.SRManufData.data.p_data, pSrData, l);

		g_BleAppData.SRManufData.data.size = l;
		g_AdvInstance.adv_data.scan_rsp_data.len = BLE_GAP_ADV_SET_DATA_SIZE_MAX;

		uint32_t err = ble_advdata_encode(&g_BleAppData.SrData, g_AdvInstance.adv_data.scan_rsp_data.p_data,
										 &g_AdvInstance.adv_data.scan_rsp_data.len);
		APP_ERROR_CHECK(err);
	}

	// SDK15 doesn't allow dynamically updating adv data.  Have to stop and re-start advertising
	if (g_BleAppData.bAdvertising == true)
	{
		sd_ble_gap_adv_stop(g_AdvInstance.adv_handle);
	}

	err = sd_ble_gap_adv_set_configure(&g_AdvInstance.adv_handle, &g_AdvInstance.adv_data, NULL);
//	APP_ERROR_CHECK(err);

	if (g_BleAppData.bAdvertising == true)
	{
		g_BleAppData.bAdvertising = false;
		BleAppAdvStart(BLEAPP_ADVMODE_FAST);
	}

#if 0
    int l = min(Len, BLE_GAP_ADV_MAX_SIZE);

    memcpy(g_AdvInstance.manuf_data_array, pData, l);
    uint32_t ret = ble_advdata_set(&(g_AdvInstance.advdata), &g_BleAppData.SRData);
#endif

    return g_BleAppData.bAdvertising;*/
}

void BleAppAdvStart(BLEADV_TYPE AdvType)
{
	if (g_BleAppData.bAdvertising == true)// || g_BleAppData.ConnHdl != BLE_CONN_HANDLE_INVALID)
		return;

	g_BleAppData.bAdvertising = true;

}

void BleAppAdvStop()
{
//	sd_ble_gap_adv_stop(g_AdvInstance.adv_handle);
	g_BleAppData.bAdvertising = false;
}

/**@brief Overloadable function for initializing the Advertising functionality.
 */
__WEAK void BleAppAdvInit(const BleAppCfg_t *pCfg)
{
#if 0
    uint32_t               err_code;
//    ble_advdata_manuf_data_t mdata;
    ble_advertising_init_t	initdata;

    memset(&initdata, 0, sizeof(ble_advertising_init_t));

    if (pCfg->pAdvManData != NULL)
    {
		g_BleAppData.ManufData.company_identifier = pCfg->VendorID;
		g_BleAppData.ManufData.data.p_data = (uint8_t*)pCfg->pAdvManData;
		g_BleAppData.ManufData.data.size = pCfg->AdvManDataLen;
    }
    if (pCfg->pSrManData != NULL)
    {
		g_BleAppData.SRManufData.company_identifier = pCfg->VendorID;
		g_BleAppData.SRManufData.data.p_data = (uint8_t*)pCfg->pSrManData;
		g_BleAppData.SRManufData.data.size = pCfg->SrManDataLen;
    }
    // Build advertising data struct to pass into @ref ble_advertising_init.

    initdata.advdata.include_appearance = false;

    if (pCfg->AdvTimeout != 0)
    {
        // ADV for a limited time, use this flag
        initdata.advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
    }
    else
    {
        // Always ADV use this flag
        initdata.advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    }

    if (pCfg->pDevName != NULL)
    {
    	if (strlen(pCfg->pDevName) < 14)
    	{
    		initdata.advdata.name_type      = BLE_ADVDATA_SHORT_NAME;
    		initdata.advdata.short_name_len = strlen(pCfg->pDevName);
    	}
    	else
    		initdata.advdata.name_type = BLE_ADVDATA_FULL_NAME;
    }
    else
    {
    		initdata.advdata.name_type = BLE_ADVDATA_NO_NAME;
    }

    if (initdata.advdata.name_type == BLE_ADVDATA_NO_NAME)
    {
        if (pCfg->NbAdvUuid > 0 && pCfg->pAdvUuids != NULL)
        {
			if (pCfg->pAdvManData != NULL)
			{
				initdata.advdata.p_manuf_specific_data = &g_BleAppData.ManufData;
	        	initdata.srdata.uuids_complete.uuid_cnt = pCfg->NbAdvUuid;
	        	initdata.srdata.uuids_complete.p_uuids  = (ble_uuid_t*)pCfg->pAdvUuids;
			}
			else
			{
	        	initdata.advdata.uuids_complete.uuid_cnt = pCfg->NbAdvUuid;
	        	initdata.advdata.uuids_complete.p_uuids  = (ble_uuid_t*)pCfg->pAdvUuids;
				if (pCfg->pSrManData != NULL)
				{
		        	initdata.srdata.p_manuf_specific_data = &g_BleAppData.SRManufData;
				}
			}
        }
        else
        {
			if (pCfg->pAdvManData != NULL)
			{
				initdata.advdata.p_manuf_specific_data = &g_BleAppData.ManufData;
			}
			if (pCfg->pSrManData != NULL)
			{
	        	initdata.srdata.p_manuf_specific_data = &g_BleAppData.SRManufData;
			}
        }
    }
    else
    {
        if (pCfg->NbAdvUuid > 0 && pCfg->pAdvUuids != NULL)
        {
			initdata.srdata.uuids_complete.uuid_cnt = pCfg->NbAdvUuid;
			initdata.srdata.uuids_complete.p_uuids  = (ble_uuid_t*)pCfg->pAdvUuids;

			if (pCfg->pAdvManData != NULL)
			{
				initdata.advdata.p_manuf_specific_data = &g_BleAppData.ManufData;
			}

        }
        else
        {
        	if (pCfg->pAdvManData != NULL)
			{
				initdata.advdata.p_manuf_specific_data = &g_BleAppData.ManufData;
			}
			if (pCfg->pSrManData != NULL)
			{
				initdata.srdata.p_manuf_specific_data = &g_BleAppData.SRManufData;
			}
        }
    }

    if (pCfg->AppMode == BLEAPP_MODE_NOCONNECT || pCfg->AppMode == BLEAPP_MODE_IBEACON)
    {
//        err_code = ble_advdata_encode(&initdata.advdata, g_AdvData.adv_data.p_data, &g_AdvData.adv_data.len);
        g_AdvInstance.adv_data.adv_data.len = BLE_GAP_ADV_SET_DATA_SIZE_MAX;
        g_AdvInstance.adv_data.adv_data.p_data = g_AdvInstance.enc_advdata[0];

        err_code = ble_advdata_encode(&initdata.advdata, g_AdvInstance.adv_data.adv_data.p_data, &g_AdvInstance.adv_data.adv_data.len);
        APP_ERROR_CHECK(err_code);

        //err_code = ble_advdata_set(&initdata.advdata, &initdata.srdata);
        //APP_ERROR_CHECK(err_code);

        // Initialize advertising parameters (used when starting advertising).
        memset(&g_AdvInstance.adv_params, 0, sizeof(g_AdvInstance.adv_params));


        g_AdvInstance.adv_params.properties.type	= BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;//BLE_GAP_ADV_TYPE_ADV_NONCONN_IND;
        g_AdvInstance.adv_params.p_peer_addr 		= NULL;                             // Undirected advertisement.
        g_AdvInstance.adv_params.filter_policy		= BLE_GAP_ADV_FP_ANY;
        g_AdvInstance.adv_params.interval    		= pCfg->AdvInterval;
        g_AdvInstance.adv_params.duration     		= pCfg->AdvTimeout;
#if 0
        s_AdvParams.properties.type  = BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;//BLE_GAP_ADV_TYPE_ADV_NONCONN_IND;
        s_AdvParams.p_peer_addr = NULL;                             // Undirected advertisement.
        s_AdvParams.filter_policy = BLE_GAP_ADV_FP_ANY;
        s_AdvParams.interval    = pCfg->AdvInterval;
        s_AdvParams.duration     = pCfg->AdvTimeout;
#endif
        err_code = sd_ble_gap_adv_set_configure(&g_AdvInstance.adv_handle, &g_AdvInstance.adv_data, &g_AdvInstance.adv_params);
        APP_ERROR_CHECK(err_code);

    }
    else
    {
		//memset(&options, 0, sizeof(options));
		initdata.config.ble_adv_fast_enabled  = true;
		initdata.config.ble_adv_fast_interval = pCfg->AdvInterval;
		initdata.config.ble_adv_fast_timeout  = pCfg->AdvTimeout;

		if (pCfg->AdvSlowInterval > 0)
		{
			initdata.config.ble_adv_slow_enabled  = true;
			initdata.config.ble_adv_slow_interval = pCfg->AdvSlowInterval;
			initdata.config.ble_adv_slow_timeout  = BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED;
		}
	    memcpy(&g_BleAppData.SrData, &initdata.srdata, sizeof(ble_advdata_t));

	    initdata.evt_handler = on_adv_evt;
	    err_code = ble_advertising_init(&g_AdvInstance, &initdata);
	    APP_ERROR_CHECK(err_code);
    }

    memcpy(&g_BleAppData.AdvData, &initdata.advdata, sizeof(ble_advdata_t));

	// Bypass local copy of manufacturer data of the SDK
	//g_AdvInstance.manuf_specific_data.data.p_data = initdata.advdata.p_manuf_specific_data->data.p_data;
	//g_AdvInstance.advdata.p_manuf_specific_data->data.size = initdata.advdata.p_manuf_specific_data->data.size;

	ble_advertising_conn_cfg_tag_set(&g_AdvInstance, BLEAPP_CONN_CFG_TAG);
#endif
}

void BleAppDisInit(const BleAppCfg_t *pBleAppCfg)
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

uint16_t BleAppGetConnHandle()
{
	return g_BleAppData.ConnHdl;
}
#if 0
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
void BleAppGattInit(void)
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

bool BleAppConnectable(const BleAppCfg_t *pBleAppCfg, bool bEraseBond)
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

static uint8_t BleStackRandPrioLowGet(uint8_t *pBuff, uint8_t Len)
{
	for (int i = 0; i < Len; i++)
	{
		pBuff[i] = rand();
	}

	return Len;
}

static uint8_t BleStackRandPrioHighGet(uint8_t *pBuff, uint8_t Len)
{
	return BleStackRandPrioLowGet(pBuff, Len);
}

static void BleStackRandPrioLowGetBlocking(uint8_t *pBuff, uint8_t Len)
{
	BleStackRandPrioLowGet(pBuff, Len);
}

bool BleAppStackInit(const BleAppCfg_t *pBleAppCfg)
{
	// Initialize Nordic Softdevice controller
	int32_t res = sdc_init(BleStackSdcAssert);

	sdc_rand_source_t rand_functions = {
		.rand_prio_low_get = BleStackRandPrioLowGet,
		.rand_prio_high_get = BleStackRandPrioHighGet,
		.rand_poll = BleStackRandPrioLowGetBlocking
	};

	res = sdc_rand_source_register(&rand_functions);

	sdc_support_dle();
	sdc_support_le_2m_phy();
	sdc_support_le_coded_phy();
	sdc_support_le_power_control();

	if (pBleAppCfg->Role != BLEAPP_ROLE_CENTRAL)
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
//		sdc_coex_adv_mode_configure(true);
	}
	if (pBleAppCfg->Role != BLEAPP_ROLE_PERIPHERAL)
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

	sdc_default_tx_power_set(pBleAppCfg->TxPower);

    uint32_t ram = 0;
	sdc_cfg_t cfg;

	cfg.central_count.count = pBleAppCfg->CentLinkCount;
	ram = sdc_cfg_set(SDC_DEFAULT_RESOURCE_CFG_TAG,
				       	  SDC_CFG_TYPE_CENTRAL_COUNT,
						  &cfg);
	if (ram < 0)
	{
		return false;
	}

	cfg.peripheral_count.count = pBleAppCfg->PeriLinkCount;

	ram = sdc_cfg_set(SDC_DEFAULT_RESOURCE_CFG_TAG,
				       	  SDC_CFG_TYPE_PERIPHERAL_COUNT,
						  &cfg);
	if (ram < 0)
	{
		return false;
	}

	cfg.buffer_cfg.rx_packet_size = pBleAppCfg->MaxMtu;//MAX_RX_PACKET_SIZE;
	cfg.buffer_cfg.tx_packet_size = pBleAppCfg->MaxMtu;//MAX_TX_PACKET_SIZE;
	cfg.buffer_cfg.rx_packet_count = 4;//CONFIG_BT_CTLR_SDC_RX_PACKET_COUNT;
	cfg.buffer_cfg.tx_packet_count = 4;//CONFIG_BT_CTLR_SDC_TX_PACKET_COUNT;

	ram = sdc_cfg_set(SDC_DEFAULT_RESOURCE_CFG_TAG,
				       	  SDC_CFG_TYPE_BUFFER_CFG,
						  &cfg);
	if (ram < 0)
	{
		return false;
	}

	cfg.event_length.event_length_us = 7426;
//		CONFIG_BT_CTLR_SDC_MAX_CONN_EVENT_LEN_DEFAULT;
	ram = sdc_cfg_set(SDC_DEFAULT_RESOURCE_CFG_TAG,
				       	  SDC_CFG_TYPE_EVENT_LENGTH,
						  &cfg);
	if (ram < 0)
	{
		return false;
	}

	if (pBleAppCfg->Role != BLEAPP_ROLE_CENTRAL)
	{
		// Config for peripheral role
		cfg.adv_count.count = 1;//SDC_ADV_SET_COUNT;

		ram = sdc_cfg_set(SDC_DEFAULT_RESOURCE_CFG_TAG,
							  SDC_CFG_TYPE_ADV_COUNT,
							  &cfg);
		if (ram < 0)
		{
			return false;
		}

		cfg.adv_buffer_cfg.max_adv_data = 255;//SDC_DEFAULT_ADV_BUF_SIZE;

		ram = sdc_cfg_set(SDC_DEFAULT_RESOURCE_CFG_TAG,
							  SDC_CFG_TYPE_ADV_BUFFER_CFG,
							  &cfg);
		if (ram < 0)
		{
			return false;
		}
	}

	if (pBleAppCfg->Role != BLEAPP_ROLE_PERIPHERAL)
	{
		// Config for central role
		cfg.scan_buffer_cfg.count = 10;

		ram = sdc_cfg_set(SDC_DEFAULT_RESOURCE_CFG_TAG,
						  SDC_CFG_TYPE_SCAN_BUFFER_CFG,
						  &cfg);
		if (ram < 0)
		{
			return false;
		}
	}
#if 0
    // Overwrite some of the default configurations for the BLE stack.
    ble_cfg_t ble_cfg;

    // Configure the number of custom UUIDS.
    memset(&ble_cfg, 0, sizeof(ble_cfg));
    //if (CentLinkCount > 0)
        ble_cfg.common_cfg.vs_uuid_cfg.vs_uuid_count = BLESVC_UUID_BASE_MAXCNT;
    //else
    //	ble_cfg.common_cfg.vs_uuid_cfg.vs_uuid_count = 2;
    err_code = sd_ble_cfg_set(BLE_COMMON_CFG_VS_UUID, &ble_cfg, ram_start);
    APP_ERROR_CHECK(err_code);


	// Configure the maximum ATT MTU.
	memset(&ble_cfg, 0x00, sizeof(ble_cfg));
	ble_cfg.conn_cfg.conn_cfg_tag                 = BLEAPP_CONN_CFG_TAG;
	ble_cfg.conn_cfg.params.gatt_conn_cfg.att_mtu = g_BleAppData.MaxMtu;
	err_code = sd_ble_cfg_set(BLE_CONN_CFG_GATT, &ble_cfg, ram_start);
	APP_ERROR_CHECK(err_code);

	// Configure the maximum event length.
	memset(&ble_cfg, 0x00, sizeof(ble_cfg));
	ble_cfg.conn_cfg.conn_cfg_tag                     = BLEAPP_CONN_CFG_TAG;
	ble_cfg.conn_cfg.params.gap_conn_cfg.event_length = 320;
	ble_cfg.conn_cfg.params.gap_conn_cfg.conn_count   = PeriLinkCount + CentLinkCount;//BLE_GAP_CONN_COUNT_DEFAULT;
	err_code = sd_ble_cfg_set(BLE_CONN_CFG_GAP, &ble_cfg, ram_start);
	APP_ERROR_CHECK(err_code);

    memset(&ble_cfg, 0, sizeof(ble_cfg));
    ble_cfg.gatts_cfg.attr_tab_size.attr_tab_size = 3000;
    err_code = sd_ble_cfg_set(BLE_GATTS_CFG_ATTR_TAB_SIZE, &ble_cfg, ram_start);
    APP_ERROR_CHECK(err_code);

    memset(&ble_cfg, 0, sizeof(ble_cfg));
    ble_cfg.gatts_cfg.service_changed.service_changed = 1;
    err_code = sd_ble_cfg_set(BLE_GATTS_CFG_SERVICE_CHANGED, &ble_cfg, ram_start);
    APP_ERROR_CHECK(err_code);
#if 0
    memset(&ble_cfg, 0, sizeof ble_cfg);
    ble_cfg.conn_cfg.conn_cfg_tag 					= BLEAPP_CONN_CFG_TAG;
    ble_cfg.conn_cfg.params.gatts_conn_cfg.hvn_tx_queue_size = 10;
    err_code = sd_ble_cfg_set(BLE_CONN_CFG_GATTS, &ble_cfg, ram_start);
    APP_ERROR_CHECK(err_code);
#endif
#endif

    // Enable BLE stack.
	res = sdc_enable(BleStackSdcCB, s_BleStackSdcMemPool);

    return true;
}

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
bool BleAppInit(const BleAppCfg_t *pBleAppCfg)
{
	int32_t res = 0;
	mpsl_clock_lfclk_cfg_t lfclk = {MPSL_CLOCK_LF_SRC_RC, 0,};
	OscDesc_t const *lfosc = GetLowFreqOscDesc();

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
	res = mpsl_init(&lfclk, PendSV_IRQn, BleStackMpslAssert);
	if (res < 0)
	{
		return false;
	}

	NVIC_SetPriority(PendSV_IRQn, MPSL_HIGH_IRQ_PRIORITY + 15);
	NVIC_EnableIRQ(PendSV_IRQn);

	g_BleAppData.bScan = false;
	g_BleAppData.bAdvertising = false;

	g_BleAppData.ConnLedPort = pBleAppCfg->ConnLedPort;
	g_BleAppData.ConnLedPin = pBleAppCfg->ConnLedPin;
	g_BleAppData.ConnLedActLevel = pBleAppCfg->ConnLedActLevel;

	if (pBleAppCfg->ConnLedPort != -1 && pBleAppCfg->ConnLedPin != -1)
    {
		IOPinConfig(pBleAppCfg->ConnLedPort, pBleAppCfg->ConnLedPin, 0,
					IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);

		BleConnLedOff();
    }

    g_BleAppData.Role = pBleAppCfg->Role;

    BleAppStackInit(pBleAppCfg);

#if 0
    g_BleAppData.ConnHdl = BLE_CONN_HANDLE_INVALID;

    if (pBleAppCfg->MaxMtu > NRF_BLE_MAX_MTU_SIZE)
		g_BleAppData.MaxMtu = pBleAppCfg->MaxMtu;
    else
    	g_BleAppData.MaxMtu = NRF_BLE_MAX_MTU_SIZE;


    switch (g_BleAppData.AppMode)
    {
		case BLEAPP_MODE_LOOP:
		case BLEAPP_MODE_NOCONNECT:
			// app_timer_init();
			break;
		case BLEAPP_MODE_APPSCHED:
			// app_timer_init();
			APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
			break;
		case BLEAPP_MODE_RTOS:
			if (pBleAppCfg->SDEvtHandler == NULL)
				return false;

			g_BleAppData.SDEvtHandler = pBleAppCfg->SDEvtHandler;

			break;
		default:
				;
    }

//    nrf_ble_lesc_init();

	err_code = nrf_sdh_enable((nrf_clock_lf_cfg_t *)&pBleAppCfg->ClkCfg);
    APP_ERROR_CHECK(err_code);

    // Initialize SoftDevice.
    BleAppStackInit(pBleAppCfg->CentLinkCount, pBleAppCfg->PeriLinkCount,
    				pBleAppCfg->AppMode != BLEAPP_MODE_NOCONNECT);

    //err_code = ble_lesc_init();
    //APP_ERROR_CHECK(err_code);

	if (pBleAppCfg->PeriLinkCount > 0 && pBleAppCfg->AdvInterval > 0)
	{
		g_BleAppData.AppRole |= BLEAPP_ROLE_PERIPHERAL;

		if (pBleAppCfg->pDevName != NULL)
	    {
	        err_code = sd_ble_gap_device_name_set(&s_gap_conn_mode,
	                                          (const uint8_t *) pBleAppCfg->pDevName,
	                                          strlen(pBleAppCfg->pDevName));
	        APP_ERROR_CHECK(err_code);
	    }
	}

    if (pBleAppCfg->AppMode != BLEAPP_MODE_NOCONNECT)
    {
    	BleAppConnectable(pBleAppCfg, bEraseBond);
    }

    if (pBleAppCfg->CentLinkCount > 0)
	{
		g_BleAppData.AppRole |= BLEAPP_ROLE_CENTRAL;
//		ret_code_t err_code = ble_db_discovery_init(BleAppDBDiscoveryHandler);
//		APP_ERROR_CHECK(err_code);
    }

    BleAppGattInit();

    BleAppInitUserData();

    BleAppPeerMngrInit(pBleAppCfg->SecType, pBleAppCfg->SecExchg, bEraseBond);

	if (pBleAppCfg->SecType != BLEAPP_SECTYPE_NONE)
	{
	    g_BleAppData.bSecure = true;
	}
	else
	{
	    g_BleAppData.bSecure = false;
	}

   // err_code = fds_register(fds_evt_handler);
   // APP_ERROR_CHECK(err_code);

    // Generate the ECDH key pair and set public key in the peer-manager.
    //err_code = ble_lesc_ecc_keypair_generate_and_set();
    //APP_ERROR_CHECK(err_code);

    if (g_BleAppData.AppRole & BLEAPP_ROLE_PERIPHERAL)
    {
        BleAppAdvInit(pBleAppCfg);

        err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, g_AdvInstance.adv_handle, GetValidTxPower(pBleAppCfg->TxPower));
        APP_ERROR_CHECK(err_code);
    }
    else
    {
        err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_SCAN_INIT, g_AdvInstance.adv_handle, GetValidTxPower(pBleAppCfg->TxPower));
        APP_ERROR_CHECK(err_code);
    }

#if (__FPU_USED == 1)
    // Patch for softdevice & FreeRTOS to sleep properly when FPU is in used
    NVIC_SetPriority(FPU_IRQn, APP_IRQ_PRIORITY_LOW);
    NVIC_ClearPendingIRQ(FPU_IRQn);
    NVIC_EnableIRQ(FPU_IRQn);
#endif
#endif

    return true;
}

void BleAppRun()
{
#if 0
	g_BleAppData.bAdvertising = false;

	if ((g_BleAppData.AppRole & (BLEAPP_ROLE_PERIPHERAL | BLEAPP_ROLE_CENTRAL)) != BLEAPP_ROLE_CENTRAL)
	{
		BleAppAdvStart(BLEAPP_ADVMODE_FAST);
	}
/*	if (g_BleAppData.AppMode == BLEAPP_MODE_NOCONNECT)
	{
		uint32_t err_code = sd_ble_gap_adv_start(g_AdvInstance.adv_handle, BLEAPP_CONN_CFG_TAG);
		//uint32_t err_code = ble_advertising_start(&g_AdvInstance, BLE_ADV_MODE_FAST);
		APP_ERROR_CHECK(err_code);
	}
	else
	{
		if (g_BleAppData.AppRole & BLEAPP_ROLE_PERIPHERAL)
		{
			uint32_t err_code = ble_advertising_start(&g_AdvInstance, BLE_ADV_MODE_FAST);
			APP_ERROR_CHECK(err_code);
		}
	}
*/
    while (1)
    {
		if (g_BleAppData.AppMode == BLEAPP_MODE_RTOS)
		{
			BleAppRtosWaitEvt();
		}
		else
		{
			if (g_BleAppData.AppMode == BLEAPP_MODE_APPSCHED)
			{
				app_sched_execute();
			}
			nrf_ble_lesc_request_handler();
			sd_app_evt_wait();
		}
    }
#endif
}

void BleAppScan()
{
#if 0
	ret_code_t err_code;

	if (g_BleAppData.bScan == true)
	{
		err_code = sd_ble_gap_scan_start(NULL, &g_BleScanReportData);
	}
	else
	{
	    g_BleAppData.bScan = true;

		err_code = sd_ble_gap_scan_start(&s_BleScanParams, &g_BleScanReportData);
	}
	APP_ERROR_CHECK(err_code);
#endif
}

void BleAppScanStop()
{
	if (g_BleAppData.bScan == true)
	{
		//ret_code_t err_code = sd_ble_gap_scan_stop();
		//APP_ERROR_CHECK(err_code);
		g_BleAppData.bScan = false;
	}
}

bool BleAppScanInit(BleAppScanCfg_t *pCfg)
{
#if 0
	if (pCfg == NULL)
	{
		return false;
	}

	s_BleScanParams.timeout = pCfg->Timeout;
	s_BleScanParams.window = pCfg->Duration;
	s_BleScanParams.interval = pCfg->Interval;

    uint8_t uidtype = BLE_UUID_TYPE_VENDOR_BEGIN;

    ret_code_t err_code = sd_ble_uuid_vs_add(&pCfg->BaseUid, &uidtype);
    APP_ERROR_CHECK(err_code);

    g_BleAppData.bScan = true;

	err_code = sd_ble_gap_scan_start(&s_BleScanParams, &g_BleScanReportData);
	APP_ERROR_CHECK(err_code);

	return err_code == NRF_SUCCESS;
#endif
}

#if 0
bool BleAppScanInit(ble_uuid128_t * const pBaseUid, ble_uuid_t * const pServUid)
{
    ble_uuid128_t base_uid = *pBaseUid;
    uint8_t uidtype = BLE_UUID_TYPE_VENDOR_BEGIN;

    ret_code_t err_code = sd_ble_uuid_vs_add(&base_uid, &uidtype);
    APP_ERROR_CHECK(err_code);

    //ble_db_discovery_evt_register(pServUid);
    g_BleAppData.bScan = true;

	err_code = sd_ble_gap_scan_start(&s_BleScanParams, &g_BleScanReportData);
	APP_ERROR_CHECK(err_code);

	return err_code == NRF_SUCCESS;
}
#endif

#if 0
bool BleAppConnect(ble_gap_addr_t * const pDevAddr, ble_gap_conn_params_t * const pConnParam)
{
	ret_code_t err_code = sd_ble_gap_connect(pDevAddr, &s_BleScanParams,
                                  	  	  	 pConnParam,
											 BLEAPP_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);

    g_BleAppData.bScan = false;

    return err_code == NRF_SUCCESS;
}
#endif

bool BleAppEnableNotify(uint16_t ConnHandle, uint16_t CharHandle)//ble_uuid_t * const pCharUid)
{
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

bool BleAppWrite(uint16_t ConnHandle, uint16_t CharHandle, uint8_t *pData, uint16_t DataLen)
{
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


