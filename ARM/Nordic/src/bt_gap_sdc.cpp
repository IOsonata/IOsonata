/**-------------------------------------------------------------------------
@file	bt_gap_sdc.cpp

@brief	Implement Bluetooth Generic Access Profile (GAP)

Implementation on Nordic nrfxlib Softdevice Controller

Core Bluetooth Vol.1, Part A, 6.2

The Bluetooth system defines a base profile which all Bluetooth devices implement.
This profile is the Generic Access Profile (GAP), which defines the basic
requirements of a Bluetooth device. For instance, for BR/EDR, it defines a
Bluetooth device to include the Radio, Baseband, Link Manager, L2CAP, and the
Service Discovery protocol functionality; for LE, it defines the Physical Layer,
Link Layer, L2CAP, Security Manager, Attribute Protocol and Generic Attribute Profile.
This ties all the various layers together to form the basic requirements for a
Bluetooth device. It also describes the behaviors and methods for device discovery,
connection establishment, security, authentication, association models and
service discovery.

@author	Hoang Nguyen Hoan
@date	Jan. 20, 2024

@license

MIT License

Copyright (c) 2024 I-SYST inc. All rights reserved.

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

#include "sdc_hci_cmd_le.h"

#include "convutil.h"
#include "bluetooth/bt_gap.h"

#include "coredev/uart.h"
extern UART g_Uart;

BtGapScanParam_t s_ScanParams;

bool BtGapScanInit(BtGapScanCfg_t * const pCfg)
{
	uint8_t buff[255];
	sdc_hci_cmd_le_set_ext_scan_params_t *extparam = (sdc_hci_cmd_le_set_ext_scan_params_t *)buff;
	sdc_hci_cmd_le_set_scan_params_t param;

	memcpy(&s_ScanParams, &pCfg->Param, sizeof(BtGapScanParam_t));
	extparam->own_address_type = 1;
	extparam->scanning_filter_policy = 0;
	extparam->scanning_phys = pCfg->Param.Phy;
	extparam->array_params[0] = { 1, mSecTo0_625(pCfg->Param.Interval), mSecTo0_625(pCfg->Param.Duration) };

	param.le_scan_type = pCfg->Type;
	param.own_address_type = pCfg->Param.OwnAddrType;
	param.scanning_filter_policy = 0; // TODO
	param.le_scan_interval = mSecTo0_625(pCfg->Param.Interval);
	param.le_scan_window = mSecTo0_625(pCfg->Param.Duration);

	//uint8_t res = sdc_hci_cmd_le_set_scan_params(&param);
	//g_Uart.printf("sdc_hci_cmd_le_set_scan_params : 0x%x (%d)\r\n", res, res);
	uint8_t res = sdc_hci_cmd_le_set_ext_scan_params(extparam);
g_Uart.printf("sdc_hci_cmd_le_set_ext_scan_params : 0x%x (%d)\r\n", res, res);
	return res == 0;
}

bool BtGapScanStart(uint8_t * const pBuff, uint16_t Len)
{
	sdc_hci_cmd_le_set_ext_scan_enable_t extparam = { 1, 1, 0, 0 };
	sdc_hci_cmd_le_set_scan_enable_t param = { 1, 0 };

	uint8_t //res = sdc_hci_cmd_le_set_scan_enable(&param);
	res = sdc_hci_cmd_le_set_ext_scan_enable(&extparam);
	
	g_Uart.printf("sdc_hci_cmd_le_set_ext_scan_enable : 0x%x (%d)\r\n", res, res);
	return res == 0;
}

void BtGapScanStop()
{
	sdc_hci_cmd_le_set_ext_scan_enable_t extparam = {0,};

	uint8_t res = sdc_hci_cmd_le_set_ext_scan_enable(&extparam);
}

bool BtGapScanNext(uint8_t * const pBuff, uint16_t Len)
{
	
	return true;
}

bool BtGapConnect(BtGapPeerAddr_t * const pPeerAddr, BtGapConnParams_t * const pConnParam)//, BtGapScanParam_t * const pScanParam)
{
	sdc_hci_cmd_le_create_conn_t param = {
		.le_scan_interval = mSecTo0_625(s_ScanParams.Interval),
		.le_scan_window = mSecTo0_625(s_ScanParams.Duration),
		.initiator_filter_policy = 0,
		.peer_address_type = pPeerAddr->Type,
		.peer_address = {
				pPeerAddr->Addr[0], pPeerAddr->Addr[1], pPeerAddr->Addr[2],
				pPeerAddr->Addr[3], pPeerAddr->Addr[4], pPeerAddr->Addr[5]},
		.own_address_type = s_ScanParams.OwnAddrType,
		.conn_interval_min = mSecTo1_25(pConnParam->IntervalMin),
		.conn_interval_max = mSecTo1_25(pConnParam->IntervalMax),
		.max_latency = pConnParam->Latency,
		.supervision_timeout = (uint16_t)(pConnParam->Timeout / 10),
		.min_ce_length = 0,
		.max_ce_length = 0
	};

	uint8_t res = sdc_hci_cmd_le_create_conn(&param);

	return res == 0;
/*
	sdc_hci_cmd_le_ext_create_conn_t extparam = {
		.initiator_filter_policy = 0,
		.own_address_type = 0,
		.peer_address_type = pPeerAddr->Type,
		.peer_address = {
				pPeerAddr->Addr[0], pPeerAddr->Addr[1], pPeerAddr->Addr[2],
				pPeerAddr->Addr[3], pPeerAddr->Addr[4], pPeerAddr->Addr[5]},
		.initiating_phys = 0,
		.array_params = {0,},
	};*/

#if 0
	ble_gap_scan_params_t scparam = {};
	ble_gap_conn_params_t cparam;
	ble_gap_addr_t addr = { .addr_id_peer = 0, .addr_type = pPeerAddr->Type, };

	memcpy(addr.addr,  pPeerAddr->Addr, 6);

	cparam.min_conn_interval = MSEC_TO_UNITS(pConnParam->IntervalMin, UNIT_1_25_MS);
	cparam.max_conn_interval = MSEC_TO_UNITS(pConnParam->IntervalMax, UNIT_1_25_MS);
	cparam.slave_latency = pConnParam->Latency;
	cparam.conn_sup_timeout = MSEC_TO_UNITS(pConnParam->Timeout, UNIT_10_MS);


	ret_code_t err_code = sd_ble_gap_connect(&addr, &s_ScanParams, &cparam,
											 BT_GAP_CONN_CFG_TAG);
    //APP_ERROR_CHECK(err_code);

//    s_BtAppData.bScan = false;

    return err_code == NRF_SUCCESS;
#endif
}

