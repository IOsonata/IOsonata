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

bool BtGapScanInit(BtGapScanCfg_t * const pCfg)
{
	uint8_t buff[255];
	sdc_hci_cmd_le_set_ext_scan_params_t *param = (sdc_hci_cmd_le_set_ext_scan_params_t *)buff;

	param->own_address_type = 1;
	param->scanning_filter_policy = 0;
	param->scanning_phys = 1;
	param->array_params[0] = { 1, mSecTo0_625(pCfg->Interval), mSecTo0_625(pCfg->Duration) };


	uint8_t res = sdc_hci_cmd_le_set_ext_scan_params(param);
g_Uart.printf("sdc_hci_cmd_le_set_ext_scan_params : 0x%x (%d)\r\n", res, res);
	return res == 0;
}

bool BtGapScanStart(uint8_t * const pBuff, uint16_t Len)
{
	sdc_hci_cmd_le_set_ext_scan_enable_t param = { 1, 1, 0, 0 };

	uint8_t res = sdc_hci_cmd_le_set_ext_scan_enable(&param);
	
	g_Uart.printf("sdc_hci_cmd_le_set_ext_scan_enable : 0x%x (%d)\r\n", res, res);
	return res == 0;
}

bool BtGapScanNext(uint8_t * const pBuff, uint16_t Len)
{
	
	return true;
}

