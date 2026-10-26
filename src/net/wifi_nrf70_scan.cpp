/**-------------------------------------------------------------------------
@file wifi_nrf70_scan.cpp
@brief Adapter to Nordic's nRF70 bare-metal scan library.
@license MIT License
Copyright (c) 2026 I-SYST inc.

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
-------------------------------------------------------------------------*/
#include "net/wifi_nrf70_scan.h"

#ifdef IO_ENABLE_NRF70_BM
#include "system/nrf70_bm_lib.h"
#include <string.h>

static WifiNrf70ScanResultCb_t s_Callback;
static void *s_pContext;
static bool s_bInitialized;
static bool s_bBusy;

static void ScanResult(struct nrf70_bm_sys_scan_result *pEntry)
{
	WifiNrf70ScanResult_t result;

	if (pEntry == NULL)
	{
		s_bBusy = false;
		if (s_Callback != NULL)
			s_Callback(NULL, true, s_pContext);
		return;
	}
	if (s_Callback == NULL)
		return;
	memset(&result, 0, sizeof(result));
	result.SsidLen = pEntry->ssid_len;
	if (result.SsidLen > WIFI_NRF70_SSID_MAX)
		result.SsidLen = WIFI_NRF70_SSID_MAX;
	memcpy(result.Ssid, pEntry->ssid, result.SsidLen);
	memcpy(result.Bssid, pEntry->bssid, 6);
	result.Band = pEntry->band;
	result.Channel = pEntry->channel;
	result.Rssi = pEntry->rssi;
	result.Security = (uint8_t)pEntry->security;
	s_Callback(&result, false, s_pContext);
}

extern "C" bool WifiNrf70ScanInit(uint8_t pMac[6], void *pRegInfo)
{
	if (s_bInitialized || pMac == NULL || pRegInfo == NULL)
		return false;
	if (nrf70_bm_sys_init(pMac,
		(struct nrf70_bm_regulatory_info *)pRegInfo) != 0)
		return false;
	s_bInitialized = true;
	return true;
}

extern "C" bool WifiNrf70ScanStart(uint8_t Bands, uint16_t MaxResults,
	WifiNrf70ScanResultCb_t Callback, void *pContext)
{
	struct nrf70_bm_sys_scan_params params;

	if (!s_bInitialized || s_bBusy || Callback == NULL || Bands == 0)
		return false;
	memset(&params, 0, sizeof(params));
	params.scan_type = NRF70_SCAN_TYPE_PASSIVE;
	params.bands = Bands;
	params.max_bss_cnt = MaxResults;
	params.dwell_time_passive = 130;
	s_Callback = Callback;
	s_pContext = pContext;
	s_bBusy = true;
	if (nrf70_bm_sys_scan_start(&params, ScanResult) == 0)
		return true;
	s_bBusy = false;
	s_Callback = NULL;
	s_pContext = NULL;
	return false;
}

extern "C" void WifiNrf70ScanClose(void)
{
	if (s_bInitialized)
		nrf70_bm_sys_deinit();
	s_bInitialized = false;
	s_bBusy = false;
	s_Callback = NULL;
	s_pContext = NULL;
}
extern "C" bool WifiNrf70ScanBusy(void) { return s_bBusy; }

#else
extern "C" bool WifiNrf70ScanInit(uint8_t pMac[6], void *pRegInfo)
{
	(void)pMac;
	(void)pRegInfo;
	return false;
}
extern "C" bool WifiNrf70ScanStart(uint8_t Bands, uint16_t MaxResults,
	WifiNrf70ScanResultCb_t Callback, void *pContext)
{
	(void)Bands;
	(void)MaxResults;
	(void)Callback;
	(void)pContext;
	return false;
}
extern "C" void WifiNrf70ScanClose(void) {}
extern "C" bool WifiNrf70ScanBusy(void) { return false; }
#endif
