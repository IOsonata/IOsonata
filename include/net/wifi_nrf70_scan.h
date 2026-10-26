/**-------------------------------------------------------------------------
@file wifi_nrf70_scan.h
@brief Scan-only adapter for Nordic's nRF70 bare-metal system library.
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
#ifndef __WIFI_NRF70_SCAN_H__
#define __WIFI_NRF70_SCAN_H__
#include <stdint.h>
#include <stdbool.h>

#define WIFI_NRF70_SSID_MAX 32
typedef struct {
	uint8_t Ssid[WIFI_NRF70_SSID_MAX];
	uint8_t SsidLen;
	uint8_t Bssid[6];
	uint8_t Band;
	uint8_t Channel;
	int8_t Rssi;
	uint8_t Security;
} WifiNrf70ScanResult_t;

typedef void (*WifiNrf70ScanResultCb_t)(const WifiNrf70ScanResult_t *pResult,
	bool Complete, void *pContext);

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Requires Nordic nrf70-bm and its OS-agnostic nrf_wifi dependency.
 * Build only when IO_ENABLE_NRF70_BM is defined; it is not a stand-alone
 * host driver or replacement for Nordic firmware initialization.
 *
 * pMac: host-provided 6-byte MAC address.
 * pRegInfo: Nordic nrf70_bm_regulatory_info pointer (must remain valid during init).
 */
bool WifiNrf70ScanInit(uint8_t pMac[6], void *pRegInfo);
bool WifiNrf70ScanStart(uint8_t Bands, uint16_t MaxResults,
	WifiNrf70ScanResultCb_t Callback, void *pContext);
void WifiNrf70ScanClose(void);
bool WifiNrf70ScanBusy(void);

#ifdef __cplusplus
}
#endif
#endif
