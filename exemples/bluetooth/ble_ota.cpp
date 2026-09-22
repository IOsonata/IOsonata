/**-------------------------------------------------------------------------
@example	ble_ota.cpp

@brief	Firmware update over Bluetooth LE with the SMP host tools.

The application runs as usual while nRF Connect Device Manager, nRF Connect
for Mobile or any SMP client uploads a signed image into slot 1 through
the SMP service. A test or confirm marks it, a reset request restarts into
the stage 0 boot (exemples/dfu/dfu_boot_main.cpp), which checks the
signature, installs the image into slot 0 and starts it.

The flash layout is the project's linker script, which includes the one the
boot uses, see docs/architecture/dfu.md. Build this application, sign it:

	imgtool sign -k dfu_dev_key.pem --header-size 0x20 --pad-header --align 4 \
		-v 1.0.1 -S <slot 1 size - 32> BleOta.bin BleOta_signed.bin

The last 32 bytes of slot 1 are its trailer, two 16 byte state units; on a
part whose program unit is larger, two of those units.

Slot 1 is written through the MCU memory controller interface, NvmIntrf,
where the port has one: on a SoftDevice or MPSL build its erases and writes
are arbitrated with the radio like any other store. A project for a part
whose radio runs on another core and has no NvmIntrf port (nRF53 and nRF54H
network cores) defines DFU_SLOT1_TGT and writes through the target layer.

and upload BleOta_signed.bin from the phone.

The firmware version the Device Information service reports is the version
of the running image, read from its record.

@author	Hoang Nguyen Hoan
@date	Sep. 21, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

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
#include <string.h>

#include "istddef.h"
#include "idelay.h"
#include "bluetooth/bt_app.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_dfu_smp.h"
#include "app_evt_handler.h"
#include "storage/nvm.h"
#include "storage/nvm_intrf.h"
#include "dfu/dfu_smp.h"
#include "crypto/crypto_softsha256.h"

#include "iopinctrl.h"
#include "board.h"

#define DEVICE_NAME				"IOsonataOTA"

#define MANUFACTURER_NAME		"I-SYST inc."
#define MODEL_NAME				"BleOta"

#define APP_ADV_INTERVAL		64		// msec
#define APP_ADV_TIMEOUT			0		// advertise until connected

#define MIN_CONN_INTERVAL		8		// msec, short for upload speed
#define MAX_CONN_INTERVAL		40		// msec

// Largest request the SMP service reassembles, reported to the host, which
// sizes its upload pieces from it.
#define SMP_RX_SIZE				1024
#define SMP_TX_SIZE				512

// After the reset response is handed to the stack, how long it gets to go out
// before the reset.
#define RESET_DELAY_MS			300

#ifdef MCU_OSC
McuOsc_t g_McuOsc = MCU_OSC;
#endif

static char s_FwVer[24] = "0.0.0";

static const BtAppDevInfo_t s_DevInfo = {
	MODEL_NAME,
	MANUFACTURER_NAME,
	"0001",				// serial number
	s_FwVer,			// firmware version, from the running image
	"1.0",				// hardware version
};

static uint8_t s_LWrBuffer[512];

static const BtAppCfg_t s_BtAppCfg = {
	.Role = BTAPP_ROLE_PERIPHERAL,
	.PeriphDevMax = 0,
	.CentralDevMax = 1,
	.pDevName = DEVICE_NAME,
	.VendorId = ISYST_BLUETOOTH_ID,
	.ProductId = 1,
	.ProductVer = 0,
	.Appearance = 0,
	.pDevInfo = &s_DevInfo,
	.pAdvManData = nullptr,
	.AdvManDataLen = 0,
	.pSrManData = nullptr,
	.SrManDataLen = 0,
	.SecType = BTGAP_SECTYPE_NONE,
	.SecExchg = BTAPP_SECEXCHG_NONE,
	.bCompleteUuidList = false,
	.pAdvUuid = nullptr,
	.AdvInterval = APP_ADV_INTERVAL,
	.AdvTimeout = APP_ADV_TIMEOUT,
	.AdvSlowInterval = 0,
	.ConnIntervalMin = MIN_CONN_INTERVAL,
	.ConnIntervalMax = MAX_CONN_INTERVAL,
	.ConnLedPort = CONNECT_LED_PORT,
	.ConnLedPin = CONNECT_LED_PIN,
	.ConnLedActLevel = CONNECT_LED_LOGIC,
	.TxPower = 0,
	.MaxMtu = 247,		// BT_DFUSMP_CHAR_MAXLEN + 3, fewer notifications
	.pLongWrPoolMem = s_LWrBuffer,
	.LongWrPoolMemSize = sizeof(s_LWrBuffer),
};

#ifndef DFU_SLOT1_TGT
// Slot 1, through the memory controller interface: on a radio build its
// erases and writes are arbitrated with the stack like any other store.
static NvmIntrf s_NvmIntrf;
static Nvm s_Slot1;
#endif
static DfuStore_t s_Slot1Store;

alignas(16) static uint8_t s_ShaMem[CRYPTO_SOFTSHA256_MEMSIZE];

static DfuSmp s_DfuSmp;
static uint8_t s_SmpRx[SMP_RX_SIZE];
static uint8_t s_SmpTx[SMP_TX_SIZE];
static volatile bool s_bResetReq = false;

#ifndef DFU_SLOT1_TGT
static int NvmIntrfEvtCB(DevIntrf_t * const pDev, DEVINTRF_EVT EvtId,
						 uint8_t *pBuffer, int Len)
{
	(void)pDev;
	(void)pBuffer;
	(void)Len;

	s_Slot1.IntrfEvent(EvtId);

	return 0;
}
#endif

// Asked from the SMP server, while its response is not yet out.
static void DfuSmpResetCB(void *pCtx)
{
	(void)pCtx;

	s_bResetReq = true;
}

// Main loop pump: once the reset response has gone to the stack, give it time
// on air, then restart into the boot.
static void OtaIdle(void)
{
	if (s_bResetReq && BtDfuSmpTxBusy() == false)
	{
		msDelay(RESET_DELAY_MS);
		DfuTgtReset();
	}
}

// The running image version, as the host tools print it.
static void OtaFwVersion(void)
{
	DfuLayout_t lay;
	DfuImgInfo_t info;

	if (DfuLayoutGet(&lay) == false || DfuSlot0Parse(lay, &info) != DFU_IMG_OK)
	{
		return;
	}

	const DfuImgVer_t &v = info.Hdr.Ver;
	uint32_t n[4] = { v.Major, v.Minor, v.Rev, v.Build };
	int cnt = v.Build != 0 ? 4 : 3;
	char *p = s_FwVer;

	for (int i = 0; i < cnt; i++)
	{
		char t[10];
		int l = 0;
		do
		{
			t[l++] = (char)('0' + n[i] % 10);
			n[i] /= 10;
		} while (n[i] != 0);
		while (l > 0)
		{
			*p++ = t[--l];
		}
		if (i + 1 < cnt)
		{
			*p++ = '.';
		}
	}
	*p = 0;
}

static bool OtaInit(void)
{
	DfuLayout_t lay;

	if (DfuLayoutGet(&lay) == false)
	{
		return false;
	}

#ifndef DFU_SLOT1_TGT
	NvmCfg_t cfg;
	memset(&cfg, 0, sizeof(cfg));
	NvmMcuCfg(cfg);

	if (s_NvmIntrf.Init(NvmIntrfEvtCB) == false ||
		s_Slot1.Init(cfg, &s_NvmIntrf, lay.Slot1, lay.Slot1Size) == false ||
		DfuStoreNvm(&s_Slot1Store, &s_Slot1) == false)
	{
		return false;
	}
#else
	if (DfuStoreTgt(&s_Slot1Store, lay.Slot1, lay.Slot1Size) == false)
	{
		return false;
	}
#endif

	static DfuMgr s_DfuMgr;
	DfuMgrCfg_t dmcfg = {
		.pStore = &s_Slot1Store,
		.bDirect = false,
		.bManifestOnly = false,
		.bAllowDowngrade = false,
		.pHash = CryptoSoftSha256Create(s_ShaMem, sizeof(s_ShaMem)),
		.pBoot = nullptr,
	};
	if (s_DfuMgr.Init(dmcfg) == false)
	{
		return false;
	}
	DfuSmpCfg_t mcfg = {
		.pMgr = &s_DfuMgr,
		.BufSize = sizeof(s_SmpRx),
		.ResetCB = DfuSmpResetCB,
		.pCtx = nullptr,
	};
	if (s_DfuSmp.Init(mcfg) == false)
	{
		return false;
	}

	BtDfuSmpCfg_t bcfg = {
		.pMgr = &s_DfuSmp,
		.pRxBuf = s_SmpRx,
		.RxBufSize = sizeof(s_SmpRx),
		.pTxBuf = s_SmpTx,
		.TxBufSize = sizeof(s_SmpTx),
		.SecType = BT_GAP_SECTYPE_NONE,
	};
	if (BtDfuSmpInit(bcfg) == false)
	{
		return false;
	}

	return AppEvtHandlerIdleRegister(OtaIdle);
}

void BtAppInitUserServices(void)
{
	if (OtaInit() == false)
	{
		// No update path: keep running, the application itself still works.
		return;
	}
}

void BtAppEvtDisconnected(uint16_t ConnHdl)
{
	(void)ConnHdl;

	BtDfuSmpReset();
}

int main()
{
	OtaFwVersion();

	if (BtAppInit(&s_BtAppCfg) == false)
	{
		while (true)
		{
			__WFE();
		}
	}

	BtAppRun();

	while (true)
	{
		__WFE();
	}
}
