/**-------------------------------------------------------------------------
@example	BlePdmDemo.cpp


@brief	PDM BLE demo

This application demo shows PDM over BLE custom service using IOsonata library.
For evaluating power consumption of the UART, the button 1 is used to enable/disable it.

@author	Hoang Nguyen Hoan
@date	Nov. 17, 2021

@license

MIT License

Copyright (c) 2021 I-SYST inc. All rights reserved.

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

#include "app_util_platform.h"
#include "app_scheduler.h"

#include "istddef.h"
#include "convutil.h"
#include "ble_app.h"
#include "ble_intrf.h"
#include "ble_service.h"
#include "bluetooth/blueio_blesrvc.h"
#include "blueio_board.h"
#include "coredev/uart.h"
#include "custom_board.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "coredev/pdm.h"
//#include "audio/audiodev.h"
#include "board.h"

void MicCharSetNotify(BLESRVC *pBleSvc, bool bEnable);
void CfgSrvcCallback(BLESRVC *pBleSvc, uint8_t *pData, int Offset, int Len);

#define DEVICE_NAME                     "BlePdmDemo"                          /**< Name of device. Will be included in the advertising data. */

#define PACKET_SIZE						400

#define MAX_MTU							NRF_SDH_BLE_GATT_MAX_MTU_SIZE

#define MANUFACTURER_NAME               "I-SYST inc."                       /**< Manufacturer. Will be passed to Device Information Service. */

#ifdef NRF52
#define MODEL_NAME                      "IMM-NRF52x"                        /**< Model number. Will be passed to Device Information Service. */
#else
#define MODEL_NAME                      "IMM-NRF51x"                        /**< Model number. Will be passed to Device Information Service. */
#endif

#define MANUFACTURER_ID                 ISYST_BLUETOOTH_ID                  /**< Manufacturer ID, part of System ID. Will be passed to Device Information Service. */
#define ORG_UNIQUE_ID                   ISYST_BLUETOOTH_ID                  /**< Organizational Unique ID, part of System ID. Will be passed to Device Information Service. */

#define APP_ADV_INTERVAL                MSEC_TO_UNITS(64, UNIT_0_625_MS)	/**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */

#if (NRF_SD_BLE_API_VERSION < 6)
#define APP_ADV_TIMEOUT			      	0										/**< The advertising timeout (in units of seconds). */
#else
#define APP_ADV_TIMEOUT					MSEC_TO_UNITS(0, UNIT_10_MS)		/**< The advertising timeout (in units of 10ms seconds). */
#endif

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(8, UNIT_1_25_MS)     /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(40, UNIT_1_25_MS)     /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */

// 00000000-2a76-4901-a32a-db0eea85d0e5

#define BLE_PDM_UUID_BASE				{ 0xe5, 0xd0, 0x85, 0xea, 0x0e, 0xdb, 0x2a, 0xa3, \
										  0x01, 0x49, 0x76, 0x2a, 0x00, 0x00, 0x00, 0x00 }

#define BLE_PDM_UUID_SERVICE		1
#define BLE_PDM_CFG_UUID_CHAR		2
#define BLE_PDM_DATA_UUID_CHAR		3

typedef struct {
	PDM_OPMODE Mode;
	bool bDownsample;
} MicConfig_t;

#define PDM_BUFF_MAXLEN				400

typedef struct __Pdm_Packet {
	uint32_t Cnt;
	int16_t Data[PDM_BUFF_MAXLEN / 2];
} PdmPacket_t;

void UartTxSrvcCallback(BLESRVC *pBlueIOSvc, uint8_t *pData, int Offset, int Len);

static const ble_uuid_t  s_AdvUuids[] = {
	{BLE_PDM_UUID_SERVICE, BLE_UUID_TYPE_VENDOR_BEGIN}
};

static const char s_CfgCharDescString[] = {
	"PDM Config characteristic",
};

static const char s_DataCharDescString[] = {
	"PDM Data characteristic",
};

uint8_t g_ManData[8];

/// Characteristic definitions
BleSrvcChar_t g_PdmChars[] = {
	{
		// Write characteristic
		.Uuid = BLE_PDM_CFG_UUID_CHAR,		// char UUID
		.MaxDataLen = 1,					// char max data length
		.Property = BLESVC_CHAR_PROP_WRITE ,	// char properties define by BLUEIOSVC_CHAR_PROP_...
		.pDesc = s_CfgCharDescString,	// char UTF-8 description string
		.WrCB = CfgSrvcCallback,	// Callback for write char, set to NULL for read char
		.SetNotifCB = NULL,					// Callback on set notification
		.TxCompleteCB = NULL,				// Tx completed callback
		.pDefValue = NULL,					// pointer to char default values
		.ValueLen = 0						// Default value length in bytes
	},
	{
		// Read characteristic
		.Uuid = BLE_PDM_DATA_UUID_CHAR,
		.MaxDataLen = sizeof(PdmPacket_t),
		.Property =
		BLESVC_CHAR_PROP_READ | BLESVC_CHAR_PROP_NOTIFY | BLESVC_CHAR_PROP_VARLEN,
		.pDesc = s_DataCharDescString,		// char UTF-8 description string
		.WrCB = NULL,						// Callback for write char, set to NULL for read char
		.SetNotifCB = MicCharSetNotify,					// Callback on set notification
		.TxCompleteCB = NULL,				// Tx completed callback
		.pDefValue = NULL,					// pointer to char default values
		.ValueLen = 0,						// Default value length in bytes
	},
};

static const int s_BlePdmNbChar = sizeof(g_PdmChars) / sizeof(BleSrvcChar_t);

uint8_t g_LWrBuffer[512];

/// Service definition
const BleSrvcCfg_t s_PdmSrvcCfg = {
	.SecType = BLESRVC_SECTYPE_NONE,		// Secure or Open service/char
	.UuidBase = {BLE_PDM_UUID_BASE,},		// Base UUID
	1,
	.UuidSvc = BLE_PDM_UUID_SERVICE,		// Service UUID
	.NbChar = s_BlePdmNbChar,				// Total number of characteristics for the service
	.pCharArray = g_PdmChars,				// Pointer a an array of characteristic
	.pLongWrBuff = g_LWrBuffer,				// pointer to user long write buffer
	.LongWrBuffSize = sizeof(g_LWrBuffer),	// long write buffer size
};

BleSrvc_t g_BlePdmSrvc;

const BleAppDevInfo_t s_BlePdmDevDesc = {
	MODEL_NAME,       		// Model name
	MANUFACTURER_NAME,		// Manufacturer name
	"123",					// Serial number string
	"0.0",					// Firmware version string
	"0.0",					// Hardware version string
};

const BleAppCfg_t s_BleAppCfg = {
#ifdef IMM_NRF51822
		.ClkCfg = { NRF_CLOCK_LF_SRC_RC, 1, 1, 0},
#else
		.ClkCfg = { NRF_CLOCK_LF_SRC_XTAL, 0, 0, NRF_CLOCK_LF_ACCURACY_20_PPM},
#endif
	.CentLinkCount = 0, 				// Number of central link
	.PeriLinkCount = 1, 				// Number of peripheral link
	.AppMode = BLEAPP_MODE_APPSCHED,	// Use scheduler
	.pDevName = DEVICE_NAME,			// Device name
	.VendorID = ISYST_BLUETOOTH_ID,		// PnP Bluetooth/USB vendor id
	.ProductId = 1,						// PnP Product ID
	.ProductVer = 0,					// Pnp prod version
	.bEnDevInfoService = true,			// Enable device information service (DIS)
	.pDevDesc = &s_BlePdmDevDesc,
	.pAdvManData = g_ManData,			// Manufacture specific data to advertise
	.AdvManDataLen = sizeof(g_ManData),	// Length of manufacture specific data
	.pSrManData = NULL,
	.SrManDataLen = 0,
	.SecType = BLEAPP_SECTYPE_NONE,//BLEAPP_SECTYPE_STATICKEY_MITM,//BLEAPP_SECTYPE_NONE,    // Secure connection type
	.SecExchg = BLEAPP_SECEXCHG_NONE,	// Security key exchange
	.pAdvUuids = NULL,      			// Service uuids to advertise
	.NbAdvUuid = 0, 					// Total number of uuids
	.AdvInterval = APP_ADV_INTERVAL,	// Advertising interval in msec
	.AdvTimeout = APP_ADV_TIMEOUT,		// Advertising timeout in sec
	.AdvSlowInterval = 0,				// Slow advertising interval, if > 0, fallback to
										// slow interval on adv timeout and advertise until connected
	.ConnIntervalMin = MIN_CONN_INTERVAL,
	.ConnIntervalMax = MAX_CONN_INTERVAL,
	.ConnLedPort = BLUEIO_CONNECT_LED_PORT,// Led port nuber
	.ConnLedPin = BLUEIO_CONNECT_LED_PIN,// Led pin number
	.TxPower = 0,						// Tx power
	.SDEvtHandler = NULL,				// RTOS Softdevice handler
	.MaxMtu = MAX_MTU,
};

int BleIntrfEvtCallback(DevIntrf_t *pDev, DEVINTRF_EVT EvtId, uint8_t *pBuffer, int BufferLen);

#define BLEINTRF_FIFOSIZE			BLEINTRF_CFIFO_TOTAL_MEMSIZE(20, sizeof(PdmPacket_t))

alignas(4) static uint8_t s_BleIntrfRxFifo[BLEINTRF_FIFOSIZE];
alignas(4) static uint8_t s_BleIntrfTxFifo[BLEINTRF_FIFOSIZE];


static const BleIntrfCfg_t s_BleInrfCfg = {
	&g_BlePdmSrvc,
	0,
	1,
	sizeof(PdmPacket_t),			// Packet size : use default
	true,
	BLEINTRF_FIFOSIZE,			// Rx Fifo mem size
	s_BleIntrfRxFifo,		// Rx Fifo mem pointer
	BLEINTRF_FIFOSIZE,			// Tx Fifo mem size
	s_BleIntrfTxFifo,		// Tx Fifo mem pointer
	nullptr,//BleIntrfEvtCallback,
};

BleIntrf g_BleIntrf;

static const IOPinCfg_t s_PdmPins[] = {
	{ICS_41352_CLK_PORT, ICS_41352_CLK_PIN, ICS_41352_CLK_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{ICS_41352_DIN_PORT, ICS_41352_DIN_PIN, ICS_41352_DIN_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
};

void PdmHandler(PdmDev_t *pDev, DEVINTRF_EVT Evt);

#define PDM_FIFO_BLKSIZE		400
#define PDM_FIFO_MEMSIZE		CFIFO_TOTAL_MEMSIZE(4, PDM_FIFO_BLKSIZE)

alignas(4) static uint8_t s_PdmFifoMem[PDM_FIFO_MEMSIZE];

static const PdmCfg_t s_PdmCfg = {
	.pPins = s_PdmPins,
	.NbPins = sizeof(s_PdmPins) / sizeof(IOPinCfg_t),
	.Freq = 1032000,
	.SmplMode = PDM_SMPLMODE_FALLING,
	.OpMode = PDM_OPMODE_STEREO,
	.GainLeft = 0,
	.GainRight = 0,
	.bIntEn = true,
	.IntPrio = 6,
	.EvtHandler = PdmHandler,
	.pFifoMem = s_PdmFifoMem,
	.FifoMemSize = PDM_FIFO_MEMSIZE,
	.FifoBlkSize = PDM_FIFO_BLKSIZE,
};

PdmDev_t g_PdmDev;

PdmPacket_t g_PdmPacket = {0,};

MicConfig_t g_MicConfig = {PDM_OPMODE_STEREO,true};;

int g_DelayCnt = 0;
volatile bool g_bUartState = false;
volatile bool g_bEnable = false;

int BleIntrfEvtCallback(DEVINTRF *pDev, DEVINTRF_EVT EvtId, uint8_t *pData, int Len)
{
//	g_InactCntdwn = g_InactTimeOut;

	if (EvtId == DEVINTRF_EVT_RX_DATA)
	{
//		app_sched_event_put(NULL, 0, DisplayUpdateHandler);
	}

	return 0;
}

void CfgSrvcCallback(BLESRVC *pBleSvc, uint8_t *pData, int Offset, int Len)
{

}

void MicCharSetNotify(BLESRVC *pBleSvc, bool bEnable)
{
	g_bEnable = bEnable;
	if (bEnable)
	{
		//nrfx_pdm_start();
		//g_AudioPkt.PktCnt = 0;
		PdmStart(&g_PdmDev);

	}
	else
	{
		//nrfx_pdm_stop();
		PdmStop(&g_PdmDev);
	}
}

void BlePeriphEvtUserHandler(ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
    	case BLE_GAP_EVT_DISCONNECTED:
    		PdmStop(&g_PdmDev);
    		break;
    }
    BleSrvcEvtHandler(&g_BlePdmSrvc, p_ble_evt);
}

void BleAppInitUserServices()
{
    uint32_t       err_code;

    err_code = BleSrvcInit(&g_BlePdmSrvc, &s_PdmSrvcCfg);
    APP_ERROR_CHECK(err_code);
}

void PdmHandler(PdmDev_t *pDev, DEVINTRF_EVT Evt)
{
	if (Evt == DEVINTRF_EVT_RX_DATA)
	{
		static int bidx = 0;
		static int sample_counter = 0;
		static int sample_sum = 0;
		static int sample_sum_L = 0;
		static int sample_sum_R = 0;
		//static int pkcnt = 0;
		//static int j = 0;
		static int16_t *pl = g_PdmPacket.Data;
		static int16_t *pr = &g_PdmPacket.Data[1];

		/*
		int16_t Data1[PDM_BUFF_MAXLEN / 2];
		for (int i=0;i<PDM_BUFF_MAXLEN / 2;i++){
			Data1[i]=i;
		}
		memcpy(g_PdmPacket.Data, Data1, PDM_BUFF_MAXLEN);

		g_BleIntrf.Tx(0, (uint8_t*)&g_PdmPacket, sizeof(PdmPacket_t));

		g_PdmPacket.Cnt++; */


		int16_t *sl = (int16_t*)PdmGetSamples(pDev);
		if (sl)
		{
			//audio_pkt.PktCnt = pkcnt;
			if (g_MicConfig.bDownsample)
			{
				//if (g_MicConfig.Mode != PDM_OPMODE_STEREO)
				if (g_MicConfig.Mode == PDM_OPMODE_STEREO)
				{
					//Downsample STEREO-------------------------------------------------------
					int16_t *sr = sl + 1;
					for (int i = 0; i < (PDM_BUFF_MAXLEN / 2); i += 2)
					{
						sample_sum_L += EndianCvt16(*sl);
						sample_sum_R += EndianCvt16(*sr);
						sample_counter +=1;
						if (sample_counter==3)
						{
							*pl = EndianCvt16(sample_sum_L/3);
							*pr = EndianCvt16(sample_sum_R/3);
							pl += 2;
							pr += 2;
							//j=j+2;
							sample_sum_L = EndianCvt16(*sl);
							sample_sum_R = EndianCvt16(*sr);
							sample_counter = 1;
						}
						sl += 2;
						sr += 2;
					}
				}
				else
				{
					//Downsample MONO-------------------------------------------------------
					for (int i = 0; i < (PDM_BUFF_MAXLEN / 2); i++)
					{
						sample_sum += EndianCvt16(*sl);
						sample_counter +=1;
						if (sample_counter==3)
						{
							*pl = EndianCvt16(sample_sum/3);
							pl++;
							sample_sum = EndianCvt16(*sl);
							sample_counter = 1;
						}
						sl++;
					}
				}

				if(bidx & 1)//== 0)
				{
					g_BleIntrf.Tx(0, (uint8_t*)&g_PdmPacket, sizeof(PdmPacket_t));
					//j = 0;
					g_PdmPacket.Cnt++;
					pl = g_PdmPacket.Data;
					pr = &g_PdmPacket.Data[1];
					//pkcnt++;
				}

			}
			else
			{
				memcpy(g_PdmPacket.Data, sl, PDM_BUFF_MAXLEN);
				g_BleIntrf.Tx(0, (uint8_t*)&g_PdmPacket, sizeof(PdmPacket_t));
				g_PdmPacket.Cnt++;
				//pkcnt++;
			}

			bidx = (bidx + 1) & 1;
		}
	}
}

void HardwareInit()
{
	IOPinConfig(ICS_41352_VDD_PORT, ICS_41352_VDD_PIN, ICS_41352_VDD_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	IOPinSet(ICS_41352_VDD_PORT, ICS_41352_VDD_PIN);

	PdmInit(&g_PdmDev, &s_PdmCfg);
}

void BleAppInitUserData()
{
	// Add passkey pairing
    ble_opt_t opt;
    opt.gap_opt.passkey.p_passkey = (uint8_t*)"123456";
	uint32_t err_code =  sd_ble_opt_set(BLE_GAP_OPT_PASSKEY, &opt);
	APP_ERROR_CHECK(err_code);

}

//
// Print a greeting message on standard output and exit.
//
// On embedded platforms this might require semi-hosting or similar.
//
// For example, for toolchains derived from GNU Tools for Embedded,
// to enable semi-hosting, the following was added to the linker:
//
// --specs=rdimon.specs -Wl,--start-group -lgcc -lc -lm -lrdimon -Wl,--end-group
//
// Adjust it for other toolchains.
//

int main()
{
   HardwareInit();

    BleAppInit((const BLEAPP_CFG *)&s_BleAppCfg, true);

    g_BleIntrf.Init(s_BleInrfCfg);

    BleAppRun();

	return 0;
}
