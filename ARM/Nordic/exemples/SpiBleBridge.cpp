/**-------------------------------------------------------------------------
@example	SpiBleBridge.cpp

@brief	SPI BLE streaming demo

This firmware demonstrates SPI streaming over Bluetooth custom service.
Here, the mobile app BleSpiBridge wants to read/write data from/to the flash memory device MX25U1635E connected
to the nRF528xx MCU via a SPI interface.


@author	Duy Thinh Tran
@date	Feb. 09, 2023

@license

Copyright (c) 2023, I-SYST inc., all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : info at i-syst dot com

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------*/

#include "app_util_platform.h"
#include "app_scheduler.h"

#include "istddef.h"
#include "device_intrf.h"
#include "bluetooth/bt_app.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_intrf.h"
#include "bluetooth/blueio_blesrvc.h"
#include "coredev/uart.h"
#include "storage/flash.h"
#include "storage/diskio_flash.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "app_evt_handler.h"

#include "board.h"

//#define NORDIC_NUS_SERVICE

// SPI Command code in the SPI_PKT
#define SPI_WRITE_DATA_FLASH			0x10
#define SPI_READ_DATA_FLASH				0x1F
#define SPI_WRITE_CFG					0x20
#define SPI_READ_CFG					0x2F
#define SPI_WRITE_FLASH_SPEC			0x30
#define SPI_READ_FLASH_SPEC				0x3F
#define SPI_ERASE_FLASH					0x35 // Erase the whole flash memory
#define SPI_TEST_FLASH_SECTOR			0x36
#define SPI_WRITE_GENENRIC				0xA0
#define SPI_READ_GENERIC				0xAF

#define FLASH_CHUNK_TEST_SIZE	512 // Number of bytes to verify, counting from the given Address

#define SPI_PKT_SIZE				sizeof(SPI_PKT) // size of a SPI packet in byte
#define SPI_BLEINTRF_PKTSIZE		(SPI_PKT_SIZE)
#define SPI_BLEINTRF_FIFOSIZE		BTINTRF_CFIFO_TOTAL_MEMSIZE(5, SPI_BLEINTRF_PKTSIZE)//(NbPkt, PktSize)
#define SPI_MAX_DATA_LEN			(SPI_PKT_SIZE)
//#define SPIFIFOSIZE 				CFIFO_MEMSIZE(SPI_MAX_DATA_LEN * 10)

#if defined(BLUEIO_TAG_EVIM)
#define FLASH_CFG(InitCB, WaitCB)	FLASH_MX25U1635E(InitCB, WaitCB)
#endif

#define DEVICE_NAME                     "SpiBleBridge"                            /**< Name of device. Will be included in the advertising data. */
#define PACKET_SIZE						32
#define BLE_MTU_SIZE					499//byte

#define MANUFACTURER_NAME               "I-SYST inc."							/**< Manufacturer. Will be passed to Device Information Service. */
#define MODEL_NAME                      "IMM-NRF52x"                            /**< Model number. Will be passed to Device Information Service. */
#define MANUFACTURER_ID                 ISYST_BLUETOOTH_ID						/**< Manufacturer ID, part of System ID. Will be passed to Device Information Service. */
#define ORG_UNIQUE_ID                   ISYST_BLUETOOTH_ID						/**< Organizational Unique ID, part of System ID. Will be passed to Device Information Service. */

#define APP_ADV_INTERVAL                64 /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */

#if (NRF_SD_BLE_API_VERSION < 6)
#define APP_ADV_TIMEOUT			      	180										/**< The advertising timeout (in units of seconds). */
#else
#define APP_ADV_TIMEOUT					0 /**< The advertising timeout (in units of 10ms seconds). */
#endif

#define MIN_CONN_INTERVAL               8 /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               60 /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */

#ifdef NORDIC_NUS_SERVICE
#define BLE_UART_UUID_BASE			NUS_BASE_UUID

#define BLE_UART_UUID_SERVICE		BLE_UUID_NUS_SERVICE			/**< The UUID of the Nordic UART Service. */
#define BLE_UART_UUID_READ_CHAR		BLE_UUID_NUS_TX_CHARACTERISTIC	/**< The UUID of the TX Characteristic. */
#define BLE_UART_UUID_WRITE_CHAR	BLE_UUID_NUS_RX_CHARACTERISTIC	/**< The UUID of the RX Characteristic. */
#else
#define BLE_SPI_UUID_BASE				BLUEIO_UUID_BASE				// Base UUID of the device
#define BLE_SPI_UUID_SERVICE			BLUEIO_UUID_SPI_SERVICE			// BlueIO SPI service

#define BLE_SPI_UUID_RX_CHAR			BLUEIO_UUID_SPI_RX_CHAR			// SPI Rx characteristic
#define BLE_SPI_UUID_RX_CHAR_PROP		(BT_GATT_CHAR_PROP_READ | \
											BT_GATT_CHAR_PROP_NOTIFY | BT_GATT_CHAR_PROP_VALEN) // Property of Tx characteristic

#define BLE_SPI_UUID_TX_CHAR			BLUEIO_UUID_SPI_TX_CHAR			// SPI Tx characteristic
#define BLE_SPI_UUID_TX_CHAR_PROP		(BT_GATT_CHAR_PROP_WRITE | \
											BT_GATT_CHAR_PROP_WRITE_WORESP | BT_GATT_CHAR_PROP_VALEN) // Property of Tx characteristic

#define BLE_SPI_UUID_CONFIG_CHAR		BLUEIO_UUID_SPI_CONFIG_CHAR 	// SPI configuration characteristic
#define BLE_SPI_UUID_CONFIG_CHAR_PROP	(BT_GATT_CHAR_PROP_WRITE | BT_GATT_CHAR_PROP_WRITE_WORESP | BT_GATT_CHAR_PROP_VALEN | \
										 BT_GATT_CHAR_PROP_READ | BT_GATT_CHAR_PROP_NOTIFY ) // Property of SPI config. char.
#endif

#define BLESRV_SPIM_READ_CHAR_IDX		0
#define BLESRV_SPIM_WRITE_CHAR_IDX		1
#define BLESRV_SPIM_CONFIG_CHAR_IDX		2

/// UART object instance
UART g_Uart;


static const BtUuidArr_t s_AdvUuid = {
	.BaseIdx = 1,
	.Type = BT_UUID_TYPE_16,
	.Count = 1,
	.Uuid16 = {BLE_SPI_UUID_SERVICE,}
};

static const char s_SpiRxCharDescStr[] = {
	"SPI_Master Rx characteristic",
};
static const char s_SpiTxCharDescStr[] = {
	"SPI_Master Tx characteristic",
};

uint8_t g_ManData[8];

#if 0
static const char s_SpiCfgDescStr[] = {
	"SPI_Master Configuration characteristic",
};

alignas(4) static uint8_t s_SpiMasCharData[SPI_PKT_SIZE];
#endif

/* SPI characteristic definitions */
static BtGattChar_t g_SpiMasChars[] = {
	{
		// Read characteristic
		.Uuid = BLE_SPI_UUID_RX_CHAR,
		.MaxDataLen = SPI_BLEINTRF_PKTSIZE,
		.Property = BLE_SPI_UUID_RX_CHAR_PROP,
		.pDesc = s_SpiRxCharDescStr,			// char UTF-8 description string
		.WrCB = NULL,							// Callback for write char, set to NULL for read char
		.SetNotifCB = NULL,						// Callback on set notification
		.TxCompleteCB = NULL,					// Tx completed callback
		.pValue = NULL, //s_SpiMasCharData,				// pointer to char default values
		.ValueLen = 0,							// Default value length in bytes
	},
	{
		// Write characteristic
		.Uuid = BLE_SPI_UUID_TX_CHAR,			// char UUID
		.MaxDataLen = SPI_BLEINTRF_PKTSIZE,		// char max data length
		.Property = BLE_SPI_UUID_TX_CHAR_PROP,	// char properties define by BLUEIOSVC_CHAR_PROP_...
		.pDesc = s_SpiTxCharDescStr,			// char UTF-8 description string
		.WrCB = NULL,//SpiTxSrvcCallback,		// Callback for write char, set to NULL for read char
		.SetNotifCB = NULL,						// Callback on set notification
		.TxCompleteCB = NULL,					// Tx completed callback
		.pValue = NULL,							// pointer to char default values
		.ValueLen = 0							// Default value length in bytes
	},
//	{
//		// SPI Configuration characteristic
//		.Uuid = BLE_SPI_UUID_CONFIG_CHAR,		// char UUID
//		.MaxDataLen = SPI_MAX_DATA_LEN,			// char max data length
//		.Property = BLE_SPI_UUID_CONFIG_CHAR_PROP,// char properties define by BLUEIOSVC_CHAR_PROP_...
//		.pDesc = s_SpiCfgDescStr,			// char UTF-8 description string
//		.WrCB = SpiCfgSrvcCallback,			// Callback for write char, set to NULL for read char
//		.SetNotifCB = NULL,						// Callback on set notification
//		.TxCompleteCB = NULL,					// Tx completed callback
//		.pValue = NULL,						// pointer to char default values
//		.ValueLen = 0							// Default value length in bytes
//	},
};

static const int s_BleSpiNbChar = sizeof(g_SpiMasChars) / sizeof(BtGattChar_t);
//uint8_t g_LWrBuffer[512];

/* BlueIO SPI service definition */
BtGattSrvcCfg_t s_SpiSrvcCfg = {
	.SecType = BT_GAP_SECTYPE_NONE,		// Secure or Open service/char
	.bCustom = true,						// True - for custom service Base UUID, false - Bluetooth SIG standard
	.UuidBase = BLE_SPI_UUID_BASE,			// Base UUID
	.UuidSrvc = BLE_SPI_UUID_SERVICE,		// Service UUID
	.NbChar = s_BleSpiNbChar,				// Total number of characteristics for the service
	.pCharArray = g_SpiMasChars,			// Pointer a an array of characteristic
	.pLongWrBuff = NULL,					// Pointer to user long write buffer
	.LongWrBuffSize = 0,					// long write buffer size
};

BtGattSrvc_t g_SpiBleSrvc;

alignas(4) static uint8_t s_BleSpiIntrfRxFifo[SPI_BLEINTRF_FIFOSIZE];
alignas(4) static uint8_t s_BleSpiIntrfTxFifo[SPI_BLEINTRF_FIFOSIZE];

BtIntrfCfg_t s_SpiBleIntrfCfg = {
	.pSrvc = &g_SpiBleSrvc,
	.RxCharIdx = BLESRV_SPIM_WRITE_CHAR_IDX,		// Mobile app --> BleDev
	.TxCharIdx = BLESRV_SPIM_READ_CHAR_IDX,			// BleDev --> Mobile app
	.PacketSize = SPI_BLEINTRF_PKTSIZE,				// Size equal to 1 SPI packet
	.bBlocking = true,
	.RxFifoMemSize = SPI_BLEINTRF_FIFOSIZE,			// Rx Fifo mem size
	.pRxFifoMem = s_BleSpiIntrfRxFifo,				// Rx Fifo mem pointer
	.TxFifoMemSize = SPI_BLEINTRF_FIFOSIZE,			// Tx Fifo mem size
	.pTxFifoMem = s_BleSpiIntrfTxFifo,				// Tx Fifo mem pointer
	.EvtCB = BleSpiIntrfEvtCb						// Event callback fn
};

BtIntrf g_BleSpiIntrf;

/* ***************************************************************************************
 * ******************** SPI operation mode config section *******************************
 ****************************************************************************************/

IOPinCfg_t s_SpiMasterPins[] = SPI_MASTER_PIN_MAP;
static int s_NbSpiMasterPins = sizeof(s_SpiMasterPins) / sizeof(IOPinCfg_t);

static SPICfg_t s_SpiMasCfg = {
	.DevNo = SPI_MASTER_DEVNO,			// SPI engine index
	.Phy = SPIPHY_NORMAL,				// SPI physical interface type (standard, 3 wire, quad,..)
	.Mode = SPIMODE_MASTER,				// Master/Slave mode
	.pIOPinMap = s_SpiMasterPins,		// Define I/O pins used by SPI (including CS array)
	.NbIOPins = s_NbSpiMasterPins,		// Total number of I/O pins
	.Rate = 8000000,					// Speed in Hz
	.DataSize = 8, 						// Data Size 4-16 bits
	.MaxRetry = 5,						// Max number of retry
	.BitOrder = SPIDATABIT_MSB,			// Data bit ordering
	.DataPhase = SPIDATAPHASE_FIRST_CLK,// Data Out Phase.
	.ClkPol = SPICLKPOL_HIGH,			// Clock Out Polarity.
	.ChipSel = SPICSEL_AUTO,			// Chip select mode
	.bDmaEn = true,						// true - DMA transfer support
	.bIntEn = false,					// Interrupt enable
	.IntPrio = APP_IRQ_PRIORITY_LOW,	// Interrupt priority
	.EvtCB = NULL,//nRFSpiEvtHandler,	// Event callback
};

SPI g_SpiMaster;
alignas(4) uint8_t g_SpiRxBuff[SPI_PKT_SIZE];
volatile int g_SpiRxBuffLen = 0;
alignas(4) uint8_t s_SpiRxFifo[SPI_BLEINTRF_FIFOSIZE];
alignas(4) uint8_t s_SpiTxFifo[SPI_BLEINTRF_FIFOSIZE];
HCFIFO g_Ble2SpiFifo;
HCFIFO g_Spi2BleFifo;

/* ***************************************************************************************
 * ******************** Flash memory config section *******************************
 ****************************************************************************************/
static const FlashCfg_t s_FlashCfg = FLASH_CFG(MX25U1635E_init, NULL);//FLASH_CFG(MX25U1635E_init, NULL)
FlashDiskIO g_Flash;
alignas(4) static uint8_t s_FlashCache[4096];
DiskIOCache_t g_FlashCache = {
		-1, 0xFFFFFFFF, s_FlashCache
};

static SPI_PKT g_SpiPkt = {0,};
static SPI_PKT g_SpiReadPkt;

#if 0
bool FlashWriteDelayCallback(int DevNo, DeviceIntrf *pInterf)
{
	msDelay(3);
	return true;
}
#endif

bool MX25U1635E_init(int DevNo, DevIntrf_t* pInterface)
{
	if (pInterface == NULL)
		return false;

	int cnt = 0;
	uint32_t r = 0;

	// Find the Flash MX25U1635E
	uint32_t cmd = FLASH_CMD_READID;
	//cnt = pInterface->Read(DevNo, (uint8_t*)&cmd, 1, (uint8_t*)&r, 2);
	cnt = DeviceIntrfRead(pInterface, DevNo, (uint8_t*)&cmd, 1, (uint8_t*)&r, 2);
	if ( r != 0x25C2 )
	{
		DEBUG_PRINTF("Wrong FLASH_CMD_READID response\r\n");
		return false;
	}

	DEBUG_PRINTF("Flash MX25U1635E found!\r\n");

	// Enable flash write
	cmd = FLASH_CMD_EN4B;
	//cnt = pInterface->Tx(DevNo, (uint8_t*)&cmd, 1);
	cnt = DeviceIntrfTx(pInterface, DevNo, (uint8_t*)&cmd, 1);

	return true;
}

const BtAppDevInfo_t s_UartBleDevDesc {
	MODEL_NAME,           	// Model name
	MANUFACTURER_NAME,      // Manufacturer name
	"",                     // Serial number string
	"0.0",                  // Firmware version string
	"0.0",                  // Hardware version string
};

const BtAppCfg_t s_BleAppCfg = {
	.Role = BTAPP_ROLE_PERIPHERAL,
	.CentLinkCount = 0, 				// Number of central link
	.PeriLinkCount = 1, 				// Number of peripheral link
	.pDevName = DEVICE_NAME,			// Device name
	.VendorId = ISYST_BLUETOOTH_ID,		// PnP Bluetooth/USB vendor id
	.ProductId = 1,						// PnP Product ID
	.ProductVer = 0,					// Pnp prod version
	.pDevInfo = &s_UartBleDevDesc,
	.bExtAdv = false,
	.pAdvManData = g_ManData,			// Manufacture specific data to advertise
	.AdvManDataLen = sizeof(g_ManData),	// Length of manufacture specific data
	.pSrManData = NULL,
	.SrManDataLen = 0,
	.SecType = BTGAP_SECTYPE_NONE,    // Secure connection type
	.SecExchg = BTAPP_SECEXCHG_NONE,	// Security key exchange
	.pAdvUuid = NULL,      			// Service uuids to advertise
	//.NbAdvUuid = 0, 					// Total number of uuids
	.AdvInterval = APP_ADV_INTERVAL,	// Advertising interval in msec
	.AdvTimeout = APP_ADV_TIMEOUT,		// Advertising timeout in sec
	.AdvSlowInterval = 0,				// Slow advertising interval, if > 0, fallback to
										// slow interval on adv timeout and advertise until connected
	.ConnIntervalMin = MIN_CONN_INTERVAL,
	.ConnIntervalMax = MAX_CONN_INTERVAL,
	.ConnLedPort = BLUEIO_CONNECT_LED_PORT,// Led port nuber
	.ConnLedPin = BLUEIO_CONNECT_LED_PIN,// Led pin number
	.TxPower = 4,						// Tx power
	.SDEvtHandler = NULL,				// RTOS Softdevice handler
	.MaxMtu = BLE_MTU_SIZE,
};

//BtIntrf g_BtIntrf;


/* ***************************************************************************************
 * ******************** UART operation mode config section *******************************
 ****************************************************************************************/
#define UART_MAX_DATA_LEN  		32
#define UARTFIFOSIZE			CFIFO_MEMSIZE(UART_MAX_DATA_LEN*10)

alignas(4) static uint8_t s_UartRxFifo[UARTFIFOSIZE];
alignas(4) static uint8_t s_UartTxFifo[UARTFIFOSIZE];

static const IOPinCfg_t s_UartPins[] = UART_PIN_MAP;
static int s_NbUartPins = sizeof(s_UartPins) / sizeof(IOPinCfg_t);

/// UART configuration
const UARTCfg_t g_UartCfg = {
	.DevNo = 0,							// Device number zero based
	.pIOPinMap = s_UartPins,				// UART assigned pins
	.NbIOPins = s_NbUartPins,	// Total number of UART pins used
	.Rate = 115200,						// Baudrate
	.DataBits = 8,						// Data bits
	.Parity = UART_PARITY_NONE,			// Parity
	.StopBits = 1,						// Stop bit
	.FlowControl = UART_FLWCTRL_NONE,	// Flow control
	.bIntMode = true,					// Interrupt mode
	.IntPrio = APP_IRQ_PRIORITY_LOW,	// Interrupt priority
	.EvtCallback = nRFUartEvthandler,	// UART event handler
	.bFifoBlocking = true,				// Blocking FIFO
	.RxMemSize = UARTFIFOSIZE,
	.pRxMem = s_UartRxFifo,
	.TxMemSize = UARTFIFOSIZE,
	.pTxMem = s_UartTxFifo,
};

#if 0
int g_DelayCnt = 0;

int BleIntrfEvtCallback(DevIntrf_t *pDev, DEVINTRF_EVT EvtId, uint8_t *pBuffer, int BufferLen)
{
	int cnt = 0;

	if (EvtId == DEVINTRF_EVT_RX_DATA)
	{
		uint8_t buff[128];

		int l = g_BtIntrf.Rx(0, buff, 128);
		if (l > 0)
		{
			g_Uart.Tx(buff, l);
		}
		cnt += l;
	}

	return cnt;
}
#endif

void BtAppPeriphEvtHandler(uint32_t Evt, void *pCtx)
{
	BtGattEvtHandler(Evt, pCtx);
}

void BtAppInitUserServices()
{
    SpiBleInit();
}

void BtAppInitUserData()
{

}

//void UartRxChedHandler(void * p_event_data, uint16_t event_size)
void UartRxChedHandler(uint32_t Evt, void *pCtx)
{
#if 0
	static uint8_t buff[PACKET_SIZE];
	static int bufflen = 0;
	bool flush = false;

	int l = g_Uart.Rx(&buff[bufflen], PACKET_SIZE - bufflen);
	if (l > 0)
	{
		bufflen += l;
		if (bufflen >= PACKET_SIZE)
		{
			flush = true;
		}
	}
	else
	{
		if (bufflen > 0)
		{
			flush = true;
		}
	}

	if (flush)
	{
		g_BtIntrf.Tx(0, buff, bufflen);
		bufflen = 0;
//		app_sched_event_put(NULL, 0, UartRxChedHandler);
		AppEvtHandlerQue(0, 0, UartRxChedHandler);
	}
#endif
}

int nRFUartEvthandler(UARTDev_t *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen)
{
	int cnt = 0;
	uint8_t buff[20];

	switch (EvtId)
	{
		case UART_EVT_RXTIMEOUT:
		case UART_EVT_RXDATA:
			//app_sched_event_put(NULL, 0, UartRxChedHandler);
			AppEvtHandlerQue(0, 0, UartRxChedHandler);
			break;
		case UART_EVT_TXREADY:
			break;
		case UART_EVT_LINESTATE:
			break;
	}

	return cnt;
}

void HardwareInit()
{
	g_Uart.Init(g_UartCfg);
	DEBUG_PRINTF("Spi-Ble Bridge Demo\r\n");
}

bool SpiBleInit()
{
	bool res = BtGattSrvcAdd(&g_SpiBleSrvc, &s_SpiSrvcCfg);
	if (res == true)
	{
		res = g_SpiMaster.Init(s_SpiMasCfg);
		if (res == false)
		{
			DEBUG_PRINTF("SPI interface init FAILED\r\n");
			return false;
		}

		res = g_BleSpiIntrf.Init(s_SpiBleIntrfCfg);
		if (res == false)
		{
			DEBUG_PRINTF("BleSpiInterface init FAILED\r\n");
			return false;
		}

		res = g_Flash.Init(s_FlashCfg, &g_SpiMaster, &g_FlashCache, 1);
		if (res == false)
		{
			DEBUG_PRINTF("Flash memory init FAILED\r\n");
			return false;
		}
	}

	//g_Ble2SpiFifo = CFifoInit(s_SpiTxFifo, SPIFIFOSIZE, 1, true);

	DEBUG_PRINTF("SpiBleInit() SUCCESS\r\n");
	return res;
}


//
// Print a greeting message on standard output and exit.
//
// On embedded platforms this might require semi-hosting or similar.
//
// For example, for toolchains derived from GNU Tools for Embedded,
// to enable semi-hosting, the following was added to the linker:
//
// --specs=rdimon.specs -Wl,--start-group -lgcc -lc -lc -lm -lrdimon -Wl,--end-group
//
// Adjust it for other toolchains.
//

int main()
{
    HardwareInit();

    BtAppInit(&s_BleAppCfg);

    BtAppRun();

	return 0;
}


/** SPI_Master_Tx event
 *
 */
int BleSpiIntrfEvtCb(DevIntrf_t *pDev, DEVINTRF_EVT EvtId, uint8_t *pBuffer, int BufferLen)
{
	ToggleLed();
	int cnt = 0;
	int len = 0;

	// Data flow: Mobile app --> SPI_Write char --> target device
	if (EvtId == DEVINTRF_EVT_RX_DATA)
	{
		int l = g_BleSpiIntrf.Rx(0, (uint8_t*)&g_SpiPkt, SPI_BLEINTRF_PKTSIZE);
		if (l > 0)
		{
			ProcSpiPkt(&g_SpiPkt);
		}
		cnt += l;
#if 0
		// Get available FifoBuffer's capacity for writing data to
		p = CFifoPutMultiple(g_Ble2SpiFifo, &len);
		if (p != NULL)
		{
			// Write BleIntrf's internal buffer data to FifoBuffer
			cnt = g_BleSpiIntrf.Rx(0, p, len);
		}
		app_sched_event_put(NULL, 0, SpiTxSchedHandler);
		//DEBUG_PRINTF("\r\n");
#endif
	}

	ToggleLed();
	return cnt;
}

/** Process the packet received from SPI_Write char
 * @param pkt
 */
void ProcSpiPkt(SPI_PKT *pkt)
{
	switch (pkt->Cmd)
	{
	case SPI_WRITE_DATA_FLASH:
	{
		uint32_t l = g_Flash.Write(pkt->Addr, pkt->Data, pkt->DataLen);
		if (l < pkt->DataLen)
			DEBUG_PRINTF("Flash write miss %d byte data\n", pkt->DataLen - l);
	}
		break;
	case SPI_READ_DATA_FLASH:
		ReadFlash(pkt);
		break;

	case SPI_WRITE_CFG:
		// TODO: Update new SPI config
		break;
	case SPI_READ_CFG:
		ReadSpiCfg();
		break;

	case SPI_WRITE_FLASH_SPEC:
		// TODO: Update new Flash Dev config
		break;
	case SPI_READ_FLASH_SPEC:
		ReadFlashDevCfg();
		break;
	case SPI_ERASE_FLASH:
		EraseWholeFlash();
		break;
	case SPI_TEST_FLASH_SECTOR:
		TestFlashSector();
		break;

	// TODO: Generic SPI interface
	case SPI_WRITE_GENENRIC:
		break;
	case SPI_READ_GENERIC:
		break;
	default:
		DEBUG_PRINTF("Wrong SPI command code\r\n");
		break;
	}

}

/** Read data From flash and send it to Ble */
bool ReadFlash(SPI_PKT *pkt)
{
	uint32_t l = 0;
	l = g_Flash.Read(pkt->Addr, pkt->Data, pkt->DataLen);
	pkt->DataLen = l;
	if (l <= 0)
	{
		DEBUG_PRINTF("Flash returns 0-byte read\r\n");
		return false;
	}
	else
	{
		DEBUG_PRINTF("Send flash data to Ble\r\n");
		g_BleSpiIntrf.Tx(0, (uint8_t*)&pkt, SPI_PKT_SIZE);
	}
	return true;
}

/** Parse current Flash device specification
 * and send it to Ble
 */
bool ReadFlashDevCfg()
{
	g_SpiPkt.Cmd = SPI_READ_FLASH_SPEC;
	g_SpiPkt.Addr = 0xFFFFFFFFu;
	g_SpiPkt.DataLen = sizeof(s_FlashCfg);
	memcpy(g_SpiPkt.Data, (uint8_t*)&s_FlashCfg, sizeof(s_FlashCfg));
	if (g_BleSpiIntrf.Tx(0, (uint8_t*)&g_SpiPkt, SPI_PKT_SIZE) == SPI_PKT_SIZE)
	{
		DEBUG_PRINTF("Flash Device Spec sent SUCCESS\n");
		return true;
	}
	else
	{
		DEBUG_PRINTF("Sending not enough Flash Device Spec data \n");
		return false;
	}
}

/** Parse current SPI config
 * and send it to Ble
 * @return
 */
bool ReadSpiCfg()
{
	g_SpiPkt.Cmd = SPI_READ_CFG;
	g_SpiPkt.Addr = 0xFFFFFFFFu;
	g_SpiPkt.DataLen = sizeof(s_SpiMasCfg);
	memcpy(g_SpiPkt.Data, (uint8_t*)&s_SpiMasCfg, sizeof(s_SpiMasCfg));
	if (g_BleSpiIntrf.Tx(0, (uint8_t*)&g_SpiPkt, SPI_PKT_SIZE) == SPI_PKT_SIZE)
	{
		DEBUG_PRINTF("Spi Cfg sent SUCCESS\n");
		return true;
	}
	else
	{
		DEBUG_PRINTF("Sending not enough Spi Cfg data \n");
		return false;
	}
}

/** Test a desired flash sector
 * by writing data to it and readback for integrity verification
 * @param SectNo	: Sector index
 * @return
 * 		0xAA	: SUCCESS
 * 		0x00	: FAIL
 */
bool TestFlashSector()
{
	uint32_t Addr = g_SpiPkt.Addr;
	DEBUG_PRINTF("Test flash sector %d ...\n", Addr);
	g_SpiPkt.DataLen = 1;
	memset(g_SpiPkt.Data, 0xFF, MAX_FLASH_DATA_LEN);
	if (Addr > 31)
	{
		DEBUG_PRINTF("Wrong sector index\r\n");
		g_SpiPkt.Data[0] = 0x00;
		g_BleSpiIntrf.Tx(0, (uint8_t*)&g_SpiPkt, SPI_PKT_SIZE);
		return false;
	}

	uint8_t buff[s_FlashCfg.SectSize];
	uint8_t d[s_FlashCfg.SectSize];
	bool ret;

	DEBUG_PRINTF("Prepare data for testing\r\n");
	for(int i = 0; i < 256; i++)
	{
		d[i] = i;
		d[FLASH_CHUNK_TEST_SIZE - i - 1] = i;
	}

	DEBUG_PRINTF("Write data to sector %d...", Addr);
	if (g_Flash.SectWrite(Addr, d))
		DEBUG_PRINTF("Done.\n");

	DEBUG_PRINTF("Read back data from sector %d...", Addr);
	if (g_Flash.SectRead(Addr, buff))
		DEBUG_PRINTF("Done\n");

	if (memcmp(buff, d, sizeof(d)) != 0)
	{
		DEBUG_PRINTF("Sector %d FAILED\n", Addr);
		g_SpiPkt.Data[0] = 0x00;
		ret = false;
	}
	else
	{
		DEBUG_PRINTF("Sector %d SUCCESS\n", Addr);
		g_SpiPkt.Data[0] = 0xAA;
		ret = true;
	}

	g_BleSpiIntrf.Tx(0, (uint8_t*)&g_SpiPkt, SPI_PKT_SIZE);
	return ret;
}

/** Erase the whole flash memory */
bool EraseWholeFlash()
{
	DEBUG_PRINTF("Erase the whole flash memory. Wait for about 10 seconds...");
	g_Flash.Erase();

	g_SpiPkt.Cmd = SPI_ERASE_FLASH;
	g_SpiPkt.Addr = 0xFFFFFFFFu;
	g_SpiPkt.DataLen = 1;
	memset(g_SpiPkt.Data, 0xFF, MAX_FLASH_DATA_LEN);
	g_SpiPkt.Data[0] = 0xAA;
	g_BleSpiIntrf.Tx(0, (uint8_t*)&g_SpiPkt, SPI_PKT_SIZE);
	DEBUG_PRINTF("Done\r\n");
	return true;
}


void ToggleLed()
{
#if defined(BLYST840)
	IOPinToggle(LED_RED_PORT, LED_RED_PIN);
#elif defined(BLUEIO832)
#if HW_REV >= 2
	IOPinToggle(RGB_GREEN_PORT, RGB_GREEN_PIN);
#else
	IOPinToggle(LED1_PORT, LED1_PIN);
#endif
#elif defined(BLYST_NANO)
	IOPinToggle(s_Leds[1].PortNo, s_Leds[1].PinNo);
#elif defined(BLUEIO_TAG_EVIM)
	IOPinToggle(RGB_GREEN_PORT, RGB_GREEN_PIN);
#endif
}
