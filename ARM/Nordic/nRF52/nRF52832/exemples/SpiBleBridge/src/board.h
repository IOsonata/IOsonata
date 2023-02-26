/**-------------------------------------------------------------------------
@file	board.h

@brief	Board specific definitions

This file contains all I/O definitions for a specific board for the
application firmware.  This files should be located in each project and
modified to suit the need for the application use case.
Typically, the hardware used in this example is BlueIO_TAG_EVIM, i.e., I-SYST CS-BLYST-07.
This module can be found here:
https://www.i-syst.com/products/blyst-nano


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

#ifndef __BOARD_H__
#define __BOARD_H__

#include "blueio_board.h"
#include "coredev/iopincfg.h"

/** Hardware selection */
//#define NORDIC_DK
#define BLUEIO_TAG_EVIM

/** Print debug data via UART **/
#define UART_DEBUG_ENABLE

#ifdef UART_DEBUG_ENABLE
#define DEBUG_PRINTF(...)		g_Uart.printf(__VA_ARGS__)
#else
#define DEBUG_PRINTF(...)
#endif



#if defined(NORDIC_DK)
// Nordic DK PCA10040 board

#define UART_RX_PORT		0
#define UART_RX_PIN			8
#define UART_RX_PINOP		1	//
#define UART_TX_PORT		0
#define UART_TX_PIN			6//7
#define UART_TX_PINOP		0
#define UART_CTS_PORT		0
#define UART_CTS_PIN		7//12
#define UART_CTS_PINOP		0
#define UART_RTS_PORT		0
#define UART_RTS_PIN		5//11
#define UART_RTS_PINOP		0

#define BUTTON1_PORT		0
#define BUTTON1_PIN			13
#define BUTTON2_PORT		0
#define BUTTON2_PIN			14

#elif defined(BLUEIO_TAG_EVIM)
#define LED1_PORT		0
#define LED1_PIN		30
#define LED1_PINOP 		0

// RGB led
#define RGB_RED_PORT		0
#define RGB_RED_PIN			18
#define RGB_RED_PINOP		0

#define RGB_BLUE_PORT		0
#define RGB_BLUE_PIN		19
#define RGB_BLUE_PINOP		0

#define RGB_GREEN_PORT		0
#define RGB_GREEN_PIN		20
#define RGB_GREEN_PINOP		0

//#define LED_PIN_MAP 	{\
//	{LED1_PORT, LED1_PIN, LED1_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
//	{RGB_GREEN_PORT, RGB_GREEN_PIN, RGB_GREEN_PINOP, IOPINDIR_OUTPUT, IOPINRES_PULLDOWN, IOPINTYPE_NORMAL},\
//	{RGB_BLUE_PORT,  RGB_BLUE_PIN, RGB_BLUE_PINOP, IOPINDIR_OUTPUT, IOPINRES_PULLDOWN, IOPINTYPE_NORMAL},\
//	{RGB_RED_PORT, RGB_RED_PIN, RGB_RED_PINOP, IOPINDIR_OUTPUT, IOPINRES_PULLDOWN, IOPINTYPE_NORMAL},\
//}
#define RGB_LED_PIN_MAP 	{\
	{RGB_GREEN_PORT, RGB_GREEN_PIN, RGB_GREEN_PINOP, IOPINDIR_OUTPUT, IOPINRES_PULLDOWN, IOPINTYPE_NORMAL},\
	{RGB_BLUE_PORT,  RGB_BLUE_PIN, RGB_BLUE_PINOP, IOPINDIR_OUTPUT, IOPINRES_PULLDOWN, IOPINTYPE_NORMAL},\
	{RGB_RED_PORT, RGB_RED_PIN, RGB_RED_PINOP, IOPINDIR_OUTPUT, IOPINRES_PULLDOWN, IOPINTYPE_NORMAL},\
}

// Buttons
#define BUTTON1_PORT		0
#define BUTTON1_PIN			2

#define BUTTON2_PORT		0
#define BUTTON2_PIN			13

#define BUTTON_PIN_MAP  	{\
	{BUTTON1_PORT, BUTTON1_PIN, 0, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},\
	{BUTTON2_PORT, BUTTON2_PIN, 0, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},\
}

// UART
#define UART_RX_PORT		0
#define UART_RX_PIN			8
#define UART_RX_PINOP		0

#define UART_TX_PORT		0
#define UART_TX_PIN			7
#define UART_TX_PINOP		0

#define UART_CTS_PORT		0
#define UART_CTS_PIN		12
#define UART_CTS_PINOP		0

#define UART_RTS_PORT		0
#define UART_RTS_PIN		11
#define UART_RTS_PINOP		0

/// UART pins definitions
#define UART_PIN_MAP  		{\
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},\
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},\
	{UART_CTS_PORT, UART_CTS_PIN, UART_CTS_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},\
	{UART_RTS_PORT, UART_RTS_PIN, UART_RTS_PINOP, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},\
}

// SPI - Flash MX25U1635E and Sensor ICM-20948
#define SPI_MASTER_DEVNO	1

#define SPI_MISO_PORT		0
#define SPI_MISO_PIN		15
#define SPI_MISO_PINOP		1

#define SPI_MOSI_PORT		0
#define SPI_MOSI_PIN		16
#define SPI_MOSI_PINOP		1

#define SPI_SCK_PORT		0
#define SPI_SCK_PIN		17
#define SPI_SCK_PINOP		1

#define SPI_CS_FLASH_PORT	0
#define SPI_CS_FLASH_PIN	26
#define SPI_CS_FLASH_PINOP	1

#define SPI_CS_ICM20948_PORT	0
#define SPI_CS_ICM20948_PIN		5
#define SPI_CS_ICM20948_PINOP	1

#define ICM20948_IRQ_PORT	0
#define ICM20948_IRQ_PIN	6
#define ICM20948_IRQ_PORT	0

#define SPI_MASTER_PIN_MAP {\
	{SPI_SCK_PORT, SPI_SCK_PIN, SPI_SCK_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
	{SPI_MISO_PORT, SPI_MISO_PIN, SPI_MISO_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},\
	{SPI_MOSI_PORT, SPI_MOSI_PIN, SPI_MOSI_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
	{SPI_CS_FLASH_PORT, SPI_CS_FLASH_PIN, SPI_CS_FLASH_PINOP,  IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},\
	{SPI_CS_ICM20948_PORT, SPI_CS_ICM20948_PIN, SPI_CS_ICM20948_PINOP, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},\
}

// I2C - Conn P7, BME680, EEPROM
#define I2C0_MASTER_SCL_PORT		0
#define I2C0_MASTER_SCL_PIN 		29
#define I2C0_MASTER_SCL_PINOP		IOPINTYPE_OPENDRAIN

#define I2C0_MASTER_SDA_PORT 		0
#define I2C0_MASTER_SDA_PIN			28
#define I2C0_MASTER_SDA_PINOP		IOPINTYPE_OPENDRAIN

#define I2C_MASTER_DEVNO	0

#define I2C_MASTER_PIN_MAP 	{\
	{I2C0_MASTER_SCL_PORT, I2C0_MASTER_SCL_PIN, I2C0_MASTER_SCL_PINOP, \
		IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
	{I2C0_MASTER_SDA_PORT, I2C0_MASTER_SDA_PIN, I2C0_MASTER_SDA_PINOP, \
		IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
}

// ADC
#define AIN1_PORT	0
#define AIN1_PIN	3

#define AIN2_PORT	0
#define AIN2_PIN	4

#define AIN7_PORT	0
#define AIN7_PIN	31

// NFC
#define NFC1_PORT	0
#define NFC1_PIN	9
#define NFC1_PINOP	1

#define NFC2_PORT	0
#define NFC2_PIN	10
#define NFC2_PINOP	1

// Buzzer
#define BUZZER_PORT		0
#define BUZZER_PIN		14
#define BUZZER_PINOP	1

// PDM
#define PDM_SELECT_PORT		0
#define PDM_SELECT_PIN		22
#define PDM_SELECT_PINOP	1

#define PDM_CLK_PORT	0
#define PDM_CLK_PIN		24
#define PDM_CLK_PINOP	1

#define PDM_DIN_PORT	0
#define PDM_DIN_PIN		25
#define PDM_DIN_PINOP	1

// FDS - flash data storage
#define FDS_DATA_FILE     			0xA010

#define FDS_MODECTRL_DATA_FILE  	FDS_DATA_FILE
#define FDS_MODECTRL_REC_KEY 		0x7010

#define FDS_UART_DATA_FILE			FDS_DATA_FILE
#define FDS_UART_REC_KEY  			0x7011

#define FDS_I2C_DATA_FILE 			FDS_DATA_FILE
#define FDS_I2C_REC_KEY	  			0x7012

#define FDS_SPI_DATA_FILE 			FDS_DATA_FILE
#define FDS_SPI_REC_KEY				0x7013

#define FDS_SIGCAP_DATA_FILE		FDS_DATA_FILE
#define FDS_SIGCAP_REC_KEY			0x7014

#endif

typedef enum {
	RTC0_LFCLK = 0,
	RTC1_LFCLK,
	RTC2_LFCLK,
	TIMER0_HFCLK,
	TIMER1_HFCLK,
	TIMER2_HFCLK,
	TIMER3_HFCLK,
	TIMER4_HFCLK
} TIMER_SOURCES;


#define MAX_FLASH_DATA_LEN		233 // Max data bytes to write to flash per transaction
									// must be smaller than SPI_PACKET_SIZE < BLE_MTU_SIZE
#pragma pack(push,1)
typedef struct __SpiPkt__{
	uint8_t Cmd;					// Command or Response code
	uint32_t Addr;					// Memory address of the SPI slave device
	uint16_t DataLen;				// Number of bytes in the Data field
	uint8_t Data[MAX_FLASH_DATA_LEN];// Raw data buffer
}SPI_PKT;
#pragma pack(pop)


int nRFUartEvthandler(UARTDev_t *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen);

// SPI function declaration
int BleSpiIntrfEvtCb(DevIntrf_t *pDev, DEVINTRF_EVT EvtId, uint8_t *pBuffer, int BufferLen);

void SpiCfgSrvcCallback(BtGattChar_t *pChar, uint8_t *pData, int Offset, int Len);

void SpiTxSrvcCb(BtGattChar_t *pChar, uint8_t *pData, int Offset, int Len);
void SpiCfgSrvcCn(BtGattChar_t *pChar, uint8_t *pData, int Offset, int Len);
int  nRFSpiEvtHandler(DevIntrf_t * const pDev, DEVINTRF_EVT EvtId, uint8_t *pBuffer, int BufferLen);
void SpiTxSchedHandler(void * p_event_data, uint16_t event_size);

//void SpiSlaveTxSrvcCallback(BtGattChar_t *pChar, uint8_t *pData, int Offset, int Len);
//int  nRFSpiSlaveEvtHandler(DevIntrf_t * const pDev, DEVINTRF_EVT EvtId, uint8_t *pBuffer, int BufferLen);
//void SpiSlaveCfgSrvcCallback(BtGattChar_t *pChar, uint8_t *pData, int Offset, int Len);

void BleSpiSrvcSetValue();
void UpdateRecordSpiSrvc();
void RetrieveFdsSpiCfg();
void CalcFdsSpiChecksum();
void ShowSpiConfig();
bool SpiBleInit();

void ProcSpiPkt(SPI_PKT *pkt);
void SpiToFlashMem(SPI_PKT *pkt);
bool ReadFlash(SPI_PKT *pkt);
bool ReadSpiCfg();
bool ReadFlashDevCfg();
bool TestFlashSector();
bool EraseWholeFlash();
void FlashWriteInit(SPI_PKT *pkt);
void WritePacketToFlash(SPI_PKT *pkt);

bool MT25QL512_Init(int DevNo, DeviceIntrf* pInterface);
bool MX25U1635E_init(int DevNo, DevIntrf_t* pInterface);
bool IS25LP512M_Init(int DevNo, DeviceIntrf* pInterface);
bool MX25U6435F_init(int DevNo, DeviceIntrf* pInterface);
bool FlashWriteDelayCallback(int DevNo, DeviceIntrf *pInterf);

void ToggleLed();


#endif // __BOARD_H__

