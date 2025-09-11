/**-------------------------------------------------------------------------
@file	i2c.h

@brief	Generic I2C driver definitions

Current implementation
	- Master mode
	- Slave mode
	- Polling
	- Interrupt

Each MCU supports different max speed. Currently known max speed.
	- Standard : 100 KHz
	- Fast mode : 400 KHz
	- Fast mode+ : 1000 KHz

@author	Hoang Nguyen Hoan
@date	Nov. 20, 2011

@license

MIT License

Copyright (c) 2011-2021 I-SYST inc. All rights reserved.

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
#ifndef __I2C_H__
#define __I2C_H__

#include <stdint.h>
#include <string.h>

#ifndef __cplusplus
#include <stdbool.h>
#endif

#include "device_intrf.h"
#include "iopincfg.h"

/** @addtogroup device_intrf	Device Interface
  * @{
  */

/// Minimum timing
#define I2C_SCL_STD_MODE_MAX_SPEED			100		// KHz
#define I2C_SCL_TLOW_STD_MODE_MIN			4700	// ns
#define I2C_SCL_THIGH_STD_MODE_MIN			4000	// ns

#define I2C_SCL_FAST_MODE_MAX_SPEED			400		// KHz
#define I2C_SCL_TLOW_FAST_MODE_MIN			1300	// ns
#define I2C_SCL_THIGH_FAST_MODE_MIN			600		// ns

#define I2C_SCL_FAST_MODE_PLUS_MAX_SPEED	1000	// KHz
#define I2C_SCL_TLOW_FAST_MODE_PLUS_MIN		500		// ns
#define I2C_SCL_THIGH_FAST_MODE_PLUS_MIN	260		// ns

// Data setup time
#define I2C_TSUDAT_STDMODE_MIN				250		// ns
#define I2C_TSUDAT_FASTMODE_MIN				100		// ns
#define I2C_TSUDAT_FASTMODEPLUS_MIN			50 		// ns

// Data setup time
#define I2C_THDDAT_STDMODE_MAX				3450	// ns
#define I2C_THDDAT_FASTMODE_MAX				900		// ns
#define I2C_THDDAT_FASTMODEPLUS_MAX			450 	// ns

// Rising time
#define I2C_TR_STDMODE_MAX					1000	// ns
#define I2C_TR_FASTMODE_MAX					300		// ns
#define I2C_TR_FASTMODEPLUS_MAX				120		// ns

// Falling time
#define I2C_TF_STDMODE_MAX					300		// ns
#define I2C_TF_FASTMODE_MAX					300		// ns
#define I2C_TF_FASTMODEPLUS_MAX				120		// ns

/// I2C Status code
typedef enum __I2C_Status {
	I2CSTATUS_START_COND = 8,		//!< Start condition transmitted
	I2CSTATUS_RESTART_COND = 0x10,	//!< Start condition re-transmitted
	I2CSTATUS_SLAW_ACK = 0x18,		//!< SLA+W has been transmitted; ACK has been received
	I2CSTATUS_SLAW_NACK = 0x20,		//!< SLA+W has been transmitted; NO ACK has been received
	I2CSTATUS_M_TXDAT_ACK = 0x28,	//!< Data byte in I2DAT has been transmitted; ACK has been received
	I2CSTATUS_M_TXDAT_NACK = 0x30,	//!< Data byte in I2DAT has been transmitted; NO ACK has been received
	I2CSTATUS_ARB_LOST = 0x38,		//!< Arbitration lost in SLA+R/W or Data bytes
	I2CSTATUS_SLAR_ACK = 0x40,		//!< SLA+R has been transmitted; ACK has been received
	I2CSTATUS_SLAR_NACK = 0x48,		//!< SLA+R has been transmitted; NO ACK has been received
	I2CSTATUS_RXDATA_ACK = 0x50,	//!< Data byte has been received; ACK has been returned
	I2CSTATUS_RXDATA_NACK = 0x58,	//!< ata byte has been received; NO ACK has been returned
	I2CSTATUS_OWNSLAW_ACK = 0x60,	//!< Own SLA+W has been received; ACK has been returned
	I2CSTATUS_ARB_LOST_OWNSLAW_NACK = 0x68,	//!< Arbitration lost in SLA+R/W as master; Own SLA+W has been received, ACK returned
	I2CSTATUS_GENCALL_ACK = 0x70,	//!< General Call address (0x00) has been received; ACK has been returned
	I2CSTATUS_ARB_LOST_GENCALL_NACK = 0x78,	//!< Arbitration lost in SLA+R/W as master; General Call address has been received, ACK has been returned
	I2CSTATUS_SLA_OWN_DATA_ACK = 0x80,	//!< Previously addressed with own SLA address; DATA has been received; ACK has been returned
	I2CSTATUS_SLA_OWN_DATA_NACK = 0x88,	//!< Previously addressed with own SLA address; DATA has been received; NO ACK has been returned
	I2CSTATUS_GENCALL_DATA_ACK = 0x90,	//!< Previously addressed with General Call; DATA byte has been received; ACK has been returned
	I2CSTATUS_GENCALL_DATA_NACK = 0x98,	//!< Previously addressed with General Call; DATA byte has been received; NO ACK has been returned
	I2CSTATUS_STOP_COND = 0xA0,		//!< A STOP condition or repeated START condition has been received while still addressed as Slave Receiver or Slave Transmitter
	I2CSTATUS_OWN_SLAR_ACK = 0xA8,	//!< Own SLA+R has been received; ACK has been returned
	I2CSTATUS_ARB_LOST_OWN_SLAR_ACK = 0xB0,	//!< Arbitration lost in SLA+R/W as master; Own SLA+R has been received, ACK has been returned
	I2CSTATUS_S_TXDAT_ACK = 0xB8,	//!< Data byte in I2DAT has been transmitted; ACK has been received
	I2CSTATUS_S_TXDAT_NACK = 0xC0,	//!< Data byte in I2DAT has been transmitted; NOT ACK has been received
	I2CSTATUS_S_LAST_TXDAT_ACK = 0xC8,	//!< Last data byte in I2DAT has been transmitted (AA = 0); ACK has been received
} I2CSTATUS;

typedef enum __I2C_Type {
	I2CTYPE_STANDARD,				//!< Standard I2C
	I2CTYPE_SMBUS					//!< SMBus
} I2CTYPE;

typedef enum __I2C_Mode {
	I2CMODE_MASTER,
	I2CMODE_SLAVE
} I2CMODE;

typedef enum __I2C_Address_Type {
	I2CADDR_TYPE_NORMAL,			//!< normal 7 bits
	I2CADDR_TYPE_EXT				//!< extended 10 bits address length
} I2CADDR_TYPE;

#define I2C_SLAVEMODE_MAX_ADDR		4	//!< Max number of response addresses in slave mode
	 									//!< the real implementation may support less depending on hardware

#define I2C_MAX_RETRY				5	//!< Max number of retries

/// I/O pin map index
#define I2C_SDA_IOPIN_IDX			0	//!< SDA pin index
#define I2C_SCL_IOPIN_IDX			1	//!< SCL pin index
#define I2C_SMBA_IOPIN_IDX			2	//!< SMBus Alert


#pragma pack(push, 4)

/// Configuration data used to initialize device
typedef struct __I2C_Config {
	int DevNo;				//!< I2C interface number
	I2CTYPE	Type;			//!< I2C type standard or SMBus
	I2CMODE Mode;			//!< Master/Slave mode
	const IOPinCfg_t *pIOPinMap;//!< Define I/O pins used by I2C
	int NbIOPins;			//!< Number of IO pins mapped
	uint32_t Rate;			//!< Speed in Hz
	int MaxRetry;			//!< Max number of retry
	I2CADDR_TYPE AddrType;	//!< I2C address type normal 7bits or extended 10bits
	int NbSlaveAddr;		//!< Number of slave mode address configured
	uint8_t SlaveAddr[I2C_SLAVEMODE_MAX_ADDR];	//!< I2C slave address used in slave mode only
	bool bDmaEn;			//!< true - Use DMA mode only on supported devices
	bool bIntEn;			//!< true - Interrupt enable
	int	IntPrio;			//!< Interrupt priority.  Value is implementation specific
//	bool bClkStretch;		//!< Clock stretching enable
	DevIntrfEvtHandler_t EvtCB;	//!< Interrupt based event callback function pointer. Must be set to NULL if not used
} I2CCfg_t;

typedef I2CCfg_t	TwiCfg_t;

/// Device driver data require by low level functions
typedef struct {
	I2CCfg_t Cfg;			//!< Config data
	DevIntrf_t DevIntrf;	//!< I2C device interface implementation
	uint8_t *pRRData[I2C_SLAVEMODE_MAX_ADDR];	//!< Pointer to data buffer to return upon receiving read request
	int RRDataLen[I2C_SLAVEMODE_MAX_ADDR];		//!< Read request data length in bytes
	uint8_t *pTRBuff[I2C_SLAVEMODE_MAX_ADDR];	//!< Pointer to buffer to receive data upon receiving write request
	int TRBuffLen[I2C_SLAVEMODE_MAX_ADDR];		//!< Write request buffer length in bytes
} I2CDev_t;

typedef I2CDev_t	TwiDev_t;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif	// __cplusplus

/**
 * @brief - I2C initialization.
 *
 * The main initialization of the i2c engine.  This must be implemented per architecture.
 *
 * @param	pDev 		: Pointer to be filed by this init function with implementation specific data
 * @param	pCfgData 	: Pointer to I2C configuration
 *
 * @return
 * 			- true	: Success
 * 			- false	: Failed
 */
bool I2CInit(I2CDev_t * const pDev, const I2CCfg_t *pCfgData);
void I2CBusReset(I2CDev_t * const pDev);
static inline int I2CGetRate(I2CDev_t * const pDev) { return pDev->DevIntrf.GetRate(&pDev->DevIntrf); }
static inline int I2CSetRate(I2CDev_t * const pDev, int Rate) {
	return pDev->DevIntrf.SetRate(&pDev->DevIntrf, Rate);
}
static inline void I2CEnable(I2CDev_t * const pDev) { DeviceIntrfEnable(&pDev->DevIntrf); }
static inline void I2CDisable(I2CDev_t * const pDev) { DeviceIntrfDisable(&pDev->DevIntrf); }
static inline void I2CPowerOff(I2CDev_t * const pDev) { DeviceIntrfPowerOff(&pDev->DevIntrf); }
static inline int I2CRx(I2CDev_t * const pDev, int DevAddr, uint8_t *pBuff, int Bufflen) {
	return DeviceIntrfRx(&pDev->DevIntrf, DevAddr, pBuff, Bufflen);
}
static inline int I2CTx(I2CDev_t * const pDev, int DevAddr, uint8_t *pData, int Datalen) {
	return DeviceIntrfTx(&pDev->DevIntrf, DevAddr, pData, Datalen);
}
static inline int I2CRead(I2CDev_t * const pDev, int DevAddr, uint8_t *pAdCmd, int AdCmdLen,
        uint8_t *pRxBuff, int RxLen) {
	return DeviceIntrfRead(&pDev->DevIntrf, DevAddr, pAdCmd, AdCmdLen, pRxBuff, RxLen);
}
static inline int I2CWrite(I2CDev_t * const pDev, int DevAddr, uint8_t *pAdCmd, int AdCmdLen,
        uint8_t *pTxData, int TxLen) {
	return DeviceIntrfWrite(&pDev->DevIntrf, DevAddr, pAdCmd, AdCmdLen, pTxData, TxLen);
}
static inline bool I2CStartRx(I2CDev_t * const pDev, int DevAddr) {
	return DeviceIntrfStartRx(&pDev->DevIntrf, DevAddr);
}
static inline int I2CRxData(I2CDev_t * const pDev, uint8_t *pBuff, int Bufflen) {
	return DeviceIntrfRxData(&pDev->DevIntrf, pBuff, Bufflen);
}
static inline void I2CStopRx(I2CDev_t * const pDev) { DeviceIntrfStopRx(&pDev->DevIntrf); }
static inline bool I2CStartTx(I2CDev_t * const pDev, int DevAddr) {
	return DeviceIntrfStartTx(&pDev->DevIntrf, DevAddr);
}
static inline int I2CTxData(I2CDev_t * const pDev, uint8_t *pData, int Datalen) {
	return DeviceIntrfTxData(&pDev->DevIntrf, pData, Datalen);
}
static inline void I2CStopTx(I2CDev_t * const pDev) { DeviceIntrfStopTx(&pDev->DevIntrf); }
static inline I2CDev_t *I2CGetDevHandle(DevIntrf_t * const pDevIntrf) { return (I2CDev_t *)DeviceIntrfGetHandle(pDevIntrf); }


/**
 * @brief	Set I2C slave data for read command.
 *
 * This function sets internal pointer to the location of data to be returned to I2C master upon
 * receiving read command.
 *
 * @param	pDev	: Pointer I2C driver data initialized be I2CInit function
 * @param	SlaveIdx: Slave address index to assign the data buffer
 * @param	pData	: Pointer to data buffer to send for read command
 * @param	DataLen	: Total data length in bytes
 *
 * @return	None
 */
void I2CSetReadRqstData(I2CDev_t * const pDev, int SlaveIdx, uint8_t * const pData, int DataLen);

/**
 * @brief	Set I2C slave buff for write command.
 *
 * This function sets internal pointer to the location of buffer to data from I2C master upon
 * receiving write command.
 *
 * @param	pDev	: Pointer I2C driver data initialized be I2CInit function
 * @param	SlaveIdx: Slave address index to assign the data buffer
 * @param	pBuff	: Pointer to data buffer to receive for write command
 * @param	BuffLen	: Total data length in bytes
 *
 * @return	None
 */
void I2CSetWriteRqstBuffer(I2CDev_t * const pDev, int SlaveIdx, uint8_t * const pBuff, int BuffLen);

#ifdef __cplusplus
}

/// Generic I2C base class
class I2C : public DeviceIntrf {
public:
	I2C() {
		memset((void*)&vDevData, 0, (int)sizeof(vDevData));
	}

	virtual ~I2C() {
		Disable();
	}

	I2C(I2C&);	// Copy ctor not allowed

	bool Init(const I2CCfg_t &CfgData) { return I2CInit(&vDevData, &CfgData); }
	operator DevIntrf_t * const () { return &vDevData.DevIntrf; }
	operator I2CDev_t& () { return vDevData; };	// Get device data
	operator I2CDev_t* const () { return &vDevData; };	// Get pointer to device data
	uint32_t Rate(uint32_t RateHz) { return DeviceIntrfSetRate(&vDevData.DevIntrf, RateHz); }
	uint32_t Rate(void) { return vDevData.Cfg.Rate; };	// Get rate in Hz
	void Enable(void) { DeviceIntrfEnable(&vDevData.DevIntrf); }
	void Disable(void) { DeviceIntrfDisable(&vDevData.DevIntrf); }
	virtual bool StartRx(uint32_t DevAddr) {
		return DeviceIntrfStartRx(&vDevData.DevIntrf, DevAddr);
	}
	// Receive Data only, no Start/Stop condition
	virtual int RxData(uint8_t *pBuff, int BuffLen) {
		return DeviceIntrfRxData(&vDevData.DevIntrf, pBuff, BuffLen);
	}
	virtual void StopRx(void) { DeviceIntrfStopRx(&vDevData.DevIntrf); }
	virtual bool StartTx(uint32_t DevAddr) {
		return DeviceIntrfStartTx(&vDevData.DevIntrf, DevAddr);
	}
	// Send Data only, no Start/Stop condition
	virtual int TxData(const uint8_t *pData, int DataLen) {
		return DeviceIntrfTxData(&vDevData.DevIntrf, pData, DataLen);
	}
	virtual void StopTx(void) { DeviceIntrfStopTx(&vDevData.DevIntrf); }


	/**
	 * @brief	Set I2C slave data for read command.
	 *
	 * This function sets internal pointer to the location of data to be returned to I2C master upon
	 * receiving read command.
	 *
	 * @param	SlaveIdx: Slave address index to assign the data buffer
	 * @param	pData	: Pointer to data buffer to send for read command
	 * @param	DataLen	: Total data length in bytes
	 *
	 * @return	None
	 */
	virtual void SetReadRqstData(int SlaveIdx, uint8_t * const pData, int DataLen) {
		I2CSetReadRqstData(&vDevData, SlaveIdx, pData, DataLen);
	}

	/**
	 * @brief	Set I2C slave buff for write command.
	 *
	 * This function sets internal pointer to the location of buffer to data from I2C master upon
	 * receiving write command.
	 *
	 * @param	SlaveIdx: Slave address index to assign the data buffer
	 * @param	pBuff	: Pointer to data buffer to receive for write command
	 * @param	BuffLen	: Total data length in bytes
	 *
	 * @return	None
	 */
	virtual void SetWriteRqstBuffer(int SlaveIdx, uint8_t * const pBuff, int BuffLen) {
		I2CSetWriteRqstBuffer(&vDevData, SlaveIdx, pBuff, BuffLen);
	}

private:
	I2CDev_t vDevData;
};

#endif	// __cplusplus

/** @} end group device_intrf */

#endif	// __I2C_H__
