/**-------------------------------------------------------------------------
@file	rftag.h

@brief	RF tag device class

RFTag is the device class facet for a tag presented to an RF reader, the same
position Sensor holds for measurement devices. A concrete subclass implements
the facet for its hardware: a raw frame transport such as the Nordic NFCT
peripheral serves the tag from a local memory image and binds a protocol
engine, a chip that runs the protocol in silicon such as the ST25DV maps the
facet onto its registers and binds no engine. Application code programs the
facet only and does not depend on which side runs the protocol.

RFTagProto is the base of the protocol engines, the processing layer of this
subsystem in the position fusion holds for motion. An engine is a device
independent state machine consuming reader frames and producing responses
through the tag memory access of the facet. Engines contain no hardware
knowledge.

The base RFTag implements a local memory tag directly: MemRead and MemWrite
operate on the configured memory image, SetNdef and GetNdef wrap the NDEF
formats over it. Bus attached tag chips override MemRead and MemWrite with
their bus access.

Write protection is board wiring, not tag protocol. The facet exposes
SetWriteProt which reports unsupported unless the configuration provides a
control callback or a subclass overrides it with a native mechanism.

@author	Hoang Nguyen Hoan
@date	Jul. 7, 2026

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
#ifndef __RFTAG_H__
#define __RFTAG_H__

#include <stdint.h>
#include <string.h>

#include "device.h"

/// NDEF container format inside the tag memory
typedef enum __RFTag_Ndef_Fmt {
	RFTAG_NDEF_FMT_NONE = 0,		//!< Raw NDEF bytes at NdefAddr
	RFTAG_NDEF_FMT_NLEN16,			//!< 2 byte big endian length then NDEF bytes
	RFTAG_NDEF_FMT_TLV,				//!< NFC Forum TLV: 0x03 LEN DATA 0xFE
} RFTAG_NDEF_FMT;

/// Tag events raised through RFTag::EvtHandler
typedef enum __RFTag_Evt {
	RFTAG_EVT_FIELD_ON,				//!< Reader field detected
	RFTAG_EVT_FIELD_OFF,			//!< Reader field lost
	RFTAG_EVT_SELECTED,				//!< Tag selected or activated by the reader
	RFTAG_EVT_DESELECTED,			//!< Tag deselected
	RFTAG_EVT_READ,					//!< Reader read tag memory
	RFTAG_EVT_WRITE,				//!< Reader wrote tag memory
	RFTAG_EVT_MEM_CHANGED,			//!< Tag memory content changed
	RFTAG_EVT_ERROR,				//!< Protocol or transport error
} RFTAG_EVT;

/**
 * @brief	Write protect control callback.
 *
 * Board level hook driving whatever realizes protection: a GPIO, an
 * expander pin, a power latch or a policy. Return true when applied.
 */
typedef bool (*RFTAGWRPROTCB)(void *pCtx, bool bVal);

#define RFTAG_NFCID_MAX_LEN			10

/// RFTag configuration
typedef struct __RFTag_Config {
	uint8_t NfcId[RFTAG_NFCID_MAX_LEN];	//!< Tag id, protocol defines the valid length
	int IdLen;						//!< Id length in bytes, 0 selects the engine default
	bool bReadOnly;					//!< Tag is read only on the RF side
	uint8_t *pMem;					//!< Local tag memory image, null for bus tag chips
	uint32_t MemSize;				//!< Tag memory size in bytes
	uint32_t NdefAddr;				//!< NDEF area offset in tag memory
	uint32_t NdefMaxLen;			//!< NDEF area size in bytes
	RFTAG_NDEF_FMT NdefFmt;			//!< NDEF container format
	RFTAGWRPROTCB WrProtCB;			//!< Optional write protect control, null when unsupported
	void *pWrProtCtx;				//!< Context for WrProtCB
} RFTagCfg_t;

class RFTag;

/**
 * @brief	Protocol engine base class.
 *
 * A target side tag protocol state machine, bound to an RFTag by Attach. The
 * engine reaches tag memory only through the facet, it holds no hardware
 * knowledge. One engine instance serves one tag instance.
 */
class RFTagProto {
public:
	RFTagProto() : vpTag(nullptr) {}
	virtual ~RFTagProto() {}

	/**
	 * @brief	Bind and prepare the engine for the given tag.
	 *
	 * Validates the tag configuration for the protocol and initializes the
	 * protocol area of the tag memory.
	 *
	 * @param	pTag	Tag the engine serves
	 *
	 * @return	true when the tag configuration fits the protocol
	 */
	virtual bool Init(RFTag * const pTag) = 0;

	/**
	 * @brief	Handle one reader frame.
	 *
	 * @param	pRx		Received frame
	 * @param	RxLen	Received length in bytes
	 * @param	pTx		Response buffer
	 * @param	TxCap	Response buffer capacity
	 *
	 * @return	Response length in bytes, 0 for no response
	 */
	virtual int OnFrame(const uint8_t *pRx, int RxLen, uint8_t *pTx, int TxCap) = 0;

protected:
	RFTag *vpTag;					//!< Tag served by this engine
};

/**
 * @brief	RF tag device class facet.
 *
 * The base implements a local memory tag. Concrete subclasses map the facet
 * onto their hardware and override what differs.
 */
class RFTag : virtual public Device {
public:
	RFTag() : vpProto(nullptr) {
		memset(&vCfg, 0, sizeof(vCfg));
	}
	virtual ~RFTag() {}

	/**
	 * @brief	Initialize the tag.
	 *
	 * The base requires a local memory image covering the NDEF area. A bus
	 * tag subclass overrides and uses the interface instead.
	 *
	 * @param	pCfg	Tag configuration
	 * @param	pIntrf	Bus interface for bus tag chips, null for local tags
	 *
	 * @return	true on success
	 */
	virtual bool Init(const RFTagCfg_t &Cfg, DeviceIntrf * const pIntrf = nullptr);

	/**
	 * @brief	Turn the RF side on. The base has no RF hardware, no op.
	 */
	virtual bool Enable() { return true; }

	/**
	 * @brief	Turn the RF side off. The base has no RF hardware, no op.
	 */
	virtual void Disable() {}

	/**
	 * @brief	Reset tag state. Reinitializes the bound engine when present.
	 */
	virtual void Reset();

	/**
	 * @brief	Bind a protocol engine and initialize it for this tag.
	 *
	 * A tag chip running the protocol in silicon never calls this. A frame
	 * transport tag binds exactly one engine.
	 *
	 * @param	pProto	Engine instance
	 *
	 * @return	true when the engine accepts the tag configuration
	 */
	virtual bool Attach(RFTagProto * const pProto);

	/**
	 * @brief	Bound protocol engine, null when the chip runs the protocol.
	 */
	RFTagProto *Proto() const { return vpProto; }

	/**
	 * @brief	Feed one reader frame to the bound engine.
	 *
	 * Called by the frame transport on frame reception.
	 *
	 * @param	pRx		Received frame
	 * @param	RxLen	Received length in bytes
	 * @param	pTx		Response buffer
	 * @param	TxCap	Response buffer capacity
	 *
	 * @return	Response length in bytes, 0 for no response
	 */
	virtual int ProcessFrame(const uint8_t *pRx, int RxLen, uint8_t *pTx, int TxCap);

	/**
	 * @brief	Read tag memory. The base reads the local image.
	 *
	 * @param	Addr	Tag memory offset
	 * @param	pBuff	Destination
	 * @param	Len		Byte count
	 *
	 * @return	Bytes read
	 */
	virtual int MemRead(uint32_t Addr, uint8_t *pBuff, int Len);

	/**
	 * @brief	Write tag memory. The base writes the local image.
	 *
	 * Host side write, not subject to the RF read only policy. Engines
	 * enforce bReadOnly on reader initiated writes.
	 *
	 * @param	Addr	Tag memory offset
	 * @param	pData	Source
	 * @param	Len		Byte count
	 *
	 * @return	Bytes written
	 */
	virtual int MemWrite(uint32_t Addr, const uint8_t *pData, int Len);

	/**
	 * @brief	Store an NDEF message in the configured container format.
	 *
	 * @param	pNdef	NDEF message bytes
	 * @param	Len		Message length in bytes
	 *
	 * @return	true on success
	 */
	virtual bool SetNdef(const uint8_t *pNdef, uint16_t Len);

	/**
	 * @brief	Read the NDEF message back from the container format.
	 *
	 * @param	pNdef	Destination
	 * @param	Len		Destination capacity
	 *
	 * @return	Message length read, 0 when none
	 */
	virtual int GetNdef(uint8_t *pNdef, uint16_t Len);

	/**
	 * @brief	Apply or release write protection.
	 *
	 * The base drives the configured callback. A subclass with a native
	 * mechanism overrides. Without either, reports unsupported.
	 *
	 * @param	bVal	true protects, false releases
	 *
	 * @return	true when applied, false when failed or unsupported
	 */
	virtual bool SetWriteProt(bool bVal);

	/**
	 * @brief	Tag event sink, override to observe tag activity.
	 *
	 * Called by engines and transports. The base ignores events.
	 *
	 * @param	Evt	Event code
	 * @param	P0	Event parameter, address for memory events
	 * @param	P1	Event parameter, length for memory events
	 */
	virtual void EvtHandler(RFTAG_EVT Evt, uint32_t P0, uint32_t P1) {
		(void)Evt; (void)P0; (void)P1;
	}

	uint32_t MemSize() const { return vCfg.MemSize; }
	bool ReadOnly() const { return vCfg.bReadOnly; }
	const uint8_t *NfcId() const { return vCfg.NfcId; }
	int IdLen() const { return vCfg.IdLen; }
	uint32_t NdefAddr() const { return vCfg.NdefAddr; }
	uint32_t NdefMaxLen() const { return vCfg.NdefMaxLen; }
	RFTAG_NDEF_FMT NdefFmt() const { return vCfg.NdefFmt; }

protected:
	RFTagCfg_t vCfg;				//!< Tag configuration copy
	RFTagProto *vpProto;			//!< Bound protocol engine

private:
	bool SetNdefNLen16(const uint8_t *pNdef, uint16_t Len);
	bool SetNdefTlv(const uint8_t *pNdef, uint16_t Len);
	int GetNdefNLen16(uint8_t *pNdef, uint16_t Len);
	int GetNdefTlv(uint8_t *pNdef, uint16_t Len);
};

#endif // __RFTAG_H__
