/**-------------------------------------------------------------------------
@file	dfu_mgr.h

@brief	DFU manager: the one API every DFU protocol uses.

Three layers, each replaced without touching the others:

	protocol	DfuWire (stage 0 wire protocol), DfuSmp (SMP), ...
				calls DfuMgr only, never the memory or the crypto
	DfuMgr		manifest check, image writer, hash, record or pending mark
	target		dfu_<mcu>, dfu_layout_*.ld

A protocol reads its bytes from any DeviceIntrf: UART, UsbdCdc, BtIntrf.
Each MCU target picks the protocol and the transport; nothing above the
target layer differs between MCUs.

An upload starts one of two ways:

	manifest first	BeginManifest with the image header and TLV areas, then
					the payload. The signature is checked before anything
					is erased or written. DfuWire uses this.
	image in order	BeginImage, then the image as imgtool wrote it: header,
					payload, TLVs. The signature can only be checked once
					all is in. SMP uses this. Refused when the
					configuration says bManifestOnly, as stage 0 does.

Either way nothing is startable until Finish has checked the image and
Commit has marked it: the record in direct mode, the pending word of the
slot 1 trailer otherwise.

Offsets given to Write are payload offsets for a manifest upload and image
offsets for an image upload. A Write at an offset before the one taken is
a resend: the part already taken is skipped. A Write past it is refused.

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
#ifndef __DFU_MGR_H__
#define __DFU_MGR_H__

#include <stdint.h>

#include "crypto/icrypto.h"
#include "dfu/dfu_boot.h"
#include "dfu/dfu_writer.h"

/** @addtogroup DFU
  * @{
  */

// Results beyond the DFU_IMG_ERR_ ones
#define DFU_MGR_ERR_STATE		-20		//!< Not valid in this state
#define DFU_MGR_ERR_OFFSET		-21		//!< Past what was taken
#define DFU_MGR_ERR_VERSION		-22		//!< Older than slot 0
#define DFU_MGR_ERR_LEN			-23		//!< Manifest length wrong

// Upload state
#define DFU_MGR_STATE_IDLE		0
#define DFU_MGR_STATE_RECV		1		//!< Taking the image
#define DFU_MGR_STATE_DONE		2		//!< Checked, Commit makes it count

// What a target picks in its board.h, DFU_PROTO and DFU_TRANSPORT
#define DFU_PROTO_WIRE			1		//!< DfuWire, dfu_wire.h
#define DFU_PROTO_SMP			2		//!< DfuSmp, dfu_smp.h
#define DFU_PROTO_NRFDFU		3		//!< nRF5 SDK DFU opcodes, not yet

#define DFU_TRANSPORT_UART		1
#define DFU_TRANSPORT_CDC		2		//!< USB CDC ACM, UsbdCdc
#define DFU_TRANSPORT_BLE		3		//!< BtIntrf, application only

/// Largest manifest: header and both TLV areas.
#define DFU_MGR_MANIFEST_MAX	(DFU_IMG_HDR_MAX + DFU_IMG_TLV_MAX)

#ifdef __cplusplus

typedef struct __DfuMgr_Cfg {
	const DfuStore_t *pStore;	//!< Slot 1, or slot 0 in direct mode
	bool bDirect;				//!< Payload into slot 0, record last (stage 0)
	bool bManifestOnly;			//!< Refuse BeginImage (stage 0)
	bool bAllowDowngrade;		//!< Take an image older than slot 0
	HashEngine *pHash;			//!< SHA-256
	const DfuBootCfg_t *pBoot;	//!< Keys and verify engine. Needed in direct
								//!< mode; in slot 1 mode, when given, a
								//!< manifest signature is checked at once
} DfuMgrCfg_t;

class DfuMgr {
public:
	DfuMgr();

	/**
	 * @brief	Set up over the layout the linker declares.
	 *
	 * @return	false when the store is not the region the layout says, or
	 * 			direct mode has no keys.
	 */
	bool Init(const DfuMgrCfg_t &Cfg);

	/**
	 * @brief	Start an upload from its manifest.
	 *
	 * Checks, in order and before touching the memory: header, sizes, TLV
	 * areas, key hash and signature over the SHA-256 value, version. Then
	 * the record (direct) or the trailer (slot 1) is cleared.
	 *
	 * @param	pMan : Image header, HdrSize bytes, then both TLV areas.
	 * @param	Len  : At most DFU_MGR_MANIFEST_MAX.
	 *
	 * @return	DFU_IMG_OK, a DFU_IMG_ERR_ or DFU_MGR_ERR_ value.
	 */
	int BeginManifest(const uint8_t *pMan, uint32_t Len);

	/**
	 * @brief	Start an upload of a whole image sent in order.
	 *
	 * @param	Len : Image length, header to last TLV.
	 *
	 * @return	DFU_IMG_OK, DFU_IMG_ERR_SIZE, DFU_MGR_ERR_STATE when the
	 * 			configuration is manifest only.
	 */
	int BeginImage(uint32_t Len);

	/**
	 * @brief	Next bytes of the upload.
	 *
	 * @param	Off   : Payload offset (manifest) or image offset (image).
	 * @param	pData : Bytes.
	 * @param	Len   : Count.
	 *
	 * @return	DFU_IMG_OK, DFU_MGR_ERR_OFFSET, DFU_MGR_ERR_STATE,
	 * 			DFU_IMG_ERR_SIZE, or the writer's failure, which ends the
	 * 			upload.
	 */
	int Write(uint32_t Off, const uint8_t *pData, uint32_t Len);

	/**
	 * @brief	All in: check the image.
	 *
	 * Hash from the memory against the SHA-256 value, entry for slot 0,
	 * and the signature: already done for a manifest (the value is the one
	 * signed), done here for an image in direct mode, left to stage 0 for
	 * an image in slot 1.
	 *
	 * @param	pInfo : Filled in, may be null.
	 *
	 * @return	DFU_IMG_OK or an error; any error ends the upload.
	 */
	int Finish(DfuImgInfo_t *pInfo);

	/**
	 * @brief	Make the checked image count.
	 *
	 * Direct: write the record, magic last. Slot 1: mark it pending, which
	 * also works on an image left in slot 1 by an earlier session.
	 */
	bool Commit(void);

	/// Drop the upload. What was written stays, unmarked.
	void Abort(void);

	/// Restart. bRecovery: stay in stage 0 recovery after the reset.
	void Reset(bool bRecovery) __attribute__((noreturn));

	/// Init succeeded.
	bool Ready(void) const { return vbInit; }

	int State(void) const { return vState; }
	bool Active(void) const { return vState == DFU_MGR_STATE_RECV; }
	bool Manifest(void) const { return vbManifest; }
	bool Direct(void) const { return vCfg.bDirect; }
	bool AllowDowngrade(void) const { return vCfg.bAllowDowngrade; }

	/// Bytes taken, payload (manifest) or image (image).
	uint32_t Offset(void) const { return vOff; }

	/// Bytes the upload has in all, payload or image.
	uint32_t Length(void) const { return vLen; }

	/// CRC-32 IEEE of the bytes taken, as zlib crc32 gives it.
	uint32_t Crc(void) const { return vCrc; }

	/// Largest image this takes, header to last TLV.
	uint32_t MaxLen(void) const { return vWr.MaxLen(); }

	/// Version of the image in slot 0, zero when there is none.
	const DfuImgVer_t &CurVer(void) const { return vCurVer; }

	const DfuLayout_t &Layout(void) const { return vLay; }

	/// SHA-256 engine of the configuration.
	HashEngine *Hash(void) const { return vCfg.pHash; }

	/// The writer, for slot 1 listing and marking by a protocol.
	DfuWriter &Writer(void) { return vWr; }

	/**
	 * @brief	Where a protocol may gather a manifest sent in pieces.
	 *
	 * DFU_MGR_MANIFEST_MAX bytes, the record body. Gathering there drops
	 * any upload under way; call Abort first. Then BeginManifest on it.
	 */
	static uint8_t *ManifestBuf(void);

	/// DfuImgRead_t over the image written, pCtx is this DfuMgr.
	static bool Read(void *pCtx, uint32_t Off, void *pBuf, uint32_t Len);

private:
	int Fail(int Res) { vState = DFU_MGR_STATE_IDLE; vWr.Abort(); return Res; }

	DfuMgrCfg_t vCfg;
	DfuLayout_t vLay;
	DfuWriter vWr;
	bool vbInit;

	int vState;
	bool vbManifest;
	uint32_t vOff;
	uint32_t vLen;
	uint32_t vCrc;
	uint32_t vHdrSize;			//!< Manifest: header bytes
	uint32_t vTlvLen;			//!< Manifest: TLV bytes, after the header
	uint8_t vSha[32];			//!< Manifest: the SHA-256 value signed
	DfuImgVer_t vCurVer;
};

#endif

/** @} */

#endif
