/**-------------------------------------------------------------------------
@file	dfu_smp.h

@brief	SMP server: OS and image management groups.

The protocol the SMP host tools speak: nRF Connect Device Manager,
nRF Connect for Mobile, nrfutil mcu-manager, mcumgr, smpmgr. Each packet is
an 8 byte header then a CBOR map:

	byte 0		bits 0-2 operation, bits 3-4 protocol version
	byte 1		flags
	byte 2-3	payload length, big endian
	byte 4-5	group, big endian
	byte 6		sequence number
	byte 7		command id

DfuSmp is transport independent. A transport reassembles one request, hands
it to Process and sends back what Process wrote. Reassembly across
transport frames is defined per transport by SMP and belongs there.

Process may erase and write memory and hashes images, which takes time, so
call it from the application event queue, never from a stack callback or an
interrupt.

All image work is DfuMgr's (dfu_mgr.h); DfuSmp is one protocol over it.
SMP sends an image in order, header first and TLVs last, so the manager
checks the signature only once all is in:

	application	OTA. Uploads go to slot 1 through the store the manager
				has: an Nvm where a radio stack arbitrates the memory, the
				target layer elsewhere. Stage 0 installs them.
	stage 0		Possible (direct mode, manager not manifest only), but the
				stage 0 protocol is DfuWire, which checks the signature
				before anything is written.

Only the running image and one upload slot exist. The boot overwrites slot 0,
there is no revert, so image test and confirm both mark slot 1 pending, and
confirming the running image changes nothing.

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
#ifndef __DFU_SMP_H__
#define __DFU_SMP_H__

#include <stdint.h>
#include <stddef.h>

#include "cbor.h"
#include "crypto/icrypto.h"
#include "dfu/dfu_boot.h"
#include "dfu/dfu_mgr.h"

/** @addtogroup DFU
  * @{
  */

#define DFUSMP_HDR_SIZE				8

// Operations
#define DFUSMP_OP_READ				0
#define DFUSMP_OP_READ_RSP			1
#define DFUSMP_OP_WRITE				2
#define DFUSMP_OP_WRITE_RSP			3

// Protocol versions, header bits 3-4
#define DFUSMP_VER_1				0
#define DFUSMP_VER_2				1

// Groups
#define DFUSMP_GRP_OS				0
#define DFUSMP_GRP_IMG				1

// OS group commands
#define DFUSMP_OS_ECHO				0
#define DFUSMP_OS_RESET				5
#define DFUSMP_OS_PARAMS			6
#define DFUSMP_OS_BOOTLOADER		8

// Image group commands
#define DFUSMP_IMG_STATE			0
#define DFUSMP_IMG_UPLOAD			1
#define DFUSMP_IMG_ERASE			5

// Generic results, the "rc" of a response
#define DFUSMP_ERR_OK				0
#define DFUSMP_ERR_EUNKNOWN			1
#define DFUSMP_ERR_ENOMEM			2
#define DFUSMP_ERR_EINVAL			3
#define DFUSMP_ERR_ENOENT			5
#define DFUSMP_ERR_EBADSTATE		6
#define DFUSMP_ERR_EMSGSIZE			7
#define DFUSMP_ERR_ENOTSUP			8
#define DFUSMP_ERR_ECORRUPT			9
#define DFUSMP_ERR_EBUSY			10
#define DFUSMP_ERR_TOO_NEW			13

// OS group results, protocol version 2
#define DFUSMP_OS_ERR_NO_ANSWER		3	//!< Query yields no answer

// Image group results, protocol version 2
#define DFUSMP_IMG_ERR_UNKNOWN		1
#define DFUSMP_IMG_ERR_NO_IMAGE		3
#define DFUSMP_IMG_ERR_NO_TLVS		4
#define DFUSMP_IMG_ERR_INVALID_TLV	5
#define DFUSMP_IMG_ERR_HASH_NOT_FOUND	8
#define DFUSMP_IMG_ERR_READ_FAILED	11
#define DFUSMP_IMG_ERR_WRITE_FAILED	12
#define DFUSMP_IMG_ERR_ERASE_FAILED	13
#define DFUSMP_IMG_ERR_INVALID_SLOT	14
#define DFUSMP_IMG_ERR_INVALID_LENGTH	21
#define DFUSMP_IMG_ERR_INVALID_HEADER	22
#define DFUSMP_IMG_ERR_INVALID_MAGIC	23
#define DFUSMP_IMG_ERR_INVALID_HASH	24
#define DFUSMP_IMG_ERR_VECTOR_TABLE	29
#define DFUSMP_IMG_ERR_TOO_LARGE	30
#define DFUSMP_IMG_ERR_DATA_OVERRUN	31

/// Boot mode answered to a bootloader "mode" query from the application:
/// overwrite only.
#define DFUSMP_BOOT_MODE_OVERWRITE	2

/// Boot mode answered from stage 0 recovery: single application, the boot
/// writes it in place.
#define DFUSMP_BOOT_MODE_SINGLE		0

#ifdef __cplusplus

/// Called when a host asks for a reset. The response has not gone out yet:
/// reset once the transport has sent it.
typedef void (*DfuSmpResetCb_t)(void *pCtx);

typedef struct __DfuSmp_Cfg {
	DfuMgr *pMgr;				//!< Initialised manager: store, mode, keys.
								//!< Not manifest only: SMP sends the image
								//!< in order
	uint16_t BufSize;			//!< Largest request the transport reassembles
	DfuSmpResetCb_t ResetCB;	//!< Reset asked, null when not allowed
	void *pCtx;					//!< Passed back to ResetCB
} DfuSmpCfg_t;

class DfuSmp {
public:
	DfuSmp();

	/**
	 * @brief	Bring the server up over an initialised DfuMgr.
	 *
	 * @param	Cfg : Configuration.
	 *
	 * @return	true on success.
	 */
	bool Init(const DfuSmpCfg_t &Cfg);

	/**
	 * @brief	Handle one request.
	 *
	 * @param	pReq    : Request packet, header and payload.
	 * @param	ReqLen  : Its length.
	 * @param	pRsp    : Response buffer.
	 * @param	RspSize : Its size, 512 is enough for every response here.
	 *
	 * @return	Response length, 0 when the request gets no response: a
	 * 			malformed header or a packet that is not a request.
	 */
	int Process(const uint8_t *pReq, uint32_t ReqLen, uint8_t *pRsp,
				uint32_t RspSize);

	/// True while an upload is under way.
	bool Uploading(void) const { return vCfg.pMgr->Active(); }

	/// Drop an upload that will not be finished, as when the link that
	/// carried it has gone. What was written stays unmarked; the host starts
	/// again from 0.
	void UploadAbort(void) { vCfg.pMgr->Abort(); }

private:
	// Handlers fill the response map and return a generic result; a group
	// result, when there is one, goes in GrpRc.
	int OsEcho(const uint8_t *p, uint32_t Len, CborWr_t &Wr);
	int OsReset(CborWr_t &Wr);
	int OsParams(CborWr_t &Wr);
	int OsBootInfo(const uint8_t *p, uint32_t Len, CborWr_t &Wr, int &GrpRc);
	int ImgList(CborWr_t &Wr, int &GrpRc);
	int ImgState(const uint8_t *p, uint32_t Len, CborWr_t &Wr, int &GrpRc);
	int ImgUpload(const uint8_t *p, uint32_t Len, CborWr_t &Wr, int &GrpRc);
	int ImgErase(CborWr_t &Wr, int &GrpRc);

	int UploadDone(CborWr_t &Wr, int &GrpRc);

	DfuSmpCfg_t vCfg;
	bool vbInit;

	uint8_t vSha[32];			//!< Whole file SHA-256 the host sent
	bool vbSha;
};

#endif

/** @} */

#endif
