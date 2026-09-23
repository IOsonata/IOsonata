/**-------------------------------------------------------------------------
@file	dfu_image.h

@brief	MCUboot image format: header, TLV areas, hash and signature check.

The image a host uploads is the one imgtool signs:

	offset 0                header, HdrSize bytes (the 32 byte DfuImgHdr_t
							then padding)
	HdrSize                 payload, ImgSize bytes
	HdrSize + ImgSize       protected TLV area, ProtTlvSize bytes, may be 0
	...                     unprotected TLV area: info word, then TLVs

The SHA-256 TLV is the hash of everything before the unprotected TLV area.
The signature TLV signs that hash. The key hash TLV is the SHA-256 of the
public key in the DER form imgtool getpub prints.

Nothing here knows where the image is stored. The caller hands a read
function over the image offsets above, so the same code verifies an image
in one piece (slot 1) and one that is split between the record and the
application slot (slot 0).

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
#ifndef __DFU_IMAGE_H__
#define __DFU_IMAGE_H__

#include <stdint.h>
#include <stddef.h>

#include "crypto/icrypto.h"

/** @addtogroup DFU
  * @{
  */

#define DFU_IMG_MAGIC				0x96f3b83dUL	//!< Header magic
#define DFU_IMG_TLV_INFO_MAGIC		0x6907			//!< Unprotected TLV area
#define DFU_IMG_TLV_PROT_MAGIC		0x6908			//!< Protected TLV area

// TLV types, as imgtool writes them.
#define DFU_IMG_TLV_KEYHASH			0x01	//!< SHA-256 of the DER public key
#define DFU_IMG_TLV_SHA256			0x10	//!< Image hash
#define DFU_IMG_TLV_ECDSA_SIG		0x22	//!< ECDSA P-256 signature, DER

// Header flags that describe an image this loader cannot run.
#define DFU_IMG_F_PIC				0x00000001UL	//!< Position independent
#define DFU_IMG_F_ENC_AES128		0x00000004UL	//!< Encrypted, AES-128
#define DFU_IMG_F_ENC_AES256		0x00000008UL	//!< Encrypted, AES-256
#define DFU_IMG_F_NON_BOOTABLE		0x00000010UL	//!< Not an application
#define DFU_IMG_F_RAM_LOAD			0x00000020UL	//!< Runs from RAM
#define DFU_IMG_F_LZMA1				0x00000200UL	//!< Compressed, LZMA1
#define DFU_IMG_F_LZMA2				0x00000400UL	//!< Compressed, LZMA2
#define DFU_IMG_F_ARM_THUMB			0x00000800UL	//!< Compressed with ARM filter
#define DFU_IMG_F_UNSUPPORTED		(DFU_IMG_F_PIC | DFU_IMG_F_ENC_AES128 | \
									 DFU_IMG_F_ENC_AES256 | \
									 DFU_IMG_F_NON_BOOTABLE | \
									 DFU_IMG_F_RAM_LOAD | DFU_IMG_F_LZMA1 | \
									 DFU_IMG_F_LZMA2 | DFU_IMG_F_ARM_THUMB)

/// Largest header this loader keeps in the record. imgtool pads the header
/// to --header-size; 0x20 is the smallest and the one to use.
#define DFU_IMG_HDR_MAX				0x800

/// Largest TLV area, protected and unprotected together, kept in the record.
#define DFU_IMG_TLV_MAX				0x400

/// DER SubjectPublicKeyInfo of a P-256 key, as imgtool getpub prints it.
#define DFU_IMG_P256_DER_LEN		91

/// Where the 64 byte X,Y point starts in that DER form.
#define DFU_IMG_P256_DER_POINT		27

// Results. Negative, so a check for a negative return is enough.
#define DFU_IMG_OK					0
#define DFU_IMG_ERR_READ			-1		//!< The read function failed
#define DFU_IMG_ERR_MAGIC			-2		//!< Not an image
#define DFU_IMG_ERR_HDR				-3		//!< Header sizes or flags bad
#define DFU_IMG_ERR_SIZE			-4		//!< Does not fit where it is
#define DFU_IMG_ERR_TLV				-5		//!< TLV area malformed
#define DFU_IMG_ERR_NOHASH			-6		//!< No SHA-256 TLV
#define DFU_IMG_ERR_HASH			-7		//!< Hash does not match
#define DFU_IMG_ERR_KEY				-8		//!< Key hash does not match
#define DFU_IMG_ERR_SIG				-9		//!< Signature missing or bad
#define DFU_IMG_ERR_VECTOR			-10		//!< Vector table not for here
#define DFU_IMG_ERR_CRYPTO			-11		//!< Engine refused the work

#pragma pack(push, 1)

typedef struct __Dfu_Img_Version {
	uint8_t Major;
	uint8_t Minor;
	uint16_t Rev;
	uint32_t Build;
} DfuImgVer_t;

typedef struct __Dfu_Img_Header {
	uint32_t Magic;				//!< DFU_IMG_MAGIC
	uint32_t LoadAddr;			//!< Only for RAM load, 0 otherwise
	uint16_t HdrSize;			//!< Header size including padding
	uint16_t ProtTlvSize;		//!< Protected TLV area size, 0 when none
	uint32_t ImgSize;			//!< Payload size
	uint32_t Flags;				//!< DFU_IMG_F_...
	DfuImgVer_t Ver;			//!< Image version
	uint32_t Pad;
} DfuImgHdr_t;

typedef struct __Dfu_Img_Tlv_Info {
	uint16_t Magic;				//!< DFU_IMG_TLV_INFO_MAGIC or _PROT_MAGIC
	uint16_t Len;				//!< Area size including this info word
} DfuImgTlvInfo_t;

typedef struct __Dfu_Img_Tlv {
	uint16_t Type;
	uint16_t Len;				//!< Value length, not counting this header
} DfuImgTlv_t;

#pragma pack(pop)

static_assert(sizeof(DfuImgHdr_t) == 32, "MCUboot header is 32 bytes");

/// Reads Len bytes at image offset Off. Returns false when it cannot.
typedef bool (*DfuImgRead_t)(void *pCtx, uint32_t Off, void *pBuf,
							 uint32_t Len);

/// What DfuImgParse found in an image.
typedef struct __Dfu_Img_Info {
	DfuImgHdr_t Hdr;
	uint32_t TlvOff;			//!< Start of the TLV areas, HdrSize + ImgSize
	uint32_t TlvLen;			//!< Both TLV areas together
	uint32_t TotalLen;			//!< Whole image, header to last TLV
	uint8_t Sha[32];			//!< SHA-256 TLV value
	uint8_t KeyHash[32];		//!< Key hash TLV value, when bKeyHash
	uint8_t Sig[64];			//!< Signature as r then s, when bSig
	bool bKeyHash;
	bool bSig;
} DfuImgInfo_t;

/// A verification key.
typedef struct __Dfu_Img_Key {
	const uint8_t *pDer;		//!< DER SubjectPublicKeyInfo, P-256
	uint32_t DerLen;			//!< DFU_IMG_P256_DER_LEN
} DfuImgKey_t;

#ifdef __cplusplus

/**
 * @brief	Read and check an image header and its TLV areas.
 *
 * Checks the magic, the header and TLV sizes against MaxLen, the flags this
 * loader cannot honour, and that the TLV areas are well formed and hold one
 * SHA-256. Nothing is hashed.
 *
 * @param	Read   : Reads the image.
 * @param	pCtx   : Passed back to Read.
 * @param	MaxLen : Room the image has, the whole image must fit.
 * @param	pInfo  : Filled in.
 *
 * @return	DFU_IMG_OK or a DFU_IMG_ERR_ value.
 */
int DfuImgParse(DfuImgRead_t Read, void *pCtx, uint32_t MaxLen,
				DfuImgInfo_t *pInfo);

/**
 * @brief	Hash an image the way the SHA-256 TLV was made.
 *
 * Covers the header, the payload and the protected TLV area.
 *
 * @param	pHash   : SHA-256 engine.
 * @param	Read    : Reads the image.
 * @param	pCtx    : Passed back to Read.
 * @param	Info    : From DfuImgParse.
 * @param	pDigest : 32 bytes out.
 *
 * @return	DFU_IMG_OK or a DFU_IMG_ERR_ value.
 */
int DfuImgHash(HashEngine *pHash, DfuImgRead_t Read, void *pCtx,
			   const DfuImgInfo_t &Info, uint8_t *pDigest);

/**
 * @brief	Check an image hash, then its key hash and signature.
 *
 * With pSign null only the hash is checked: that shows the image arrived
 * whole, not who made it.
 *
 * With Read null nothing is hashed: the signature is checked over the
 * SHA-256 TLV value in Info, before the payload is there. The payload must
 * then be checked against that value once it is in.
 *
 * @param	pHash  : SHA-256 engine.
 * @param	pSign  : P-256 verify engine, or null for the hash alone.
 * @param	Key    : Key the signature must be made with, ignored when pSign
 * 					 is null.
 * @param	Read   : Reads the image, or null, see above.
 * @param	pCtx   : Passed back to Read.
 * @param	Info   : From DfuImgParse.
 *
 * @return	DFU_IMG_OK or a DFU_IMG_ERR_ value.
 */
int DfuImgVerify(HashEngine *pHash, SignEngine *pSign, const DfuImgKey_t &Key,
				 DfuImgRead_t Read, void *pCtx, const DfuImgInfo_t &Info);

/**
 * @brief	Check that a payload is a Cortex-M application for RunAddr.
 *
 * The first two vector table words: the stack pointer must be word aligned
 * in the part's RAM, the reset vector a Thumb address inside the payload
 * once it is at RunAddr. Called by the Cortex-M target layers.
 *
 * @param	pVec     : First two words of the payload.
 * @param	RunAddr  : Where the payload runs.
 * @param	ImgSize  : Payload size.
 * @param	RamStart : The stack pointer is above this.
 * @param	RamEnd   : and at most this.
 *
 * @return	true when it looks like an application for that address.
 */
bool DfuImgVectorValid(const uint32_t *pVec, uintptr_t RunAddr,
					   uint32_t ImgSize, uintptr_t RamStart, uintptr_t RamEnd);

#endif

/** @} */

#endif
