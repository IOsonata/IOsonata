/**-------------------------------------------------------------------------
@file	cbor.h

@brief	Small CBOR (RFC 8949) reader and writer.

The subset a management protocol needs: unsigned and negative integers,
byte and text strings, arrays, maps, true, false and null. The reader
takes definite and indefinite length maps and arrays, since host libraries
send both, and skips anything it is not asked for. Indefinite length
strings, floats and tags are rejected.

Reading is by field table: the caller lists the keys it wants and their
types, CborMapRead fills in what the map holds. Strings are returned in
place, pointing into the input.

Writing goes into a caller buffer and stops, with bOvf set, at the first
item that does not fit, so a sequence of puts needs one check at the end.

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
#ifndef __CBOR_H__
#define __CBOR_H__

#include <stdint.h>
#include <stdbool.h>

/** @addtogroup Utilities
  * @{
  */

/// Nesting the reader follows while skipping a value.
#define CBOR_DEPTH_MAX		8

typedef enum __Cbor_Fld_Type {
	CBOR_FLD_UINT,				//!< Unsigned integer
	CBOR_FLD_INT,				//!< Signed integer, either major type
	CBOR_FLD_BOOL,				//!< true or false
	CBOR_FLD_BSTR,				//!< Byte string, in place
	CBOR_FLD_TSTR,				//!< Text string, in place, not terminated
} CBOR_FLD_TYPE;

/// A key to look for in a map, and what was found.
typedef struct __Cbor_Fld {
	const char *pKey;			//!< Key, a text string in the map
	CBOR_FLD_TYPE Type;			//!< Type the value must have
	bool bFound;				//!< Set when the map held the key
	union {
		uint64_t U;				//!< CBOR_FLD_UINT
		int64_t I;				//!< CBOR_FLD_INT
		bool B;					//!< CBOR_FLD_BOOL
		struct {
			const uint8_t *p;
			uint32_t Len;
		} S;					//!< CBOR_FLD_BSTR, CBOR_FLD_TSTR
	};
} CborFld_t;

/// Table entry for a key and its type.
#define CBOR_FLD(Key, FldType)	{ .pKey = (Key), .Type = (FldType), \
								  .bFound = false, .U = 0 }

/// Output buffer.
typedef struct __Cbor_Wr {
	uint8_t *pBuf;
	uint32_t Size;
	uint32_t Len;				//!< Bytes written
	bool bOvf;					//!< Something did not fit
} CborWr_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Read the fields of a map.
 *
 * pData must hold exactly one map. A key not in the table is skipped, a key
 * in the table with a value of another type is an error, and so is a second
 * occurrence of a key. Fields not in the map are left with bFound false.
 *
 * @param	pData : Encoded map.
 * @param	Len   : Its length.
 * @param	pFld  : Keys wanted.
 * @param	NbFld : Number of entries in pFld.
 *
 * @return	true when pData is one well formed map.
 */
bool CborMapRead(const uint8_t *pData, uint32_t Len, CborFld_t *pFld,
				 int NbFld);

/// Begin output into pBuf.
void CborWrInit(CborWr_t *pWr, uint8_t *pBuf, uint32_t Size);

/// Map of NbPair key and value pairs, which follow.
void CborPutMap(CborWr_t *pWr, uint32_t NbPair);

/// Array of NbItem values, which follow.
void CborPutArray(CborWr_t *pWr, uint32_t NbItem);

void CborPutUint(CborWr_t *pWr, uint64_t Val);
void CborPutInt(CborWr_t *pWr, int64_t Val);
void CborPutBool(CborWr_t *pWr, bool Val);
void CborPutBstr(CborWr_t *pWr, const uint8_t *pData, uint32_t Len);
void CborPutTstr(CborWr_t *pWr, const char *pStr, uint32_t Len);

/// Null terminated text string, the usual form of a map key.
void CborPutStr(CborWr_t *pWr, const char *pStr);

#ifdef __cplusplus
}
#endif

/** @} */

#endif
