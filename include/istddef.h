/**-------------------------------------------------------------------------
@file	istddef.h

@brief	Standard generic defines.

Contains software version data structure and application specific data.\n
Mostly for compatibilities.

@author Hoang Nguyen Hoan
@date	Jan. 16, 2012

@license

Copyright (c) 2012-2018, I-SYST inc., all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : hnhoan at i-syst dot com

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

#ifndef __ISTDDEF_H__
#define __ISTDDEF_H__

#ifndef __cplusplus
#include <stdbool.h>
#endif // __cplusplus

#include <stdint.h>

/** @addtogroup Utilities
  * @{
  */
#ifndef IOSONATA
#define IOSONATA
#endif

#ifdef _MSC_VER
// Microsoft does not support C99 inline
#ifndef inline
#define inline __forceinline
#endif
#endif

/// only for backward compatibility, otherwise useless.
#ifndef Bool
typedef bool	Bool;
#endif

/// only for backward compatibility, otherwise useless.
#ifndef FALSE
#define FALSE		false
#endif
/// only for backward compatibility, otherwise useless.
#ifndef TRUE
#define TRUE		true
#endif

#pragma pack(push, 1)

#define ISYST_BLUETOOTH_ID			0x0177	//!< I-SYST Bluetooth company identifier

///
/// Structure defining software version.
///
/// Version number MM.mm.ssss.bbbbbbbb\n
/// Where MM = Major, mm = minor, ssss = Subversion, bbbbbbbb = Build number
typedef struct __Version {
	union {
		uint16_t	Vers;   	//!< Verion number 0xMMmm, MM = Major, mm = minor (MM.mm)
		struct {
			unsigned Minor:8;	//!< Version major
			unsigned Major:8;	//!< Version minor
		};
	};
	uint16_t SubVers;		//!< Subversion
	uint32_t Build;			//!< Build number
} Vers_t;

typedef Vers_t	VERS;

#pragma pack(pop)

#define APPINFO_NAMESIZE_MAX		16		//!< Max size in bytes for application name
#define APPINFO_PRIVATESIZE_MAX		16		//!< Max size in bytes for private data

#pragma pack(push, 4)

///
/// Structure defining Application data.
///
/// It contains application identifier (Name), version and application specific private data
/// This data is usually static const located at specific location where bootloader/dfu can
/// access to validate.
typedef struct __App_Info {
	char Name[APPINFO_NAMESIZE_MAX];	//!< Application signature
	Vers_t Vers;							//!< Version number
	uint8_t Private[APPINFO_PRIVATESIZE_MAX];//!< APPINFO_PRIVATESIZE_MAX bytes private data
} AppInfo_t;

typedef AppInfo_t	APP_INFO;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

/// Inline min functions when there isn't one available.
#ifndef min
static inline int min(int x, int y) { return x > y ? y : x; }
#endif
#ifndef umin
static inline unsigned umin(unsigned x, unsigned y) { return x > y ? y : x; }
#endif
#ifndef llmin
static inline int64_t llmin(int64_t x, int64_t y) { return x > y ? y : x; }
#endif
#ifndef ullmin
static inline uint64_t ullmin(uint64_t x, uint64_t y) { return x > y ? y : x; }
#endif

/// An inline max function when there isn't one available.
#ifndef max
static inline int max(int x, int y) { return x > y ? x : y; }
#endif
#ifndef umax
static inline unsigned umax(unsigned x, unsigned y) { return x > y ? x : y; }
#endif
#ifndef llmax
static inline int64_t llmax(int64_t x, int64_t y) { return x > y ? x : y; }
#endif
#ifndef ullmax
static inline uint64_t ullmax(uint64_t x, uint64_t y) { return x > y ? x : y; }
#endif

#ifdef __cplusplus
}

/// Overloading min/max for C++ only
static inline unsigned min(unsigned x, unsigned y) { return x > y ? y : x; }
#ifndef __ICCARM__
static inline int32_t min(int32_t x, int32_t y) { return x > y ? y : x; }
static inline uint32_t min(uint32_t x, uint32_t y) { return x > y ? y : x; }
#endif
static inline int64_t min(int64_t x, int64_t y) { return x > y ? y : x; }
static inline uint64_t min(uint64_t x, uint64_t y) { return x > y ? y : x; }

static inline unsigned max(unsigned x, unsigned y) { return x > y ? x : y; }
#ifndef __ICCARM__
static inline int32_t max(int32_t x, int32_t y) { return x > y ? x : y; }
static inline uint32_t max(uint32_t x, uint32_t y) { return x > y ? x : y; }
#endif
static inline int64_t max(int64_t x, int64_t y) { return x > y ? x : y; }
static inline uint64_t max(uint64_t x, uint64_t y) { return x > y ? x : y; }
#endif

/** @} End of group Utilities */

#endif // __ISTDDEF_H__

