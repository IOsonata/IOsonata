/**-------------------------------------------------------------------------
@file	endian.h

@brief	Compatibility shim for the Arm CC3xx low level driver sources

		The driver sources include the glibc style <endian.h> and use the
		bswap_32/bswap_64 macros. newlib on arm-none-eabi has neither, so
		this shim provides exactly those two over the compiler builtins.
		The include directory holding this file must precede the system
		include path only for the driver compilation units.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST. See bt_smp.h for full text.
----------------------------------------------------------------------------*/
#ifndef __CC3XX_COMPAT_ENDIAN_H__
#define __CC3XX_COMPAT_ENDIAN_H__

#ifndef bswap_32
#define bswap_32(x)		__builtin_bswap32(x)
#endif

#ifndef bswap_64
#define bswap_64(x)		__builtin_bswap64(x)
#endif

#endif // __CC3XX_COMPAT_ENDIAN_H__
