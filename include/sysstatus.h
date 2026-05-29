/**-------------------------------------------------------------------------
@file	sysstatus.h

@brief	System status standard.

 Stateless, header only encode / decode for the 32 bit status word
 defined in sysstatusdef.h. This file defines how to build and read
 a status value and how to classify its severity. It holds no state.

 How a subsystem stores, queues or reacts to a status is the user
 implementation and is intentionally not part of this standard.

 Usage rules :
   - The subsystem that first detects a failure encodes the full
	 word with its own module ID, a type and a site specific code.
   - Upper layers propagate the word verbatim unless they have a
	 more specific status to report. The module ID always points at
	 the originating subsystem.
   - A plain bool return drives control flow. The status word
	 carries diagnosis and is set only at the originating site.


@author	Hoang Nguyen Hoan
@date	Oct. 16, 1996

@license

Copyright (c) 1996-2026, I-SYST, All rights reserved

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

----------------------------------------------------------------------------
Modified by         Date            Description
Hoan                Mar. 18, 2005   namespace TS
Hoan                Nov. 18, 2014   Reimplementing for new EHAL C based
Hoan                May. 28, 2026   Reworked as stateless header only standard.
                                    Storage and queue removed, now user code.
----------------------------------------------------------------------------*/
#ifndef __SYSSTATUS_H__
#define __SYSSTATUS_H__

#include <stdbool.h>

#include "sysstatusdef.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Encode a status word from its three fields.
 *
 * @param   Type  : Severity class, a SYSSTATUS_TYPE value (pre shifted).
 * @param   ModId : Module ID, unshifted (e.g. SYSSTATUS_MODID_USB).
 * @param   Code  : Module code, 16 bits.
 *
 * @return  Encoded 32 bit status word.
 */
static inline SysStatus_t StatusEncode(SYSSTATUS_TYPE Type, uint32_t ModId, uint32_t Code) {
   return ((uint32_t)Type & SYSSTATUS_TYPE_MASK) |
          ((ModId << SYSSTATUS_MODID_POS) & SYSSTATUS_MODID_MASK) |
          (Code & SYSSTATUS_CODE_MASK);
}

/**
 * Get the type field, returned pre shifted so it compares directly against
 * the SYSSTATUS_TYPE values.
 */
static inline uint32_t StatusType(SysStatus_t Status) {
   return Status & SYSSTATUS_TYPE_MASK;
}

/**
 * Get the module ID field, returned unshifted so it compares directly
 * against the SYSSTATUS_MODID_xxx values.
 */
static inline uint32_t StatusModId(SysStatus_t Status) {
   return (Status & SYSSTATUS_MODID_MASK) >> SYSSTATUS_MODID_POS;
}

/**
 * Get the code field.
 */
static inline uint32_t StatusCode(SysStatus_t Status) {
   return Status & SYSSTATUS_CODE_MASK;
}

/**
 * True when the word carries no error, the Ready / No error resting value.
 */
static inline bool StatusIsOk(SysStatus_t Status) {
   return Status == SYSSTATUS_OK;
}

/**
 * True for runtime state words, which are state machine values, not errors.
 */
static inline bool StatusIsRuntime(SysStatus_t Status) {
   return StatusType(Status) == SYSSTATUS_TYPE_RNT;
}

/**
 * True for warning class words.
 */
static inline bool StatusIsWarning(SysStatus_t Status) {
   return StatusType(Status) == SYSSTATUS_TYPE_WRN;
}

/**
 * True for any error, non fatal or fatal.
 */
static inline bool StatusIsError(SysStatus_t Status) {
   uint32_t t = StatusType(Status);
   return t == SYSSTATUS_TYPE_ERR || t == SYSSTATUS_TYPE_FERR;
}

/**
 * True only for fatal error words.
 */
static inline bool StatusIsFatal(SysStatus_t Status) {
   return StatusType(Status) == SYSSTATUS_TYPE_FERR;
}

#ifdef __cplusplus
}

//
// C++ convenience wrapper. Thin inline pass through over the C inlines,
// also stateless.
//
class SysStatus {
public:
   SysStatus() : vStatus(SYSSTATUS_OK) {}
   SysStatus(SysStatus_t Status) : vStatus(Status) {}

   static SysStatus_t Encode(SYSSTATUS_TYPE Type, uint32_t ModId, uint32_t Code) {
      return StatusEncode(Type, ModId, Code);
   }

   SysStatus & operator = (STATUS Status) { vStatus = Status; return *this; }
   operator STATUS () const { return vStatus; }

   uint32_t Type(void) const   { return StatusType(vStatus); }
   uint32_t ModId(void) const  { return StatusModId(vStatus); }
   uint32_t Code(void) const   { return StatusCode(vStatus); }

   bool IsOk(void) const       { return StatusIsOk(vStatus); }
   bool IsRuntime(void) const  { return StatusIsRuntime(vStatus); }
   bool IsWarning(void) const  { return StatusIsWarning(vStatus); }
   bool IsError(void) const    { return StatusIsError(vStatus); }
   bool IsFatal(void) const    { return StatusIsFatal(vStatus); }

private:
   SysStatus_t vStatus;
};

#endif // __cplusplus

#endif // __SYSSTATUS_H__
