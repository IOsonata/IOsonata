/*---------------------------------------------------------------------------
File : lpc11uxx_iap.h

Author : Hoang Nguyen Hoan          Jan. 26, 2015

Desc : LPC11Uxx IAP functions

Copyright (c) 2015, I-SYST inc, all rights reserved

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

----------------------------------------------------------------------------
Modified by          Date              Description
----------------------------------------------------------------------------*/

#ifndef __LPC11UXX_IAP_H__
#define __LPC11UXX_IAP_H__

// Compatibility include for existing LPC11Uxx applications. The common
// declarations and command structures are shared with the other LPC ports.
#include "lpc_iap.h"

// Preserve the command name exported by older LPC11Uxx headers.
#define IAPCMD_CPY_RAM_TO_FLASH IAPCMD_COPY_RAM_TO_FLASH

#endif // __LPC11UXX_IAP_H__ 
