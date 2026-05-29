/**--------------------------------------------------------------------------
@file	sysstatusdef.h

@brief	System status definitions and encoding.

 A system status is a single 32 bit value that identifies what
 happened, where it happened and how severe it is.  It is the
 common status word shared by all subsystems.

 Layout :

 31           28|27                 16|15                          0
 +--------------+---------------------+-----------------------------+
 | Type (4 bits)| Module ID (12 bits) |        Code (16 bits)       |
 +--------------+---------------------+-----------------------------+

 Type     : Severity class of the status. See SYSSTATUS_TYPE.
 Module ID: Identifies the subsystem that raised the status.
			Every subsystem reserves its own ID in this file.
 Code     : Meaning within the module. Codes are allocated per
			originating site by the module owner, not just per
			symptom. 16 bits give 65536 codes per module.

 The encoded value 0 is reserved for Ready / No error. It is the
 resting value returned when nothing is wrong.

 Type values (top nibble) :
		0000b  Runtime state  (state machine value, not an error)
		0100b  Warning        (needs attention)
		1000b  Non fatal error(recoverable)
		1111b  Fatal error    (system halt)

 Common codes shared by all modules are defined and maintained
 here. Module specific codes start at SYSSTATUS_CODE_USERSTART.

 Module ID :

	---- Applications
	0x000 : Main application

	---- Library modules (from 0x100)
	0x100 : OS runtime (multi thread, IPC, object kernel)
	0x101 : Component class
	0x102 : Adaptor class
	0x110 : File I/O class
	0x111 : Multi processor data transfer class
	0x112 : Audio processor class
	0x113 : Video processor class
	0x114 : Mpeg processor class
	0x115 : USB


@author : Hoang Nguyen Hoan
@date Oct. 16, 1996

@license

MIT License


Copyright (c) 1996-2026, I-SYST, all rights reserved

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
Hoan                Nov. 18, 2014   Change module IDs
Hoan                May. 28, 2026   Code field widened to 16 bits, reserved
                                    field removed. Added field shift and
                                    accessor defines.
----------------------------------------------------------------------------*/
#ifndef __SYSSTATUSDEF_H__
#define __SYSSTATUSDEF_H__

// Field masks
#define SYSSTATUS_TYPE_MASK         0xf0000000UL
#define SYSSTATUS_MODID_MASK        0x0fff0000UL
#define SYSSTATUS_CODE_MASK         0x0000ffffUL

// Field shifts
#define SYSSTATUS_TYPE_POS          28
#define SYSSTATUS_MODID_POS         16
#define SYSSTATUS_CODE_POS          0

/**
 * System status types. Values are pre shifted into the top nibble so they
 * can be OR'ed straight into an encoded status word.
 */
typedef enum _System_Status_Type {
   SYSSTATUS_TYPE_RNT   = 0x00000000UL,     // Runtime state
   SYSSTATUS_TYPE_WRN   = 0x40000000UL,     // Warning
   SYSSTATUS_TYPE_ERR   = 0x80000000UL,     // Non fatal error
   SYSSTATUS_TYPE_FERR  = 0xf0000000UL      // Fatal error
} SYSSTATUS_TYPE;

/**
 * Reserve module Id here. Each value occupies the module field once shifted
 * by SYSSTATUS_MODID_POS. Stored here unshifted for readability.
 */

//
// Applications
//
#define SYSSTATUS_MODID_APP         0       // Top layer application

//
// Library modules, start from 0x100
//
#define SYSSTATUS_MODID_OSRTL       0x100   // OS runtime
#define SYSSTATUS_MODID_CMPNT       0x101   // Component class
#define SYSSTATUS_MODID_ADAPTOR     0x102   // Adaptor class
#define SYSSTATUS_MODID_FILE        0x110   // File I/O class
#define SYSSTATUS_MODID_MPTRANS     0x111   // Multi processor data transfer class
#define SYSSTATUS_MODID_AUDIO       0x112   // Audio processor class
#define SYSSTATUS_MODID_VIDEO       0x113   // Video processor class
#define SYSSTATUS_MODID_MPEG        0x114   // Mpeg processor class
#define SYSSTATUS_MODID_USB         0x115   // USB

/**
 * Common codes shared by all modules. Encoded value 0 is the resting
 * Ready / No error state.
 */
#define STATUS_OK                   0
#define SYSSTATUS_OK                0       // System status normal
#define SYSSTATUS_NOERROR           0
#define SYSSTATUS_SUCCESS           0
#define SYSSTATUS_READY             0

#define SYSSTATUS_IDLE              0
#define SYSSTATUS_STARTED           1
#define SYSSTATUS_STOPED            2
#define SYSSTATUS_RUNNING           3
#define SYSSTATUS_SUSPENDED         4
#define SYSSTATUS_END               5
#define SYSSTATUS_PENDING           6       // Processing pending
#define SYSSTATUS_BLOCKED           7       // Waiting for an event
#define SYSSTATUS_TIMEOUT           8       // Wait time out
#define SYSSTATUS_CANCEL            9       // Operation canceled
#define SYSSTATUS_CONNECTED         10      // Connected state
#define SYSSTATUS_DISCONNECTED      11      // Disconnected state
#define SYSSTATUS_NOTFOUND          12
#define SYSSTATUS_FAILED            13      // Operation failed
#define SYSSTATUS_NOTSUPPORTED      14      // Operation not supported
#define SYSSTATUS_OUTMEM            15      // Out of memory
#define SYSSTATUS_FILECREATE        16      // Can not create file
#define SYSSTATUS_FILEOPEN          17      // Can not open file
#define SYSSTATUS_FILEREAD          18      // Can not read from file
#define SYSSTATUS_FILEWRITE         19      // Can not write to file
#define SYSSTATUS_FILENOTFOUND      20      // File not found
#define SYSSTATUS_THREADCREATE      21      // Can not create thread
#define SYSSTATUS_INVALIDPARM       22      // Invalid parameters
#define SYSSTATUS_OUTOFRANGE        23      // Out of range
#define SYSSTATUS_NOTINIT           24      // Not initialized yet

// Module specific codes start here
#define SYSSTATUS_CODE_USERSTART    0x100

typedef uint32_t		SysStatus_t;        // SysStatus return type
typedef SysStatus_t		Status_t;           // Status return type

#endif // __SYSSTATUSDEF_H__
