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
 default value returned when no error is present.

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
   SYSSTATUS_TYPE_RNT   = 0x00000000UL,     //!< Runtime state
   SYSSTATUS_TYPE_WRN   = 0x40000000UL,     //!< Warning
   SYSSTATUS_TYPE_ERR   = 0x80000000UL,     //!< Non fatal error
   SYSSTATUS_TYPE_FERR  = 0xf0000000UL      //!< Fatal error
} SYSSTATUS_TYPE;

/**
 * Reserve module Id here. Each value occupies the module field once shifted
 * by SYSSTATUS_MODID_POS. Stored here unshifted for readability.
 *
 * The 12 bit module space is range partitioned by layer so a handler can
 * route on a whole class of subsystem with a range test, or on a specific
 * module with an equality test.
 *
 *   0x000 - 0x0FF  Application
 *   0x100 - 0x1FF  Core drivers        (register level peripheral transport)
 *   0x200 - 0x2FF  Subsystems          (middleware over core drivers)
 *   0x300 - 0x3FF  Spec stacks         (standardized protocol stacks)
 *   0x400 - 0x4FF  OS runtime          (kernel subsystems)
 *   0x500 - 0xEFF  Reserved for growth
 *
 * A standardized stack sits in the stack range and rides a transport driver
 * in the core range. Such a module therefore appears twice, once for the
 * stack and once for its driver, so a stack level fault and a wire level
 * fault are distinct origins. USB and Bluetooth are the worked examples.
 */

// Range bounds, for class level routing
#define SYSSTATUS_MODID_APP_FIRST       0x000
#define SYSSTATUS_MODID_APP_LAST        0x0ff
#define SYSSTATUS_MODID_DRV_FIRST       0x100
#define SYSSTATUS_MODID_DRV_LAST        0x1ff
#define SYSSTATUS_MODID_SUBSYS_FIRST    0x200
#define SYSSTATUS_MODID_SUBSYS_LAST     0x2ff
#define SYSSTATUS_MODID_STACK_FIRST     0x300
#define SYSSTATUS_MODID_STACK_LAST      0x3ff
#define SYSSTATUS_MODID_OS_FIRST        0x400
#define SYSSTATUS_MODID_OS_LAST         0x4ff

//
// Application, 0x000 - 0x0FF
//
#define SYSSTATUS_MODID_APP         0x000   //!< Top layer application

//
// Core drivers, 0x100 - 0x1FF. Register level peripheral transport.
// Allocated per peripheral class, not per instance. The bus instance is a
// code field or context detail, not a module identity.
//
#define SYSSTATUS_MODID_UART        0x100   //!< UART driver
#define SYSSTATUS_MODID_SPI         0x101   //!< SPI driver
#define SYSSTATUS_MODID_I2C         0x102   //!< I2C driver
#define SYSSTATUS_MODID_I2S         0x103   //!< I2S driver
#define SYSSTATUS_MODID_TIMER       0x104   //!< Timer driver
#define SYSSTATUS_MODID_PWM         0x105   //!< PWM driver
#define SYSSTATUS_MODID_PDM         0x106   //!< PDM driver
#define SYSSTATUS_MODID_ADC         0x107   //!< ADC driver
#define SYSSTATUS_MODID_GPIO        0x108   //!< GPIO / pin driver
#define SYSSTATUS_MODID_DMA         0x109   //!< DMA driver
#define SYSSTATUS_MODID_FLASHDRV    0x10a   //!< On chip flash / NVM driver
#define SYSSTATUS_MODID_USBPHY      0x10b   //!< USB wire transport driver
#define SYSSTATUS_MODID_MACPHY      0x10c   //!< Ethernet MAC + PHY transport driver
#define SYSSTATUS_MODID_RFPHY       0x10d   //!< Radio transport driver, shared PHY for
                                            // all RF protocols (BLE, 802.15.4, proprietary)

//
// Subsystems, 0x200 - 0x2FF. Middleware over core drivers.
//
#define SYSSTATUS_MODID_SENSOR      0x200   //!< Sensor framework
#define SYSSTATUS_MODID_FUSION      0x201   //!< Sensor fusion
#define SYSSTATUS_MODID_PWRMGNT     0x202   //!< Power management
#define SYSSTATUS_MODID_DISPLAY     0x203   //!< Display
#define SYSSTATUS_MODID_AUDIO       0x204   //!< Audio processor
#define SYSSTATUS_MODID_VIDEO       0x205   //!< Video processor
#define SYSSTATUS_MODID_STORAGE     0x206   //!< Storage / disk I/O
#define SYSSTATUS_MODID_FILE        0x207   //!< File system
#define SYSSTATUS_MODID_MISCDEV     0x208   //!< Misc devices (led, button, buzzer)
#define SYSSTATUS_MODID_CONVERTER   0x209   //!< External converters (adc, dac)

//
// Spec stacks, 0x300 - 0x3FF. Standardized protocol stacks, each riding a
// transport driver in the core range.
//
#define SYSSTATUS_MODID_USB         0x300   //!< USB stack (rides SYSSTATUS_MODID_USBPHY)
#define SYSSTATUS_MODID_BLE         0x301   //!< Bluetooth host (rides SYSSTATUS_MODID_RFPHY)
#define SYSSTATUS_MODID_TCPIP       0x302   //!< TCP/IP stack
#define SYSSTATUS_MODID_WIFI        0x303   //!< WiFi / 802.11 stack (rides SYSSTATUS_MODID_RFPHY)
#define SYSSTATUS_MODID_THREAD      0x304   //!< Thread / Matter stack (rides SYSSTATUS_MODID_RFPHY)
#define SYSSTATUS_MODID_ZIGBEE      0x305   //!< Zigbee stack (rides SYSSTATUS_MODID_RFPHY)

//
// OS runtime, 0x400 - 0x4FF. Kernel subsystems.
//
#define SYSSTATUS_MODID_OSRTL       0x400   //!< OS runtime, general
#define SYSSTATUS_MODID_SCHED       0x401   //!< Scheduler
#define SYSSTATUS_MODID_IPC         0x402   //!< IPC / synchronization
#define SYSSTATUS_MODID_HEAP        0x403   //!< Memory / heap

/**
 * Common codes shared by all modules. These are pre encoded with their
 * natural type so a bare constant is self classifying, e.g.
 * StatusIsError(SYSSTATUS_FAILED) is true without first calling
 * StatusEncode. The module field of a common code is 0 (APP / shared); a
 * subsystem that wants its own variant encodes the same code with its own
 * module ID.
 *
 * Runtime state codes use SYSSTATUS_TYPE_RNT (0) so they stay numerically
 * clean. Error codes carry SYSSTATUS_TYPE_ERR.
 */

// Ready / No error
#define STATUS_OK                   0
#define SYSSTATUS_OK                0       //!< System status normal
#define SYSSTATUS_NOERROR           0
#define SYSSTATUS_SUCCESS           0
#define SYSSTATUS_READY             0

//
// Runtime state codes, type RNT. State machine values, not errors.
//
#define SYSSTATUS_IDLE              (SYSSTATUS_TYPE_RNT | 0)
#define SYSSTATUS_STARTED           (SYSSTATUS_TYPE_RNT | 1)
#define SYSSTATUS_STOPED            (SYSSTATUS_TYPE_RNT | 2)
#define SYSSTATUS_RUNNING           (SYSSTATUS_TYPE_RNT | 3)
#define SYSSTATUS_SUSPENDED         (SYSSTATUS_TYPE_RNT | 4)
#define SYSSTATUS_END               (SYSSTATUS_TYPE_RNT | 5)
#define SYSSTATUS_PENDING           (SYSSTATUS_TYPE_RNT | 6)   //!< Processing pending
#define SYSSTATUS_BLOCKED           (SYSSTATUS_TYPE_RNT | 7)   //!< Waiting for an event
#define SYSSTATUS_CONNECTED         (SYSSTATUS_TYPE_RNT | 10)  //!< Connected state
#define SYSSTATUS_DISCONNECTED      (SYSSTATUS_TYPE_RNT | 11)  //!< Disconnected state

//
// Error codes, type ERR. Pre encoded, directly testable.
// POSIX / ISO C names are provided as aliases onto these stable numbers.
// Note: status numbers are this standard's own and are stable across
// platforms. They are not the platform errno integer values, only the
// errno names are reused as a familiar vocabulary.
//
#define SYSSTATUS_TIMEOUT           (SYSSTATUS_TYPE_ERR | 8)   //!< Wait time out
#define SYSSTATUS_CANCEL            (SYSSTATUS_TYPE_ERR | 9)   //!< Operation canceled
#define SYSSTATUS_NOTFOUND          (SYSSTATUS_TYPE_ERR | 12)  //!< Not found
#define SYSSTATUS_FAILED            (SYSSTATUS_TYPE_ERR | 13)  //!< Operation failed
#define SYSSTATUS_NOTSUPPORTED      (SYSSTATUS_TYPE_ERR | 14)  //!< Operation not supported
#define SYSSTATUS_OUTMEM            (SYSSTATUS_TYPE_ERR | 15)  //!< Out of memory
#define SYSSTATUS_FILECREATE        (SYSSTATUS_TYPE_ERR | 16)  //!< Can not create file
#define SYSSTATUS_FILEOPEN          (SYSSTATUS_TYPE_ERR | 17)  //!< Can not open file
#define SYSSTATUS_FILEREAD          (SYSSTATUS_TYPE_ERR | 18)  //!< Can not read from file
#define SYSSTATUS_FILEWRITE         (SYSSTATUS_TYPE_ERR | 19)  //!< Can not write to file
#define SYSSTATUS_FILENOTFOUND      (SYSSTATUS_TYPE_ERR | 20)  //!< File not found
#define SYSSTATUS_THREADCREATE      (SYSSTATUS_TYPE_ERR | 21)  //!< Can not create thread
#define SYSSTATUS_INVALIDPARM       (SYSSTATUS_TYPE_ERR | 22)  //!< Invalid parameters
#define SYSSTATUS_OUTOFRANGE        (SYSSTATUS_TYPE_ERR | 23)  //!< Out of range
#define SYSSTATUS_NOTINIT           (SYSSTATUS_TYPE_ERR | 24)  //!< Not initialized yet

// Additional POSIX / ISO derived common errors, this standard's numbering
#define SYSSTATUS_PERM              (SYSSTATUS_TYPE_ERR | 25)  //!< Operation not permitted
#define SYSSTATUS_ACCESS            (SYSSTATUS_TYPE_ERR | 26)  //!< Permission denied
#define SYSSTATUS_BUSY              (SYSSTATUS_TYPE_ERR | 27)  //!< Resource busy
#define SYSSTATUS_EXIST             (SYSSTATUS_TYPE_ERR | 28)  //!< Already exists
#define SYSSTATUS_NODEV             (SYSSTATUS_TYPE_ERR | 29)  //!< No such device
#define SYSSTATUS_NOSPACE           (SYSSTATUS_TYPE_ERR | 30)  //!< No space left
#define SYSSTATUS_AGAIN             (SYSSTATUS_TYPE_ERR | 31)  //!< Try again / would block
#define SYSSTATUS_INTR              (SYSSTATUS_TYPE_ERR | 32)  //!< Interrupted
#define SYSSTATUS_BADF              (SYSSTATUS_TYPE_ERR | 33)  //!< Bad descriptor / handle
#define SYSSTATUS_IO                (SYSSTATUS_TYPE_ERR | 34)  //!< I/O error
#define SYSSTATUS_NOTCONN           (SYSSTATUS_TYPE_ERR | 35)  //!< Not connected
#define SYSSTATUS_CONNREFUSED       (SYSSTATUS_TYPE_ERR | 36)  //!< Connection refused
#define SYSSTATUS_CONNRESET         (SYSSTATUS_TYPE_ERR | 37)  //!< Connection reset
#define SYSSTATUS_ADDRINUSE         (SYSSTATUS_TYPE_ERR | 38)  //!< Address in use
#define SYSSTATUS_OVERFLOW          (SYSSTATUS_TYPE_ERR | 39)  //!< Value overflow
#define SYSSTATUS_NOSYS             (SYSSTATUS_TYPE_ERR | 40)  //!< Not implemented
#define SYSSTATUS_NAMETOOLONG       (SYSSTATUS_TYPE_ERR | 41)  //!< Name too long
#define SYSSTATUS_DOM               (SYSSTATUS_TYPE_ERR | 42)  //!< Domain error (ISO C)
#define SYSSTATUS_ILSEQ             (SYSSTATUS_TYPE_ERR | 43)  //!< Illegal byte sequence (ISO C)

// POSIX / ISO C name aliases onto the common errors above
#define SYSSTATUS_ETIMEDOUT         SYSSTATUS_TIMEOUT
#define SYSSTATUS_ECANCELED         SYSSTATUS_CANCEL
#define SYSSTATUS_ENOENT            SYSSTATUS_NOTFOUND
#define SYSSTATUS_ENOTSUP           SYSSTATUS_NOTSUPPORTED
#define SYSSTATUS_ENOMEM            SYSSTATUS_OUTMEM
#define SYSSTATUS_EINVAL            SYSSTATUS_INVALIDPARM
#define SYSSTATUS_ERANGE            SYSSTATUS_OUTOFRANGE
#define SYSSTATUS_EPERM             SYSSTATUS_PERM
#define SYSSTATUS_EACCES            SYSSTATUS_ACCESS
#define SYSSTATUS_EBUSY             SYSSTATUS_BUSY
#define SYSSTATUS_EEXIST            SYSSTATUS_EXIST
#define SYSSTATUS_ENODEV            SYSSTATUS_NODEV
#define SYSSTATUS_ENOSPC            SYSSTATUS_NOSPACE
#define SYSSTATUS_EAGAIN            SYSSTATUS_AGAIN
#define SYSSTATUS_EINTR             SYSSTATUS_INTR
#define SYSSTATUS_EBADF             SYSSTATUS_BADF
#define SYSSTATUS_EIO               SYSSTATUS_IO
#define SYSSTATUS_ENOTCONN          SYSSTATUS_NOTCONN
#define SYSSTATUS_ECONNREFUSED      SYSSTATUS_CONNREFUSED
#define SYSSTATUS_ECONNRESET        SYSSTATUS_CONNRESET
#define SYSSTATUS_EADDRINUSE        SYSSTATUS_ADDRINUSE
#define SYSSTATUS_EOVERFLOW         SYSSTATUS_OVERFLOW
#define SYSSTATUS_ENOSYS            SYSSTATUS_NOSYS
#define SYSSTATUS_ENAMETOOLONG      SYSSTATUS_NAMETOOLONG
#define SYSSTATUS_EDOM              SYSSTATUS_DOM
#define SYSSTATUS_EILSEQ            SYSSTATUS_ILSEQ

// Module specific codes start here
#define SYSSTATUS_CODE_USERSTART    0x100

typedef uint32_t		SysStatus_t;        //!< SysStatus return type
typedef SysStatus_t		Status_t;           //!< Status return type

#endif // __SYSSTATUSDEF_H__
