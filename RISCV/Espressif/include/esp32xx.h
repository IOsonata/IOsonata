/**-------------------------------------------------------------------------
@file	esp32xx.h

@brief	Top-level chip-family dispatcher for Espressif RISC-V parts.

Picks the correct chip-specific header based on the ESPnnnn or
CONFIG_IDF_TARGET_ESPnnnn build symbol.  Each chip header is a
self-contained CMSIS-style device header that defines:

  1. Its own ESP32cn_xxx_BASE / ESP32cn_xxx_COUNT symbols.
  2. The chip-agnostic ESP32_xxx aliases that drivers code against.
  3. ESP32_HAS_xxx capability flags for optional peripherals.

Drivers include ONLY this file and reference ESP32_xxx names.

@author	Nguyen Hoan Hoang
@date	May 12, 2026

@license

MIT License

Copyright (c) 2026 I-SYST inc. All rights reserved.

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
#ifndef __ESP32XX_H__
#define __ESP32XX_H__

/* Accept ESP-IDF style target macros as input. */
#if defined(CONFIG_IDF_TARGET_ESP32C2) && !defined(ESP32C2)
#define ESP32C2 1
#endif
#if defined(CONFIG_IDF_TARGET_ESP32C3) && !defined(ESP32C3)
#define ESP32C3 1
#endif
#if defined(CONFIG_IDF_TARGET_ESP32C5) && !defined(ESP32C5)
#define ESP32C5 1
#endif
#if defined(CONFIG_IDF_TARGET_ESP32C6) && !defined(ESP32C6)
#define ESP32C6 1
#endif
#if defined(CONFIG_IDF_TARGET_ESP32C61) && !defined(ESP32C61)
#define ESP32C61 1
#endif
#if defined(CONFIG_IDF_TARGET_ESP32H2) && !defined(ESP32H2)
#define ESP32H2 1
#endif
#if defined(CONFIG_IDF_TARGET_ESP32H4) && !defined(ESP32H4)
#define ESP32H4 1
#endif
#if defined(CONFIG_IDF_TARGET_ESP32P4) && !defined(ESP32P4)
#define ESP32P4 1
#endif

/* Select chip-specific header. */
#if defined(ESP32C2)
#include "esp32c2.h"
#elif defined(ESP32C3)
#include "esp32c3.h"
#elif defined(ESP32C5)
#include "esp32c5.h"
#elif defined(ESP32C6)
#include "esp32c6.h"
#elif defined(ESP32C61)
#include "esp32c61.h"
#elif defined(ESP32H2)
#include "esp32h2.h"
#elif defined(ESP32H4)
#include "esp32h4.h"
#elif defined(ESP32P4)
#include "esp32p4.h"
#else
#error "No ESP32 RISC-V chip selected.  Define one of: ESP32C2, ESP32C3, ESP32C5, ESP32C6, ESP32C61, ESP32H2, ESP32H4, ESP32P4."
#endif

/* Convenience aliases preserved from earlier IOsonata headers. */
#define ESP32_PIN_MAX                       ESP32_GPIO_PIN_COUNT
#define ESP32_MAX_PIN                       ESP32_GPIO_PIN_MAX

#endif // __ESP32XX_H__
