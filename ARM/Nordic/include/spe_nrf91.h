/**-------------------------------------------------------------------------
@file	spe_nrf91.h

@brief	nRF91 secure stage: start a non secure application

On nRF91 the core comes out of reset secure, with every flash and RAM region,
peripheral, GPIO pin and DPPI channel secure. An application that uses the
modem runs non secure: the Modem library requires it. The secure stage is a
small image at 0 that gives the non secure side its memory and peripherals
through the SPU and branches to the non secure application. It replaces the
secure firmware (TF-M) of the nRF Connect SDK.

The memory split comes from the TrustZone layout of the linker scripts
(tz_layout_nrf9160.ld, tz_layout_nrf9120.ld): every flash and RAM region from
__tz_ns_flash_start and __tz_ns_ram_start to the end of memory becomes non
secure. The secure stage links with nrf91xx_xxaa_spe.ld, the application
with nrf91xx_xxaa_ns.ld and a non secure configuration of the library
(NRF_TRUSTZONE_NONSECURE).

Every peripheral whose security can be chosen becomes non secure, with its
interrupt targeting the non secure side and its DMA non secure, unless the
configuration keeps it secure. The interrupts of the peripherals that are
always non secure (GPIOTE1, FPU) target the non secure side too. CRYPTOCELL
and the SPU stay secure, the SPU does not let them be changed. All GPIO pins
and DPPI channels become non secure unless kept secure the same way.

BusFault, HardFault and NMI are given to the non secure side, so the
application handles its own faults. SecureFault is enabled and stays secure.

The secure stage is built with -mcmse (the secure configurations of the
library). Secure services callable from the non secure side are not
provided yet; the layout reserves their area.

@author	Hoang Nguyen Hoan
@date	Oct. 6, 2026

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
#ifndef __SPE_NRF91_H__
#define __SPE_NRF91_H__

#include <stdint.h>
#include <stdbool.h>

/** @addtogroup TrustZone
  * @{
  */

#pragma pack(push, 4)

/// What stays secure when the non secure application starts. A zeroed
/// configuration, or none, keeps nothing secure besides what the SPU fixes.
typedef struct __nRF91_Spe_Config {
	const uint8_t *pSecPeriph;	//!< SPU peripheral ids kept secure (PERIPHID index,
								//!< which is also the interrupt number), NULL for none
	int NbSecPeriph;			//!< Number of entries in pSecPeriph
	uint32_t SecGpio;			//!< P0 pins kept secure, bit n for pin n
	uint32_t SecDppi;			//!< DPPI channels kept secure, bit n for channel n
} nRF91SpeCfg_t;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Give the non secure side its memory and peripherals and start the
 * 			non secure application.
 *
 * The application is at __tz_ns_flash_start. Its vector table is checked
 * first: the stack pointer must be in non secure RAM, at most
 * __tz_ns_ram_end, and the reset handler a Thumb address in non secure
 * flash. Nothing is changed when it is not.
 *
 * Call it from the secure stage with interrupts unused. Every secure
 * interrupt is disabled and cleared before the branch.
 *
 * @param	pCfg : What stays secure, NULL for the default
 *
 * @return	false - no valid non secure application. Does not return otherwise.
 */
bool nRF91SpeStart(const nRF91SpeCfg_t * const pCfg);

#ifdef __cplusplus
}
#endif

/** @} End of group TrustZone */

#endif
