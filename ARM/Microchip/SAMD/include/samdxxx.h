/**-------------------------------------------------------------------------
@file	samdxxx.h

@brief	Microchip SAM D11/D21 CMSIS device selection

@license
MIT License
Copyright (c) 2026 I-SYST inc. All rights reserved.
----------------------------------------------------------------------------*/
#ifndef __SAMDXXX_H__
#define __SAMDXXX_H__

#if defined(__SAMD11D14AM__) && !defined(__SAMD21J18A__)
#include "samd11d14am.h"
#elif defined(__SAMD21J18A__) && !defined(__SAMD11D14AM__)
#include "samd21j18a.h"
#else
#error Select one Microchip SAM D device in the managed project
#endif

// NVIC priority levels 0 (most urgent) to SAMD_IRQ_PRIO_MAX (3). A less
// urgent level asked, as IRQ_PRIO_LOW (5) or IRQ_PRIO_LOWEST (7), is the
// lowest level, as on the LPC11 port.
#define SAMD_IRQ_PRIO_MAX		((1 << __NVIC_PRIO_BITS) - 1)

static inline uint32_t SamdIrqPrio(int Prio)
{
	return Prio > SAMD_IRQ_PRIO_MAX ? (uint32_t)SAMD_IRQ_PRIO_MAX : (uint32_t)Prio;
}

#endif
