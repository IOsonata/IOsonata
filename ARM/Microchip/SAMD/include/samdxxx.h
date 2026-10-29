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

#endif
