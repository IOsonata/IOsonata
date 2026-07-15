/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 *
 * BA414e PKE microcode required by nRF54L15. Copied from Nordic sdk-nrf
 * subsys/nrf_security/src/drivers/cracen/common/src/cracen/hardware/
 * microcode_binary.h. Keep the original license with this data.
 */
#ifndef __CRACEN_BA414E_UCODE_H__
#define __CRACEN_BA414E_UCODE_H__

#include <stdint.h>

static const uint32_t s_CracenBa414eUcode[] = {
#include "cracen_ba414e_ucode_00.inc"
#include "cracen_ba414e_ucode_01.inc"
#include "cracen_ba414e_ucode_02.inc"
#include "cracen_ba414e_ucode_03.inc"
#include "cracen_ba414e_ucode_04.inc"
#include "cracen_ba414e_ucode_05.inc"
#include "cracen_ba414e_ucode_06.inc"
#include "cracen_ba414e_ucode_07.inc"
#include "cracen_ba414e_ucode_08.inc"
#include "cracen_ba414e_ucode_09.inc"
#include "cracen_ba414e_ucode_10.inc"
};

#define CRACEN_BA414E_UCODE_WORDS \
	(sizeof(s_CracenBa414eUcode) / sizeof(s_CracenBa414eUcode[0]))

static_assert(sizeof(s_CracenBa414eUcode) <= 5120U,
			  "BA414e microcode exceeds nRF54L15 PKE code RAM");

#endif // __CRACEN_BA414E_UCODE_H__
