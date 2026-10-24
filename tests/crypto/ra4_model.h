#pragma once
#include <stdint.h>
struct RaSystem { uint16_t PRCR; };
struct RaMstp { uint32_t MSTPCRC; };
extern RaSystem g_System;
extern RaMstp g_Mstp;
#define R_SYSTEM (&g_System)
#define R_MSTP (&g_Mstp)
#define R_SYSTEM_PRCR_PRC1_Msk 2U
