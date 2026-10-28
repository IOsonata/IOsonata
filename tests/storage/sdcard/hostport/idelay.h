// Delays of the SD card test advance the time of the card model
#ifndef __IDELAY_H__
#define __IDELAY_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void ModelDelayUs(uint32_t Us);

#ifdef __cplusplus
}
#endif

static inline void usDelay(uint32_t Us)
{
	ModelDelayUs(Us);
}

static inline void msDelay(uint32_t Ms)
{
	ModelDelayUs(Ms * 1000U);
}

#endif
