#ifndef HOST_IDELAY_H
#define HOST_IDELAY_H

#include <stdint.h>

/* A delay is the time the other core gets to run. */
#ifdef __cplusplus
extern "C" {
#endif
void HostIdle(uint32_t Us);
#ifdef __cplusplus
}
#endif

static inline void usDelay(uint32_t us) { HostIdle(us); }
static inline void msDelay(uint32_t ms) { HostIdle(ms * 1000U); }

#endif
