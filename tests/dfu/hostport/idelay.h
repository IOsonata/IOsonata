#ifndef __IDELAY_H__
#define __IDELAY_H__

#include <stdint.h>

// Host build: delays do not wait.
static inline void usDelay(uint32_t) {}
static inline void msDelay(uint32_t) {}

#endif
