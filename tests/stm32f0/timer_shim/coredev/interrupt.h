#pragma once
extern unsigned irqState;
static inline unsigned DisableInterrupt() { unsigned old=irqState; irqState=1; return old; }
static inline void EnableInterrupt(unsigned old) { irqState=old; }
