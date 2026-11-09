#ifndef PEI_NRF52_RTC_STATE_H_
#define PEI_NRF52_RTC_STATE_H_

#include "coredev/pei.h"

typedef struct {
	PeiDev_t *pPei;
	PeiEvent_t Event;
	bool EventWasEnabled;
} PeiNrf52RtcRawState_t;

extern PeiNrf52RtcRawState_t g_PeiNrf52RtcRaw[3][2];

#endif
