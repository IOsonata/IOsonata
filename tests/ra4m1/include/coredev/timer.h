/* REDUCED TEST CONTRACT. Production uses include/coredev/timer.h unchanged. */
#ifndef RA4M1_TEST_TIMER_H
#define RA4M1_TEST_TIMER_H
#include <stdint.h>
#include <stdbool.h>
typedef enum {TIMER_CLKSRC_DEFAULT,TIMER_CLKSRC_LFRC,TIMER_CLKSRC_HFRC,
 TIMER_CLKSRC_LFXTAL,TIMER_CLKSRC_HFXTAL,TIMER_CLKSRC_EXT} TIMER_CLKSRC;
typedef enum {TIMER_TRIG_TYPE_SINGLE,TIMER_TRIG_TYPE_CONTINUOUS} TIMER_TRIG_TYPE;
typedef enum {TIMER_EXTTRIG_SENSE_DISABLE,TIMER_EXTTRIG_SENSE_LOW_TRANSITION,
 TIMER_EXTTRIG_SENSE_HIGH_TRANSITION,TIMER_EXTTRIG_SENSE_TOGGLE} TIMER_EXTTRIG_SENSE;
#define TIMER_EVT_TICK (1U<<0)
#define TIMER_EVT_COUNTER_OVR (1U<<1)
#define TIMER_EVT_EXTTRIG (1U<<2)
#define TIMER_EVT_TRIGGER(n) (1U<<((n)+3))
typedef struct __Timer_Device TimerDev_t;
typedef void (*TimerEvtHandler_t)(TimerDev_t *,uint32_t);
typedef void (*TimerTrigEvtHandler_t)(TimerDev_t *,int,void *);
#pragma pack(push,4)
typedef struct {TIMER_TRIG_TYPE Type;uint64_t nsPeriod;TimerTrigEvtHandler_t Handler;void *pContext;} TimerTrig_t;
typedef struct {int DevNo;TIMER_CLKSRC ClkSrc;uint32_t Freq;int IntPrio;TimerEvtHandler_t EvtHandler;bool bTickInt;} TimerCfg_t;
struct __Timer_Device {
 int DevNo;uint32_t Freq;uint64_t nsPeriod,Rollover;uint32_t LastCount;
 TimerEvtHandler_t EvtHandler;void *pObj;
 void (*Disable)(TimerDev_t *);bool (*Enable)(TimerDev_t *);void (*Reset)(TimerDev_t *);
 uint64_t (*GetTickCount)(TimerDev_t *);uint32_t (*SetFrequency)(TimerDev_t *,uint32_t);
 int (*GetMaxTrigger)(TimerDev_t *);int (*FindAvailTrigger)(TimerDev_t *);
 void (*DisableTrigger)(TimerDev_t *,int);
 uint64_t (*EnableTrigger)(TimerDev_t *,int,uint64_t,TIMER_TRIG_TYPE,TimerTrigEvtHandler_t,void *);
 void (*DisableExtTrigger)(TimerDev_t *);
 bool (*EnableExtTrigger)(TimerDev_t *,int,TIMER_EXTTRIG_SENSE);
};
#pragma pack(pop)
#ifdef __cplusplus
extern "C" {
#endif
bool TimerInit(TimerDev_t *,const TimerCfg_t *);
int TimerGetLowFreqDevCount(void);int TimerGetHighFreqDevCount(void);int TimerGetHighFreqDevNo(void);
#ifdef __cplusplus
}
#endif
#endif
