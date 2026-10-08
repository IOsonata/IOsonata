/* REDUCED TEST CONTRACT, not a production header. Values/signatures copied
 * from include/coredev/iopincfg.h at eef99dc8. Production IOC does not use it.
 */
#ifndef TEST_IOPINCFG_H
#define TEST_IOPINCFG_H
#include <stdint.h>
#include <stdbool.h>
#include "coredev/interrupt.h"
#define IOPINOP_GPIO 0
#define IOPINOP_FUNC0 1
#define IOPINOP_FUNC30 31
#define IOPINOP_FUNC31 32
#define IOPINOP_FUNC3 4
#define IOPINOP_FUNC4 5
#define IOPINOP_FUNC6 7
#define IOPINOP_FUNC12 13
#define IOPINOP_FUNC15 16
#define IOPINOP_FUNC17 18
#define IOPINOP_FUNC18 19
typedef enum {IOPINRES_NONE,IOPINRES_PULLUP,IOPINRES_PULLDOWN,IOPINRES_FOLLOW} IOPINRES;
typedef enum {IOPINDIR_INPUT,IOPINDIR_OUTPUT,IOPINDIR_BI} IOPINDIR;
typedef enum {IOPINTYPE_NORMAL,IOPINTYPE_OPENDRAIN} IOPINTYPE;
typedef enum {IOPINSENSE_DISABLE,IOPINSENSE_LOW_TRANSITION,IOPINSENSE_HIGH_TRANSITION,IOPINSENSE_TOGGLE} IOPINSENSE;
typedef enum {IOPINSTRENGTH_REGULAR,IOPINSTRENGTH_STRONG} IOPINSTRENGTH;
typedef enum {IOPINSPEED_LOW,IOPINSPEED_MEDIUM,IOPINSPEED_HIGH,IOPINSPEED_TURBO} IOPINSPEED;
#pragma pack(push,4)
typedef struct {int PortNo,PinNo,PinOp; IOPINDIR PinDir; IOPINRES Res; IOPINTYPE Type;} IOPINCFG;
#pragma pack(pop)
typedef void (*IOPinEvtHandler_t)(int,void *);
#ifdef __cplusplus
extern "C" {
#endif
void IOPinConfig(int,int,int,IOPINDIR,IOPINRES,IOPINTYPE);
static inline void IOPinCfg(const IOPINCFG *p, int n) {
 for (int i=0;i<n;++i) IOPinConfig(p[i].PortNo,p[i].PinNo,p[i].PinOp,p[i].PinDir,p[i].Res,p[i].Type);
}
void IOPinDisable(int,int);
void IOPinDisableInterrupt(int);
bool IOPinEnableInterrupt(int,int,uint32_t,uint32_t,IOPINSENSE,IOPinEvtHandler_t,void *);
int IOPinAllocateInterrupt(int,int,int,IOPINSENSE,IOPinEvtHandler_t,void *);
void IOPinSetSense(int,int,IOPINSENSE);
void IOPinSetStrength(int,int,IOPINSTRENGTH);
void IOPinSetSpeed(int,int,IOPINSPEED);
#ifdef __cplusplus
}
#endif
#endif
