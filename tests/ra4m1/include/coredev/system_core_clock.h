/* TEST SHIM: generic IOsonata clock contract subset, NOT a production header. */
#ifndef TEST_CLOCK_H
#define TEST_CLOCK_H
#include <stdbool.h>
#include <stdint.h>
typedef enum { OSC_TYPE_RC, OSC_TYPE_XTAL, OSC_TYPE_TCXO } OSC_TYPE;
typedef struct { OSC_TYPE Type; uint32_t Freq, Accuracy, LoadCap; } OscDesc_t;
typedef struct { OscDesc_t CoreOsc, LowPwrOsc; bool bUSBClk; } McuOsc_t;
extern McuOsc_t g_McuOsc;
bool SystemCoreClockSelect(OSC_TYPE, uint32_t);
bool SystemLowFreqClockSelect(OSC_TYPE, uint32_t);
uint32_t SystemCoreClockGet(void);
uint32_t SystemPeriphClockGet(int);
uint32_t SystemPeriphClockSet(int, uint32_t);
void SystemOscInit(void);
#endif
