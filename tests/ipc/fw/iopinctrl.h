#ifndef HOST_IOPINCTRL_H
#define HOST_IOPINCTRL_H

#ifdef __cplusplus
extern "C" {
#endif
void HostPin(int Port, int Pin, int Level);
void HostPinToggle(int Port, int Pin);
#ifdef __cplusplus
}
#endif

static inline void IOPinSet(int Port, int Pin) { HostPin(Port, Pin, 1); }
static inline void IOPinClear(int Port, int Pin) { HostPin(Port, Pin, 0); }
static inline void IOPinToggle(int Port, int Pin) { HostPinToggle(Port, Pin); }

#endif
