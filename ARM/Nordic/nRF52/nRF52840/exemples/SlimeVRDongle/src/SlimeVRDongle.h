
#ifndef __SLIMEVRDONGLE_H__
#define __SLIMEVRDONGLE_H__

#include "nrf_cli.h"

#define MAX_TRACKERS 			50
#define DETECTION_THRESHOLD 	16

#ifdef __cplusplus
extern "C" {
#endif

void UsbInit();
void init_cli();
uint32_t esb_init(void);
void EsbSetAddr(bool PairMode);
uint16_t AddTracker(uint64_t Addr);
void UpdateRecord();

extern nrf_cli_t const m_cli_cdc_acm;

#ifdef __cplusplus
}
#endif

#endif // __SLIMEVRDONGLE_H__
