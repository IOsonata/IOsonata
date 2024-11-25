
#ifndef __SLIMEVRDONGLE_H__
#define __SLIMEVRDONGLE_H__

#define MAX_TRACKERS 			50
#define DETECTION_THRESHOLD 	16

#ifdef __cplusplus
extern "C" {
#endif

uint32_t esb_init(void);
void EsbSetAddr(bool PairMode);
uint16_t AddTracker(uint64_t Addr);

#ifdef __cplusplus
}
#endif

#endif // __SLIMEVRDONGLE_H__
