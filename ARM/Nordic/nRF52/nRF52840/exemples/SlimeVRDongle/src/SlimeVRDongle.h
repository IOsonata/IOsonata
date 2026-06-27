
#ifndef __SLIMEVRDONGLE_H__
#define __SLIMEVRDONGLE_H__

#include <stdint.h>

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

// Forward one 16 byte tracker record to the host HID IN endpoint. rssi is
// stored in byte 15 for packet types other than 1 and 4.
void hid_write_packet_n(uint8_t *data, int rssi);

#ifdef __cplusplus
}
#endif

#endif // __SLIMEVRDONGLE_H__
