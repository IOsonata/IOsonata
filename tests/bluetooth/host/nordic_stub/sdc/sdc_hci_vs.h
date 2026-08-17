// Host stand-in for the vendor specific command header. The controller setup
// uses one command from it, to widen the connection event length.
#ifndef __SDC_HCI_VS_STUB_H__
#define __SDC_HCI_VS_STUB_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	uint32_t event_length_us;
} sdc_hci_cmd_vs_event_length_set_t;

uint8_t sdc_hci_cmd_vs_event_length_set(
		const sdc_hci_cmd_vs_event_length_set_t *p_params);

#ifdef __cplusplus
}
#endif

#endif // __SDC_HCI_VS_STUB_H__
