// Host stand-in for the HCI transport entry points.
#ifndef __SDC_HCI_STUB_H__
#define __SDC_HCI_STUB_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	SDC_HCI_MSG_TYPE_NONE = 0,
	SDC_HCI_MSG_TYPE_EVT = 1,
	SDC_HCI_MSG_TYPE_DATA = 2,
} sdc_hci_msg_type_t;

int32_t sdc_hci_get(uint8_t *p_msg_out, uint8_t *p_msg_type_out);
int32_t sdc_hci_data_put(uint8_t *p_data);

#ifdef __cplusplus
}
#endif

#endif // __SDC_HCI_STUB_H__
