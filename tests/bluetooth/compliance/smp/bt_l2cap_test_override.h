#ifndef BT_L2CAP_TEST_OVERRIDE_H
#define BT_L2CAP_TEST_OVERRIDE_H

// bt_l2cap.h intentionally declares variable-length protocol payloads with a
// one-byte tail. The dual-host test receives complete SMP PDUs into fixed host
// storage, so provide the same wire layout with a bounded tail before the
// production headers are parsed. This affects only the host-test translation
// units built with -include; the library ABI and embedded targets are unchanged.
#ifndef __BT_L2CAP_H__
#define __BT_L2CAP_H__

#include <stdint.h>

#include "bluetooth/bt_att.h"

#define BT_L2CAP_CID_ATT       4
#define BT_L2CAP_CID_SIGNAL    5
#define BT_L2CAP_CID_SEC_MNGR  6
#define BT_L2CAP_TEST_SMP_CAPACITY 96

#pragma pack(push, 1)

typedef struct __Bt_L2Cap_Header {
	uint16_t Len;
	uint16_t Cid;
} BtL2CapHdr_t;

typedef struct __Bt_L2Cap_Smp {
	uint8_t Code;
	uint8_t Data[BT_L2CAP_TEST_SMP_CAPACITY - 1];
} BtL2CapSmp_t;

typedef struct __Bt_L2Cap_Pdu {
	BtL2CapHdr_t Hdr;
	union {
		BtAttReqRsp_t Att;
		BtL2CapSmp_t Smp;
	};
} BtL2CapPdu_t;

#pragma pack(pop)

#ifndef BT_HCI_DEVICE_T_DEFINED
#define BT_HCI_DEVICE_T_DEFINED
typedef struct __Bt_Hci_Device BtHciDevice_t;
#endif

#endif // __BT_L2CAP_H__
#endif // BT_L2CAP_TEST_OVERRIDE_H
