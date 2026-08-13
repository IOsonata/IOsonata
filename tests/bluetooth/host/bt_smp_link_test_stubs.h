// Shared shape for the SMP transport capture. See bt_smp_link_test_stubs.cpp.

#ifndef BT_SMP_LINK_TEST_STUBS_H
#define BT_SMP_LINK_TEST_STUBS_H

#include <cstddef>
#include <cstdint>

#define SMP_TX_CAPTURE_MAX		8

struct SmpTxPdu {
	uint16_t ConnHdl;
	size_t   Len;
	uint8_t  Data[72];
};

struct SmpTxCapture {
	// Total PDUs offered, which may exceed the number retained in Pdu.
	size_t   Count;
	SmpTxPdu Pdu[SMP_TX_CAPTURE_MAX];
};

extern SmpTxCapture g_SmpTx;

void SmpTxReset(void);

// BtSmpBondAdd is weak in bt_smp.cpp, and the SMP tests include that source, so
// a strong definition has to live in this separate translation unit. Counting
// only; behaviourally the same as the weak no-op it replaces.
extern int g_SmpBondAddCount;

void SmpBondAddReset(void);

#endif // BT_SMP_LINK_TEST_STUBS_H
