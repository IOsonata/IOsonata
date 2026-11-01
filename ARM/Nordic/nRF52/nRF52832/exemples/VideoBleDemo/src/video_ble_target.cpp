/* ATT storage for the Nordic HCI video host. */
#if defined(NRFXLIB_SDC)
#include "bluetooth/bt_att.h"

// GAP, GATT, the video characteristics and DIS exceed the default 2 KB pool.
alignas(BtAttDBEntry_t) static uint8_t s_AttDBMem[3072];

const BtAttDBMemCfg_t g_BtAttDBMemCfg = {
	s_AttDBMem, sizeof(s_AttDBMem)
};

#endif
