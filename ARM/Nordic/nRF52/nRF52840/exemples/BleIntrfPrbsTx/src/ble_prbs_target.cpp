/* Target storage for the Nordic BLE PRBS transmitter. */
#if defined(NRFXLIB_SDC)
#include "bluetooth/bt_att.h"

// GAP, GATT, PRBS and Device Information require more than the default pool.
alignas(BtAttDBEntry_t) static uint8_t s_AttDBMem[3072];

extern "C" const BtAttDBMemCfg_t g_BtAttDBMemCfg = {
	s_AttDBMem,
	sizeof(s_AttDBMem)
};
#endif
