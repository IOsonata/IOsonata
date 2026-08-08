/**-------------------------------------------------------------------------
@file	bt_smp_bond.h

@brief	Bluetooth SMP bond-table identity access.

@author	Hoang Nguyen Hoan
@date	Aug. 8, 2026

@license MIT, (c) 2026 I-SYST inc.
----------------------------------------------------------------------------*/
#ifndef __BT_SMP_BOND_H__
#define __BT_SMP_BOND_H__

#include <stdbool.h>
#include <stdint.h>

#ifndef BT_SMP_BOND_MAX
#define BT_SMP_BOND_MAX		8
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Read the peer identity material from one valid bond slot.
 *
 * Only the identity address and IRK needed for controller resolving-list
 * programming are exposed. LTK, CSRK, authentication state and CCCDs remain
 * private to the bond store.
 *
 * @param	Slot		Bond-table slot, 0 .. BT_SMP_BOND_MAX-1.
 * @param	pAddrType	out: peer identity address type (0 public, 1 random).
 * @param	Addr		out: peer identity address.
 * @param	Irk			out: peer identity resolving key.
 *
 * @return	true when Slot contains a valid bond with a nonzero peer IRK and
 *			valid identity address type. Outputs are cleared on failure when
 *			their pointers are valid.
 */
bool BtSmpBondIdentityGet(int Slot, uint8_t *pAddrType,
	uint8_t Addr[6], uint8_t Irk[16]);

/**
 * @brief	Notification after persistent bonds and local identity are loaded.
 *
 * The generic weak implementation returns true. A Bluetooth port can override
 * this to synchronize controller state that depends on the restored identities,
 * such as an HCI resolving list. It runs before BtSmpBondNvmInit returns.
 *
 * @return	true when dependent controller state is ready.
 */
bool BtSmpBondLoadComplete(void);

#ifdef __cplusplus
}
#endif

#endif // __BT_SMP_BOND_H__
