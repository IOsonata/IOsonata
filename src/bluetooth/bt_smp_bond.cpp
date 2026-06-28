/**-------------------------------------------------------------------------
@file	bt_smp_bond.cpp

@brief	Simple bond store for the SMP layer.

Persists the keys produced by a successful pairing so a later reconnection
can re-encrypt the link with the stored LTK instead of pairing again. The
store overrides the weak BtSmpPairingComplete and BtSmpBondLtkLookup hooks in
bt_smp.cpp.

This implementation keeps the bond table in RAM. A platform that wants the
bonds to survive a reset can replace BtSmpBondSave / BtSmpBondLoad (declared
weak below) with a flash-backed version; the lookup and capture logic above
them does not change.

For LE Secure Connections the LTK is derived (EDIV and Rand are zero), so a
bond is matched by the peer address used on the link. Legacy bonds carry the
distributed EDIV/Rand and are matched on those first.

@author	Hoan
-------------------------------------------------------------------------*/
#include <string.h>

#include "bluetooth/bt_smp.h"
#include "bluetooth/bt_peer.h"
#include "bluetooth/bt_dev.h"

#ifndef BT_SMP_BOND_MAX
#define BT_SMP_BOND_MAX		8
#endif

typedef struct __Bt_Smp_Bond {
	bool		bValid;			//!< Slot in use
	uint8_t		PeerAddrType;	//!< Peer address type used on the link
	uint8_t		PeerAddr[6];	//!< Peer address used on the link (match key)
	BtSmpKeys_t	Keys;			//!< Stored key set (LTK, IRK, identity, ...)
	uint8_t		NbCccd;			//!< Number of persisted CCCD entries
	BtGattCccdState_t Cccd[BT_GATT_CCCD_STATE_MAX];	//!< CCCD set persisted for this bond (spec: bonded clients keep CCCD across connections)
} BtSmpBond_t;

static BtSmpBond_t s_BtSmpBondTable[BT_SMP_BOND_MAX];

// Weak persistence hooks. Default is RAM only. A flash-backed port overrides
// these to mirror the table to non-volatile storage.
//
// BtSmpBondSave  : called by the generic layer whenever a slot changes, so the
//                  platform can write that one slot to NVM.
// BtSmpBondLoad  : called once by the generic layer at init (BtSmpBondInit), so
//                  the platform can read NVM back and repopulate the RAM table
//                  through BtSmpBondRestore.
// BtSmpBondErase : called by BtSmpBondClearAll, so the platform can wipe NVM and
//                  stale bonds do not reappear on the next BtSmpBondLoad.
__attribute__((weak)) void BtSmpBondSave(int Slot, const void *pBond, size_t Len)
{
	(void)Slot;
	(void)pBond;
	(void)Len;
}

__attribute__((weak)) void BtSmpBondLoad(void)
{
}

__attribute__((weak)) void BtSmpBondErase(void)
{
}

// Number of bond slots the table holds. Lets a platform size its NVM region and
// iterate slots without knowing BT_SMP_BOND_MAX at compile time.
int BtSmpBondSlotCount(void)
{
	return BT_SMP_BOND_MAX;
}

// Size of one stored bond record. The platform persists/loads opaque blobs of
// this size; it does not need the BtSmpBond_t layout.
size_t BtSmpBondRecordSize(void)
{
	return sizeof(BtSmpBond_t);
}

// Restore one slot from a previously saved blob. Called by a platform
// BtSmpBondLoad override for each persisted slot. pBond must point at Len bytes
// previously produced by BtSmpBondSave for that slot. A record whose bValid is
// false is ignored, so erased slots stay empty.
void BtSmpBondRestore(int Slot, const void *pBond, size_t Len)
{
	if (Slot < 0 || Slot >= BT_SMP_BOND_MAX || pBond == nullptr ||
		Len != sizeof(BtSmpBond_t))
	{
		return;
	}

	const BtSmpBond_t *p = (const BtSmpBond_t *)pBond;
	if (!p->bValid)
	{
		return;
	}

	memcpy(&s_BtSmpBondTable[Slot], p, sizeof(BtSmpBond_t));
}

// Find a bond slot matching the peer address, or -1.
static int BtSmpBondFindByAddr(uint8_t AddrType, const uint8_t Addr[6])
{
	for (int i = 0; i < BT_SMP_BOND_MAX; i++)
	{
		if (s_BtSmpBondTable[i].bValid &&
			s_BtSmpBondTable[i].PeerAddrType == AddrType &&
			memcmp(s_BtSmpBondTable[i].PeerAddr, Addr, 6) == 0)
		{
			return i;
		}
	}
	return -1;
}

// Pick a free slot, or recycle the first one (simple wrap).
static int BtSmpBondAllocSlot(void)
{
	for (int i = 0; i < BT_SMP_BOND_MAX; i++)
	{
		if (!s_BtSmpBondTable[i].bValid)
		{
			return i;
		}
	}
	return 0;
}

// Capture keys on a successful pairing. Called from the SMP core right before
// the application BtSmpPairingComplete notification, so bonding works
// regardless of whether the application overrides that callback.
void BtSmpBondAdd(uint16_t ConnHdl, const BtSmpKeys_t *pKeys)
{
	if (pKeys == nullptr || !pKeys->bValid)
	{
		return;
	}

	// The match key on reconnection must be the address the lookup uses,
	// which is the connection address of the link (see BtSmpBondLtkLookup).
	// The distributed identity address is kept inside Keys as data only - it
	// must NOT become the slot key, or a peer that connects with a resolvable
	// private address (e.g. iOS) will never match its bond on reconnect.
	uint8_t addrType;
	uint8_t addr[6];

	BtDevice_t *pPeer = BtPeerFindByHdl(ConnHdl);
	if (pPeer == nullptr)
	{
		return;
	}
	addrType = pPeer->Conn.PeerAddrType;
	memcpy(addr, pPeer->Conn.PeerAddr, 6);

	int slot = BtSmpBondFindByAddr(addrType, addr);
	if (slot < 0)
	{
		slot = BtSmpBondAllocSlot();
	}

	s_BtSmpBondTable[slot].bValid = true;
	s_BtSmpBondTable[slot].PeerAddrType = addrType;
	memcpy(s_BtSmpBondTable[slot].PeerAddr, addr, 6);
	memcpy(&s_BtSmpBondTable[slot].Keys, pKeys, sizeof(BtSmpKeys_t));

	// Snapshot any CCCD subscriptions already active on this link so a client
	// that subscribed before bonding keeps them across reconnects.
	uint8_t nc = pPeer->Conn.NbCccd;
	if (nc > BT_GATT_CCCD_STATE_MAX)
	{
		nc = BT_GATT_CCCD_STATE_MAX;
	}
	for (uint8_t i = 0; i < nc; i++)
	{
		s_BtSmpBondTable[slot].Cccd[i] = pPeer->Conn.Cccd[i];
	}
	s_BtSmpBondTable[slot].NbCccd = nc;

	BtSmpBondSave(slot, &s_BtSmpBondTable[slot], sizeof(BtSmpBond_t));
}

// Persist a CCCD value into the bond for the peer on ConnHdl. No-op when the
// peer is not bonded: CCCD stays volatile for unbonded clients (Core spec Vol 3
// Part G 3.3.3.3). Value 0 removes the entry. Save is skipped when the stored
// value is unchanged, so replaying the set on reconnect costs no NVM writes.
void BtSmpBondCccdSave(uint16_t ConnHdl, uint16_t CccdHdl, uint16_t Value)
{
	BtDevice_t *pPeer = BtPeerFindByHdl(ConnHdl);
	if (pPeer == nullptr)
	{
		return;
	}

	int slot = BtSmpBondFindByAddr(pPeer->Conn.PeerAddrType, pPeer->Conn.PeerAddr);
	if (slot < 0)
	{
		return;					// not bonded
	}

	BtSmpBond_t *p = &s_BtSmpBondTable[slot];
	int idx = -1;
	for (uint8_t i = 0; i < p->NbCccd; i++)
	{
		if (p->Cccd[i].Hdl == CccdHdl)
		{
			idx = i;
			break;
		}
	}

	if (Value == 0)
	{
		if (idx < 0)
		{
			return;				// nothing stored for this handle
		}
		p->Cccd[idx] = p->Cccd[p->NbCccd - 1];
		p->NbCccd--;
	}
	else if (idx >= 0)
	{
		if (p->Cccd[idx].Value == Value)
		{
			return;				// unchanged: no NVM write
		}
		p->Cccd[idx].Value = Value;
	}
	else
	{
		if (p->NbCccd >= BT_GATT_CCCD_STATE_MAX)
		{
			return;				// set full
		}
		idx = p->NbCccd++;
		p->Cccd[idx].Hdl = CccdHdl;
		p->Cccd[idx].Value = Value;
	}

	BtSmpBondSave(slot, p, sizeof(BtSmpBond_t));
}

// Copy the persisted CCCD set for the peer on ConnHdl out as handle/value
// arrays. Returns the entry count, 0 when the peer is not bonded.
uint8_t BtSmpBondCccdGet(uint16_t ConnHdl, uint16_t *pHdl, uint16_t *pValue, uint8_t Max)
{
	BtDevice_t *pPeer = BtPeerFindByHdl(ConnHdl);
	if (pPeer == nullptr || pHdl == nullptr || pValue == nullptr)
	{
		return 0;
	}

	int slot = BtSmpBondFindByAddr(pPeer->Conn.PeerAddrType, pPeer->Conn.PeerAddr);
	if (slot < 0)
	{
		return 0;
	}

	BtSmpBond_t *p = &s_BtSmpBondTable[slot];
	uint8_t n = p->NbCccd;
	if (n > Max)
	{
		n = Max;
	}
	for (uint8_t i = 0; i < n; i++)
	{
		pHdl[i] = p->Cccd[i].Hdl;
		pValue[i] = p->Cccd[i].Value;
	}

	return n;
}

// Override: look up an LTK for an incoming controller LTK request.
// Copy a bond's encryption key size and authentication level onto the live
// link so the ATT permission checks (BtAttLinkEncryptKeySize / Authenticated)
// can gate secured attributes once this link encrypts from the stored key.
static void BtSmpBondApplySecurity(uint16_t ConnHdl, const BtSmpBond_t *pBond)
{
	BtDevice_t *p = BtPeerFindByHdl(ConnHdl);
	if (p != nullptr)
	{
		p->EncKeySize = pBond->Keys.EncKeySize;
		p->bAuthenticated = pBond->Keys.bAuthenticated;
	}
}

bool BtSmpBondLtkLookup(uint16_t ConnHdl, uint64_t Rand,
								   uint16_t Ediv, uint8_t Ltk[16])
{
	// Legacy bond: match on the distributed EDIV/Rand first.
	if (Ediv != 0 || Rand != 0)
	{
		for (int i = 0; i < BT_SMP_BOND_MAX; i++)
		{
			if (s_BtSmpBondTable[i].bValid &&
				!s_BtSmpBondTable[i].Keys.bSc &&
				s_BtSmpBondTable[i].Keys.Ediv == Ediv &&
				s_BtSmpBondTable[i].Keys.Rand == Rand)
			{
				memcpy(Ltk, s_BtSmpBondTable[i].Keys.Ltk, 16);
				BtSmpBondApplySecurity(ConnHdl, &s_BtSmpBondTable[i]);
				return true;
			}
		}
		return false;
	}

	// SC bond (EDIV/Rand zero): match on the peer address of this link.
	BtDevice_t *pPeer = BtPeerFindByHdl(ConnHdl);
	if (pPeer != nullptr)
	{
		int slot = BtSmpBondFindByAddr(pPeer->Conn.PeerAddrType,
									   pPeer->Conn.PeerAddr);
		if (slot >= 0)
		{
			memcpy(Ltk, s_BtSmpBondTable[slot].Keys.Ltk, 16);
			BtSmpBondApplySecurity(ConnHdl, &s_BtSmpBondTable[slot]);
			return true;
		}
	}

	// No address match. A central that reconnects with a rotating resolvable
	// private address presents a different address each time; if its IRK was
	// not distributed the RPA cannot be resolved and the bond cannot be
	// associated with this link. There is NO safe fallback here: returning any
	// stored LTK without a positive address/EDIV+Rand match would hand the
	// bonded key to an unauthenticated peer (authentication bypass). Refuse,
	// and let the controller report no LTK so the link stays unencrypted.
	return false;
}

// Remove all stored bonds, RAM table and non-volatile copy.
void BtSmpBondClearAll(void)
{
	memset(s_BtSmpBondTable, 0, sizeof(s_BtSmpBondTable));
	BtSmpBondErase();
}
