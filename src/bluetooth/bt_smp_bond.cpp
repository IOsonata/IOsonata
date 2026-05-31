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
} BtSmpBond_t;

static BtSmpBond_t s_BtSmpBondTable[BT_SMP_BOND_MAX];

// Weak persistence hooks. Default is RAM only. A flash-backed port overrides
// these to mirror the table to non-volatile storage.
__attribute__((weak)) void BtSmpBondSave(int Slot, const void *pBond, size_t Len)
{
	(void)Slot;
	(void)pBond;
	(void)Len;
}

__attribute__((weak)) void BtSmpBondLoad(void)
{
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
extern "C" void BtSmpBondAdd(uint16_t ConnHdl, const BtSmpKeys_t *pKeys)
{
	if (pKeys == nullptr || !pKeys->bValid)
	{
		return;
	}

	// The address to match on reconnection is the one used on this link. If
	// the peer distributed an identity address, prefer that; otherwise use the
	// connection address.
	uint8_t addrType;
	uint8_t addr[6];

	bool haveId = false;
	for (int i = 0; i < 6; i++)
	{
		if (pKeys->IdAddr[i] != 0) { haveId = true; break; }
	}

	if (haveId)
	{
		addrType = pKeys->IdAddrType;
		memcpy(addr, pKeys->IdAddr, 6);
	}
	else
	{
		BtDevice_t *pPeer = BtPeerFindByHdl(ConnHdl);
		if (pPeer == nullptr)
		{
			return;
		}
		addrType = pPeer->Conn.PeerAddrType;
		memcpy(addr, pPeer->Conn.PeerAddr, 6);
	}

	int slot = BtSmpBondFindByAddr(addrType, addr);
	if (slot < 0)
	{
		slot = BtSmpBondAllocSlot();
	}

	s_BtSmpBondTable[slot].bValid = true;
	s_BtSmpBondTable[slot].PeerAddrType = addrType;
	memcpy(s_BtSmpBondTable[slot].PeerAddr, addr, 6);
	memcpy(&s_BtSmpBondTable[slot].Keys, pKeys, sizeof(BtSmpKeys_t));

	BtSmpBondSave(slot, &s_BtSmpBondTable[slot], sizeof(BtSmpBond_t));
}

// Override: look up an LTK for an incoming controller LTK request.
extern "C" bool BtSmpBondLtkLookup(uint16_t ConnHdl, uint64_t Rand,
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
				return true;
			}
		}
		return false;
	}

	// SC bond (EDIV/Rand zero): match on the peer address of this link.
	BtDevice_t *pPeer = BtPeerFindByHdl(ConnHdl);
	if (pPeer == nullptr)
	{
		return false;
	}

	int slot = BtSmpBondFindByAddr(pPeer->Conn.PeerAddrType, pPeer->Conn.PeerAddr);
	if (slot < 0)
	{
		return false;
	}

	memcpy(Ltk, s_BtSmpBondTable[slot].Keys.Ltk, 16);
	return true;
}

// Remove all stored bonds.
extern "C" void BtSmpBondClearAll(void)
{
	memset(s_BtSmpBondTable, 0, sizeof(s_BtSmpBondTable));
}
