/**-------------------------------------------------------------------------
@file	bt_peer.cpp

@brief	Peer manager implementation.

Pool layout in the user-provided buffer:

	[ optional padding | BtPeerPoolHdr_t | aligned BtDevice_t[0] | ... ]

BtPeerInit places the header immediately before the aligned slot array,
stamps it with the library's sizeof(BtDevice_t), and zeroes every slot to
"free" (Conn.Hdl == BT_CONN_HDL_INVALID). All other ops walk the slot array
through the aligned pointer retained by the peer manager.

@author	Hoang Nguyen Hoan
@date	May 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is furnished
to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------*/
#include <string.h>

#include "istddef.h"
#include "bluetooth/bt_peer.h"
#include "bluetooth/bt_dev.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_uuid.h"

// Default pool when the app passes {NULL, 0}: sized to the hard maximum so a
// default build already supports up to BT_DEV_CONN_MAX simultaneous links.
alignas(BtDevice_t) static uint8_t s_DefaultPeerPoolMem[BT_PEER_POOL_MEMSIZE(BT_DEV_CONN_MAX)];
static BtPeerPoolHdr_t *s_pPeerPool = nullptr;
static BtDevice_t *s_pPeerSlots = nullptr;

// The generic application layer overrides these hooks. Weak defaults keep the
// peer manager independently testable and usable by ports that do not link the
// generic application helper.
__attribute__((weak)) void BtPeerConnectionStateChanged(bool Connected)
{
	(void)Connected;
}

__attribute__((weak)) void BtPeerConnectionRejected(uint16_t ConnHdl)
{
	(void)ConnHdl;
}

// Long-write reassembly pool. Split evenly across the peer slots by
// BtPeerLongWrInit; each slot's slice is re-assigned in BtPeerAlloc because
// the slot is memset on every connect. NULL/0 means long-write is disabled.
static uint8_t *s_pLongWrPool     = nullptr;
static uint16_t s_LongWrSlotSize  = 0;

static inline BtDevice_t * PeerSlots(void)
{
	return s_pPeerSlots;
}

bool BtPeerInit(uint8_t *pMem, size_t MemSize)
{
	uint8_t *mem;
	size_t  size;

	if (pMem == nullptr || MemSize == 0)
	{
		mem  = s_DefaultPeerPoolMem;
		size = sizeof(s_DefaultPeerPoolMem);
	}
	else
	{
		mem  = pMem;
		size = MemSize;
	}

	const size_t slotAlign = alignof(BtDevice_t);
	uintptr_t rawAddr = (uintptr_t)mem;
	uintptr_t slotAddr = rawAddr + sizeof(BtPeerPoolHdr_t);
	size_t misalignment = slotAddr % slotAlign;
	if (misalignment != 0)
	{
		slotAddr += slotAlign - misalignment;
	}

	size_t prefix = (size_t)(slotAddr - rawAddr);
	if (size < prefix + sizeof(BtDevice_t))
	{
		return false;
	}

	size_t payload = size - prefix;
	size_t countValue = payload / sizeof(BtDevice_t);
	size_t trailing = payload - countValue * sizeof(BtDevice_t);

	// BT_PEER_POOL_MEMSIZE reserves at most alignment-1 trailing bytes after
	// the slot array. A larger remainder means the caller did not size the
	// buffer with the current BtDevice_t layout (usually an app/library ABI
	// mismatch), so refuse it rather than infer the wrong slot count.
	if (countValue == 0 || countValue > UINT16_MAX || trailing >= slotAlign)
	{
		return false;
	}

	// The supported number of simultaneous links is whatever the caller sized
	// the pool for, but never above the hard ceiling. A larger pool is capped
	// here so a config can request up to, but not beyond, BT_DEV_CONN_MAX.
	if (countValue > BT_DEV_CONN_MAX)
	{
		countValue = BT_DEV_CONN_MAX;
	}

	BtPeerPoolHdr_t *hdr =
		(BtPeerPoolHdr_t*)(slotAddr - sizeof(BtPeerPoolHdr_t));
	uint16_t count = (uint16_t)countValue;
	hdr->SlotSize = (uint16_t)sizeof(BtDevice_t);
	hdr->Count    = count;

	BtDevice_t *slots = (BtDevice_t*)slotAddr;
	for (uint16_t i = 0; i < count; i++)
	{
		memset(&slots[i], 0, sizeof(BtDevice_t));
		slots[i].Conn.Hdl  = BT_CONN_HDL_INVALID;
		slots[i].bIsLocal = false;
	}

	s_pPeerPool = hdr;
	s_pPeerSlots = slots;
	return true;
}

void BtPeerLongWrInit(uint8_t *pMem, size_t MemSize)
{
	if (pMem == nullptr || MemSize == 0 || s_pPeerPool == nullptr ||
		s_pPeerPool->Count == 0)
	{
		// No pool, or called before BtPeerInit: disable long-write.
		s_pLongWrPool    = nullptr;
		s_LongWrSlotSize = 0;
		return;
	}

	// Split the pool evenly across the peer slots. Each connected peer gets
	// its own slice via BtPeerAlloc, so concurrent links never share a
	// reassembly buffer. Any remainder (MemSize not divisible by Count) is
	// left unused.
	size_t slotSize = MemSize / s_pPeerPool->Count;
	if (slotSize == 0 || slotSize > UINT16_MAX)
	{
		s_pLongWrPool    = nullptr;
		s_LongWrSlotSize = 0;
		return;
	}

	s_pLongWrPool    = pMem;
	s_LongWrSlotSize = (uint16_t)slotSize;
}

uint16_t BtPeerCount(void)
{
	return s_pPeerPool ? s_pPeerPool->Count : 0;
}

BtDevice_t * BtPeerSlot(uint16_t Idx)
{
	if (s_pPeerPool == nullptr || Idx >= s_pPeerPool->Count)
	{
		return nullptr;
	}
	return &s_pPeerSlots[Idx];
}

BtDevice_t * BtPeerAlloc(uint16_t ConnHdl)
{
	if (ConnHdl == BT_CONN_HDL_INVALID || s_pPeerPool == nullptr)
	{
		return nullptr;
	}

	// Reuse an existing record for this handle rather than allocating a
	// second slot. Repeated connect processing for the same link must not
	// create two active peers with the same Conn.Hdl.
	BtDevice_t *existing = BtPeerFindByHdl(ConnHdl);
	if (existing != nullptr)
	{
		return existing;
	}

	BtDevice_t *slots = PeerSlots();
	for (uint16_t i = 0; i < s_pPeerPool->Count; i++)
	{
		if (slots[i].Conn.Hdl == BT_CONN_HDL_INVALID)
		{
			BtDevice_t *p = &slots[i];
			memset(p, 0, sizeof(*p));
			p->Conn.Hdl = ConnHdl;
			p->bIsLocal = false;
			// BT_CONN_ROLE_CENTRAL is 0, so a zeroed record reads as a central
			// link rather than as an unset one, and BtPeerRole cannot tell the
			// two apart. That matters because the SMP and L2CAP role gates
			// refuse a PDU when the role says central: a record allocated but
			// not yet filled in by BtPeerConnected would make a peripheral
			// reject a Pairing Request. Start from UNKNOWN, which both gates
			// treat as "do not gate".
			p->Conn.Role = BT_CONN_ROLE_UNKNOWN;
			// Re-attach this slot's long-write slice (the memset above
			// cleared it). Slot i always maps to the same slice.
			if (s_pLongWrPool != nullptr && s_LongWrSlotSize > 0)
			{
				p->Conn.pLongWrBuff    = s_pLongWrPool + (size_t)i * s_LongWrSlotSize;
				p->Conn.LongWrBuffSize = s_LongWrSlotSize;
			}
			return p;
		}
	}
	return nullptr;
}

BtDevice_t * BtPeerFindByHdl(uint16_t ConnHdl)
{
	if (ConnHdl == BT_CONN_HDL_INVALID || s_pPeerPool == nullptr)
	{
		return nullptr;
	}

	BtDevice_t *slots = PeerSlots();
	for (uint16_t i = 0; i < s_pPeerPool->Count; i++)
	{
		if (slots[i].Conn.Hdl == ConnHdl)
		{
			return &slots[i];
		}
	}
	return nullptr;
}

void BtPeerFree(BtDevice_t *pPeer)
{
	bool bFreed = pPeer != nullptr;
	if (pPeer != nullptr)
	{
		// Drop this link's CCCD subscriptions and recompute the aggregate
		// notify/indicate mirror across the peers that remain, while this
		// slot's handle is still valid (BtGattCccdClear resolves the peer by
		// handle, so it must run before the memset below). This covers a
		// partial disconnect, where other links stay up and the last-link
		// BtGattSrvcDisconnected path further down does not run; without it
		// the departed peer's subscription would linger in pChar->bNotify/
		// bIndic and the shared CCCD value until that CCCD was written again.
		BtGattCccdClear(pPeer->Conn.Hdl);

		// Zero the slot - prevents stale MaxMtu, Role, Addr, Services and
		// any future per-link fields from surviving into the next peer
		// that lands on this slot. BtPeerAlloc memsets on allocation too,
		// but doing it here as well lets BtPeerFindByHdl never see partial
		// half-freed state if a port reads a slot between disconnect
		// and the next connect.
		memset(pPeer, 0, sizeof(*pPeer));
		pPeer->Conn.Hdl = BT_CONN_HDL_INVALID;
	}

	bool bConnected = BtPeerIsConnected();
	if (bFreed)
	{
		BtPeerConnectionStateChanged(bConnected);
	}

	// Last link out: clear per-connection GATT state (CCCD notify/indicate
	// flags) on every registered service. This used to live in
	// BtGapDeleteConnection and only covered the two internal GAP services;
	// owning the pool here lets it cover the whole service list. bt_peer is
	// above bt_gatt, so calling down is within the layering.
	if (bConnected == false)
	{
		for (BtGattSrvc_t *p = BtGattGetSrvcList(); p != nullptr; p = p->pNext)
		{
			BtGattSrvcDisconnected(p);
		}
	}
}

BtDevice_t * BtPeerGetActive(void)
{
	if (s_pPeerPool == nullptr)
	{
		return nullptr;
	}

	BtDevice_t *slots = PeerSlots();
	for (uint16_t i = 0; i < s_pPeerPool->Count; i++)
	{
		if (slots[i].Conn.Hdl != BT_CONN_HDL_INVALID)
		{
			return &slots[i];
		}
	}
	return nullptr;
}

bool BtPeerIsConnected(void)
{
	if (s_pPeerPool == nullptr)
	{
		return false;
	}

	BtDevice_t *slots = PeerSlots();
	for (uint16_t i = 0; i < s_pPeerPool->Count; i++)
	{
		if (slots[i].Conn.Hdl != BT_CONN_HDL_INVALID)
		{
			return true;
		}
	}
	return false;
}

size_t BtPeerGetConnectedHandles(uint16_t *pHdl, size_t MaxCount)
{
	if (pHdl == nullptr || MaxCount == 0 || s_pPeerPool == nullptr)
	{
		return 0;
	}

	// Fill pHdl with connected handles in pool order, up to MaxCount.
	BtDevice_t *slots = PeerSlots();
	size_t n = 0;
	for (uint16_t i = 0; i < s_pPeerPool->Count && n < MaxCount; i++)
	{
		if (slots[i].Conn.Hdl != BT_CONN_HDL_INVALID)
		{
			pHdl[n++] = slots[i].Conn.Hdl;
		}
	}
	return n;
}

void BtPeerFreeByHdl(uint16_t Hdl)
{
	BtPeerFree(BtPeerFindByHdl(Hdl));
}

BtDevice_t * BtPeerConnected(uint16_t ConnHdl, uint8_t Role, uint8_t AddrType, const uint8_t *pAddr,
							 uint8_t OwnAddrType, const uint8_t *pOwnAddr)
{
	BtDevice_t *p = BtPeerAlloc(ConnHdl);
	if (p == nullptr)
	{
		BtPeerConnectionRejected(ConnHdl);
		return nullptr;
	}

	p->Conn.Role = Role;
	p->Conn.PeerAddrType = AddrType;
	if (pAddr != nullptr)
	{
		memcpy(p->Conn.PeerAddr, pAddr, sizeof(p->Conn.PeerAddr));
	}
	p->Conn.OwnAddrType = OwnAddrType;
	if (pOwnAddr != nullptr)
	{
		memcpy(p->Conn.OwnAddr, pOwnAddr, sizeof(p->Conn.OwnAddr));
	}
	BtPeerConnectionStateChanged(true);
	return p;
}

uint16_t BtPeerActiveHdl(void)
{
	BtDevice_t *p = BtPeerGetActive();
	return p != nullptr ? p->Conn.Hdl : BT_CONN_HDL_INVALID;
}

uint8_t BtPeerRole(uint16_t ConnHdl)
{
	BtDevice_t *p = BtPeerFindByHdl(ConnHdl);
	if (p == nullptr)
	{
		return BT_CONN_ROLE_UNKNOWN;
	}
	// Only the two HCI values are meaningful; anything else in the field
	// (an unconverted vendor encoding or an unset record) reports UNKNOWN
	// rather than being misread as a role.
	if (p->Conn.Role == BT_CONN_ROLE_CENTRAL || p->Conn.Role == BT_CONN_ROLE_PERIPHERAL)
	{
		return p->Conn.Role;
	}
	return BT_CONN_ROLE_UNKNOWN;
}
