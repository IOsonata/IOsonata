/**-------------------------------------------------------------------------
@file	bt_peer.cpp

@brief	Peer manager implementation.

Pool layout in the user-provided buffer:

	[ BtPeerPoolHdr_t | BtDevice_t[0] | BtDevice_t[1] | ... | BtDevice_t[N-1] ]

BtPeerInit stamps the header with the library's sizeof(BtDevice_t) and
zeroes every slot to "free" (ConnHdl == BT_CONN_HDL_INVALID). All other
ops walk the slot array via the header's Count field.

@author	Hoang Nguyen Hoan
@date	May 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

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
#include "bluetooth/bt_uuid.h"

// Default pool size used when the app passes {NULL, 0}. Matches the
// previous CFG_BT_PEER_MAX = 4 baseline so apps that were sized for
// CFG_BT_PEER_MAX continue to work without explicit cfg fields.
#ifndef BT_PEER_POOL_DEFAULT_COUNT
#define BT_PEER_POOL_DEFAULT_COUNT		4
#endif

alignas(4) static uint8_t s_DefaultPeerPoolMem[BT_PEER_POOL_MEMSIZE(BT_PEER_POOL_DEFAULT_COUNT)];
static BtPeerPoolHdr_t *s_pPeerPool = nullptr;

static inline BtDevice_t * PeerSlots(void)
{
	return s_pPeerPool ? (BtDevice_t*)(s_pPeerPool + 1) : nullptr;
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

	if (size < BT_PEER_POOL_MEMSIZE(1))
	{
		// Not enough room for the header plus a single slot.
		return false;
	}

	size_t payload = size - sizeof(BtPeerPoolHdr_t);
	if (payload % sizeof(BtDevice_t) != 0)
	{
		// sizeof(BtDevice_t) differs between the precompiled library and
		// the application's view of bt_dev.h. Refuse rather than walk off
		// the end of a slot.
		return false;
	}

	uint16_t count = (uint16_t)(payload / sizeof(BtDevice_t));
	if (count == 0)
	{
		return false;
	}

	BtPeerPoolHdr_t *hdr = (BtPeerPoolHdr_t*)mem;
	hdr->SlotSize = (uint16_t)sizeof(BtDevice_t);
	hdr->Count    = count;

	BtDevice_t *slots = (BtDevice_t*)(hdr + 1);
	for (uint16_t i = 0; i < count; i++)
	{
		memset(&slots[i], 0, sizeof(BtDevice_t));
		slots[i].ConnHdl  = BT_CONN_HDL_INVALID;
		slots[i].bIsLocal = false;
	}

	s_pPeerPool = hdr;
	return true;
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
	return &PeerSlots()[Idx];
}

BtDevice_t * BtPeerAlloc(uint16_t ConnHdl)
{
	if (ConnHdl == BT_CONN_HDL_INVALID || s_pPeerPool == nullptr)
	{
		return nullptr;
	}

	BtDevice_t *slots = PeerSlots();
	for (uint16_t i = 0; i < s_pPeerPool->Count; i++)
	{
		if (slots[i].ConnHdl == BT_CONN_HDL_INVALID)
		{
			BtDevice_t *p = &slots[i];
			memset(p, 0, sizeof(*p));
			p->ConnHdl  = ConnHdl;
			p->bIsLocal = false;
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
		if (slots[i].ConnHdl == ConnHdl)
		{
			return &slots[i];
		}
	}
	return nullptr;
}

void BtPeerFree(BtDevice_t *pPeer)
{
	if (pPeer != nullptr)
	{
		// Zero the slot - prevents stale MaxMtu, Role, Addr, Services and
		// any future per-link fields from surviving into the next peer
		// that lands on this slot. BtPeerAlloc memsets on allocation too,
		// but doing it here as well lets BtPeerFindByHdl never see partial
		// half-freed state if a port reads a slot between disconnect
		// and the next connect.
		memset(pPeer, 0, sizeof(*pPeer));
		pPeer->ConnHdl = BT_CONN_HDL_INVALID;
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
		if (slots[i].ConnHdl != BT_CONN_HDL_INVALID)
		{
			return &slots[i];
		}
	}
	return nullptr;
}
