/**-------------------------------------------------------------------------
@file	bt_peer.h

@brief	Peer manager: tracks active BLE peer devices, one record per link.

Storage is a static, app-sized pool. The application declares a memory
block sized via BT_PEER_POOL_MEMSIZE(N) and passes it to the stack through
BtAppCfg_t's pPeerPoolMem / PeerPoolMemSize fields. The library no longer
holds a compile-time peer-count baked into a precompiled archive; a
single .a can serve a 1-peer peripheral or a multi-link central by changing
the cfg fields only.

The peer manager IS the connection table - GAP, GATT and ATT all index
per-link state through BtPeerFindByHdl. Application code does not normally
call these functions: it receives Connected / Disconnected callbacks and
uses the resulting ConnHdl with the BtApp* operations. The declarations
are public so the stack's internal layers and per-target ports can call
them; apps that include bt_app.h pick this header up transitively, but
they should treat it as internal.

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
#ifndef __BT_PEER_H__
#define __BT_PEER_H__

#include <stdint.h>
#include <stddef.h>

#ifndef __cplusplus
#include <stdbool.h>
#endif

#include "bluetooth/bt_att.h"
#include "bluetooth/bt_dev.h"

// High-level alias of bt_att.h's BT_ATT_HANDLE_INVALID. Same wire value,
// distinct name so each layer uses the term appropriate to it:
//   - low-level code (bt_gap.cpp, bt_attrsp.cpp, bt_attreq.cpp) -> BT_ATT_HANDLE_INVALID
//   - high-level code (bt_peer.cpp, bt_app.cpp, ports)          -> BT_CONN_HDL_INVALID
// Defined here in bt_peer.h (not bt_app.h) so the peer manager's own .cpp
// sees it through its own header rather than reaching up to bt_app.h.
#define BT_CONN_HDL_INVALID		BT_ATT_HANDLE_INVALID

// Header stored immediately before the aligned peer-slot array. BtPeerInit may
// leave a few padding bytes at the start of the caller's block so the first
// BtDevice_t has its native alignment on both MCU and 64-bit host builds.
typedef struct __Bt_Peer_Pool_Hdr {
	uint16_t SlotSize;		//!< sizeof(BtDevice_t) at library build time
	uint16_t Count;			//!< Number of slots in the pool
} BtPeerPoolHdr_t;

// Required slot alignment and bytes the app must reserve to hold N peer slots.
// The alignment-minus-one prefix lets BtPeerInit align the slot array even when
// the supplied uint8_t buffer itself begins at an arbitrary byte address.
#define BT_PEER_POOL_ALIGNMENT		((size_t)__alignof__(BtDevice_t))
#define BT_PEER_POOL_MEMSIZE(N)		(sizeof(BtPeerPoolHdr_t) + \
								 BT_PEER_POOL_ALIGNMENT - 1U + \
								 (N) * sizeof(BtDevice_t))

// Typical use:
//
//   #define MY_PEER_COUNT  8
//   static uint8_t s_PeerPoolMem[BT_PEER_POOL_MEMSIZE(MY_PEER_COUNT)];
//
//   const BtAppCfg_t s_BleAppCfg = {
//       ...
//       .pPeerPoolMem    = s_PeerPoolMem,
//       .PeerPoolMemSize = sizeof(s_PeerPoolMem),
//   };

// Legacy: CFG_BT_PEER_MAX used to size a fixed library array. The array
// is gone; the macro is harmless but no longer controls peer count. Emit
// a build-time nudge if user code still relies on raising it.
#ifdef CFG_BT_PEER_MAX
#if CFG_BT_PEER_MAX > 4
#warning "CFG_BT_PEER_MAX no longer sizes the peer pool. Set BtAppCfg_t.pPeerPoolMem / PeerPoolMemSize using BT_PEER_POOL_MEMSIZE(N)."
#endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

// Subsystem init. Called by each port's BtAppInit early, with the cfg's
// pool fields forwarded straight in. Passing {NULL, 0} selects the small
// library default (BT_PEER_POOL_DEFAULT_COUNT slots). Returns false if
// the buffer cannot hold at least one correctly aligned BtDevice_t slot.
bool         BtPeerInit(uint8_t *pMem, size_t MemSize);

// Register the long-write reassembly pool. Splits pMem evenly across the
// peer slots; each connected peer's Conn.pLongWrBuff points at its slice.
// Call after BtPeerInit (the slot count is fixed there). pMem == NULL or
// MemSize == 0 disables long-write (peers get a NULL buffer).
void         BtPeerLongWrInit(uint8_t *pMem, size_t MemSize);

// Pool ops. Each returns NULL when no free slot is available (Alloc) or
// no matching record exists (FindByHdl, GetActive).
BtDevice_t * BtPeerAlloc(uint16_t ConnHdl);
BtDevice_t * BtPeerFindByHdl(uint16_t ConnHdl);
void         BtPeerFree(BtDevice_t *pPeer);

// First slot with ConnHdl != BT_CONN_HDL_INVALID, or NULL. Shortcut for
// single-link apps that just want "the active peer".
BtDevice_t * BtPeerGetActive(void);

// Iteration accessors for code that walks every slot (connection table
// queries, debug introspection). Return 0 / NULL before BtPeerInit
// has succeeded.
uint16_t     BtPeerCount(void);
BtDevice_t * BtPeerSlot(uint16_t Idx);

// Connection-table queries. These own the link state now (the separate GAP
// connection pool is gone); GAP/GATT receive a borrowed &pPeer->Conn rather
// than querying a pool of their own.
//   BtPeerIsConnected       - true if any slot holds a live link
//   BtPeerGetConnectedHandles - copy up to MaxCount live handles, return count
//   BtPeerFreeByHdl         - free the slot matching Hdl (disconnect convenience)
bool         BtPeerIsConnected(void);
size_t       BtPeerGetConnectedHandles(uint16_t *pHdl, size_t MaxCount);
void         BtPeerFreeByHdl(uint16_t Hdl);

// Allocate (or reuse) a record for a new link and fill its base connection
// fields (replaces the old BtGapAddConnection). OwnAddrType/pOwnAddr are the
// device's own address as the peer saw it when the link was created; a link
// made over a resolvable private address must record that address here so
// the SMP toolbox computes f5/f6/c1 with it. pOwnAddr NULL leaves the field
// zero and the toolbox falls back to BtSmpLocalAddrGet. Returns the record
// or NULL when the pool is full.
BtDevice_t * BtPeerConnected(uint16_t ConnHdl, uint8_t Role, uint8_t AddrType, const uint8_t *pAddr,
							 uint8_t OwnAddrType, const uint8_t *pOwnAddr);

// Handle of the first live link, or BT_CONN_HDL_INVALID. Convenience for
// single-link senders (replaces the old BtGapGetConnection).
uint16_t     BtPeerActiveHdl(void);

#ifdef __cplusplus
}
#endif

#endif // __BT_PEER_H__
