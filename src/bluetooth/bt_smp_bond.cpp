/**-------------------------------------------------------------------------
@file	bt_smp_bond.cpp

@brief	Simple bond store for the SMP layer.

Persists the keys produced by a successful pairing so a later reconnection
can re-encrypt the link with the stored LTK instead of pairing again.

This implementation keeps the bond table in RAM. A platform that wants the
bonds to survive a reset replaces BtSmpBondSave / BtSmpBondLoad with a
flash-backed implementation. The stored blob is versioned and protected by a
CRC, and the restore path copies it into aligned storage before validation.

For LE Secure Connections the LTK is derived (EDIV and Rand are zero), so a
bond is matched by the peer address used on the link. Legacy bonds carry the
distributed EDIV/Rand and are matched on those first.

@author	Hoang Nguyen Hoan
@date	Jul. 17, 2026

@license

MIT License

Copyright (c) 2026, I-SYST, all rights reserved

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
#include <stdint.h>
#include <string.h>
#include <atomic>

#if defined(__arm__)
#include "iatomic.h"
#endif
#include "bluetooth/bt_smp.h"
#include "bluetooth/bt_peer.h"
#include "bluetooth/bt_dev.h"

#ifndef BT_SMP_BOND_MAX
#define BT_SMP_BOND_MAX		8
#endif

// Signed-write counters cannot be written from a high-priority Bluetooth event
// on targets whose NVM needs a lower-priority radio-arbitration callback. Reserve
// a small range ahead of the receive counter instead. The exclusive range end
// is committed before any counter in the range is accepted; after reset the
// receiver starts at that end, so every counter that might have been accepted
// before the reset is skipped.
#ifndef BT_SMP_SIGN_COUNTER_WINDOW
#define BT_SMP_SIGN_COUNTER_WINDOW		32U
#endif

#ifndef BT_SMP_SIGN_COUNTER_REFILL
#define BT_SMP_SIGN_COUNTER_REFILL		8U
#endif

#if BT_SMP_SIGN_COUNTER_WINDOW == 0 || \
	BT_SMP_SIGN_COUNTER_REFILL >= BT_SMP_SIGN_COUNTER_WINDOW
#error "BT_SMP_SIGN_COUNTER_WINDOW must be nonzero and REFILL must be smaller"
#endif

#define BT_SMP_BOND_RECORD_MAGIC		0x424D5053U	// "SMPB" little-endian
#define BT_SMP_BOND_RECORD_VERSION_OLD	1U
#define BT_SMP_BOND_RECORD_VERSION		2U

typedef struct __Bt_Smp_Bond {
	bool		bValid;			//!< Slot in use
	uint8_t		PeerAddrType;	//!< Peer address type used on the link
	uint8_t		PeerAddr[6];	//!< Peer address used on the link (match key)
	BtSmpKeys_t	Keys;			//!< Stored key set (LTK, IRK, identity, ...)
	uint8_t		NbCccd;			//!< Number of persisted CCCD entries
	BtGattCccdState_t Cccd[BT_GATT_CCCD_STATE_MAX];
	uint32_t	SignCounter;	//!< Exclusive durable signed-write counter high-water mark
} BtSmpBond_t;

typedef struct __Bt_Smp_Bond_Record {
	uint32_t	Magic;
	uint16_t	Version;
	uint16_t	Length;
	uint32_t	Crc;
	BtSmpBond_t	Bond;
} BtSmpBondRecord_t;

typedef struct __Bt_Smp_Sign_State {
	std::atomic<uint32_t> Next;		//!< Lowest counter accepted in this boot
	std::atomic<uint32_t> DurableHigh;	//!< Exclusive end already committed to NVM
	std::atomic<uint32_t> PendingHigh;	//!< Exclusive end included in the next save
	std::atomic<bool> Ready;			//!< At least one durably reserved counter remains
} BtSmpSignState_t;

static BtSmpBond_t s_BtSmpBondTable[BT_SMP_BOND_MAX];
static BtSmpSignState_t s_BtSmpSignState[BT_SMP_BOND_MAX];

#if !defined(__arm__)
static std::atomic_flag s_BtSmpBondTableLock = ATOMIC_FLAG_INIT;
#endif

static uint32_t BtSmpBondTableEnter(void)
{
#if defined(__arm__)
	return EnterCriticalSection();
#else
	while (s_BtSmpBondTableLock.test_and_set(std::memory_order_acquire))
	{
	}
	return 0;
#endif
}

static void BtSmpBondTableExit(uint32_t State)
{
#if defined(__arm__)
	ExitCriticalSection(State);
#else
	(void)State;
	s_BtSmpBondTableLock.clear(std::memory_order_release);
#endif
}

static bool BtSmpBondEqualCT(const uint8_t *a, const uint8_t *b, size_t len)
{
	uint8_t diff = 0;
	for (size_t i = 0; i < len; i++)
	{
		diff |= (uint8_t)(a[i] ^ b[i]);
	}
	return diff == 0;
}

static uint32_t BtSmpBondCrc32(const void *pData, size_t Len)
{
	const uint8_t *p = (const uint8_t *)pData;
	uint32_t crc = 0xFFFFFFFFU;
	for (size_t i = 0; i < Len; i++)
	{
		crc ^= p[i];
		for (int bit = 0; bit < 8; bit++)
		{
			uint32_t mask = 0U - (crc & 1U);
			crc = (crc >> 1) ^ (0xEDB88320U & mask);
		}
	}
	return ~crc;
}

static bool BtSmpBondHasCsrk(const BtSmpBond_t *pBond)
{
	if (pBond == nullptr)
	{
		return false;
	}

	uint8_t nz = 0;
	for (int i = 0; i < 16; i++)
	{
		nz |= pBond->Keys.Csrk[i];
	}
	return nz != 0;
}

static void BtSmpSignStateReset(int Slot, uint32_t Next, uint32_t DurableHigh)
{
	if (Slot < 0 || Slot >= BT_SMP_BOND_MAX)
	{
		return;
	}

	s_BtSmpSignState[Slot].Next.store(Next);
	s_BtSmpSignState[Slot].DurableHigh.store(DurableHigh);
	s_BtSmpSignState[Slot].PendingHigh.store(0U);
	s_BtSmpSignState[Slot].Ready.store(false);
}

static uint32_t BtSmpSignNextHigh(uint32_t High)
{
	if (High >= UINT32_MAX - BT_SMP_SIGN_COUNTER_WINDOW)
	{
		return UINT32_MAX;
	}
	return High + BT_SMP_SIGN_COUNTER_WINDOW;
}

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

int BtSmpBondSlotCount(void)
{
	return BT_SMP_BOND_MAX;
}

size_t BtSmpBondRecordSize(void)
{
	return sizeof(BtSmpBondRecord_t);
}

// Build one coherent slot image. The deferred persistence path can run from a
// different context than the Bluetooth event that changes the slot. On Arm,
// the short copy window masks interrupts; host tests use an atomic flag. CRC
// calculation and NVM work remain outside the protected window.
size_t BtSmpBondSerialize(int Slot, void *pBuff, size_t BuffLen)
{
	if (Slot < 0 || Slot >= BT_SMP_BOND_MAX || pBuff == nullptr ||
		BuffLen < sizeof(BtSmpBondRecord_t))
	{
		return 0;
	}

	BtSmpBondRecord_t record;
	memset(&record, 0, sizeof(record));
	record.Magic = BT_SMP_BOND_RECORD_MAGIC;
	record.Version = BT_SMP_BOND_RECORD_VERSION;
	record.Length = (uint16_t)sizeof(record);

	uint32_t state = BtSmpBondTableEnter();
	memcpy(&record.Bond, &s_BtSmpBondTable[Slot], sizeof(record.Bond));
	uint32_t durable = s_BtSmpSignState[Slot].DurableHigh.load();
	uint32_t pending = s_BtSmpSignState[Slot].PendingHigh.load();
	record.Bond.SignCounter = pending > durable ? pending : durable;
	BtSmpBondTableExit(state);

	record.Crc = 0U;
	record.Crc = BtSmpBondCrc32(&record, sizeof(record));

	memcpy(pBuff, &record, sizeof(record));
	CryptoSecureWipe(&record, sizeof(record));

	return sizeof(record);
}

static void BtSmpBondPersist(int Slot)
{
	BtSmpBondRecord_t record;

	size_t len = BtSmpBondSerialize(Slot, &record, sizeof(record));
	if (len > 0)
	{
		BtSmpBondSave(Slot, &record, len);
	}
	CryptoSecureWipe(&record, sizeof(record));
}

static bool BtSmpSignCounterPrepare(int Slot)
{
	if (Slot < 0 || Slot >= BT_SMP_BOND_MAX ||
		!BtSmpBondHasCsrk(&s_BtSmpBondTable[Slot]))
	{
		return false;
	}

	BtSmpSignState_t &state = s_BtSmpSignState[Slot];
	uint32_t durable = state.DurableHigh.load();
	uint32_t pending = state.PendingHigh.load();
	uint32_t next = state.Next.load();

	if (pending > durable || durable == UINT32_MAX)
	{
		return false;
	}

	if (state.Ready.load() && next < durable &&
		(durable - next) > BT_SMP_SIGN_COUNTER_REFILL)
	{
		return false;
	}

	uint32_t high = BtSmpSignNextHigh(durable);
	if (high <= durable)
	{
		return false;
	}

	state.PendingHigh.store(high);
	return true;
}

extern "C" void BtSmpBondPersistComplete(int Slot, const void *pBond,
											 size_t Len, bool Success)
{
	if (!Success || Slot < 0 || Slot >= BT_SMP_BOND_MAX || pBond == nullptr ||
		Len != sizeof(BtSmpBondRecord_t))
	{
		return;
	}

	BtSmpBondRecord_t record;
	memcpy(&record, pBond, sizeof(record));
	uint32_t savedCrc = record.Crc;
	record.Crc = 0U;

	bool valid = record.Magic == BT_SMP_BOND_RECORD_MAGIC &&
		record.Version == BT_SMP_BOND_RECORD_VERSION &&
		record.Length == sizeof(record) &&
		savedCrc == BtSmpBondCrc32(&record, sizeof(record));

	uint32_t state = BtSmpBondTableEnter();
	BtSmpBond_t *pLive = &s_BtSmpBondTable[Slot];
	if (valid)
	{
		valid = pLive->bValid &&
			pLive->PeerAddrType == record.Bond.PeerAddrType &&
			memcmp(pLive->PeerAddr, record.Bond.PeerAddr, 6) == 0 &&
			memcmp(pLive->Keys.Csrk, record.Bond.Keys.Csrk, 16) == 0;
	}

	if (valid)
	{
		BtSmpSignState_t &signState = s_BtSmpSignState[Slot];
		uint32_t committed = record.Bond.SignCounter;
		uint32_t durable = signState.DurableHigh.load();

		if (committed > durable)
		{
			signState.DurableHigh.store(committed);
			pLive->SignCounter = committed;
			durable = committed;
		}

		uint32_t pending = signState.PendingHigh.load();
		if (pending != 0U && pending <= durable)
		{
			signState.PendingHigh.store(0U);
		}

		signState.Ready.store(BtSmpBondHasCsrk(pLive) &&
							 signState.Next.load() < durable);
	}
	BtSmpBondTableExit(state);

	CryptoSecureWipe(&record, sizeof(record));
}

static bool BtSmpBondFieldsValid(const BtSmpBond_t *p)
{
	return p != nullptr && p->bValid && p->Keys.bValid &&
		p->NbCccd <= BT_GATT_CCCD_STATE_MAX &&
		p->PeerAddrType <= BTADDR_TYPE_RANDOM_STATIC &&
		p->Keys.EncKeySize >= BT_SMP_CFG_MIN_ENC_KEY_SIZE &&
		p->Keys.EncKeySize <= BT_SMP_MAX_ENC_KEY_SIZE;
}

void BtSmpBondRestore(int Slot, const void *pBond, size_t Len)
{
	if (Slot < 0 || Slot >= BT_SMP_BOND_MAX || pBond == nullptr ||
		Len != sizeof(BtSmpBondRecord_t))
	{
		return;
	}

	BtSmpBondRecord_t record;
	memcpy(&record, pBond, sizeof(record));
	uint32_t savedCrc = record.Crc;
	record.Crc = 0U;
	bool versionValid = record.Version == BT_SMP_BOND_RECORD_VERSION ||
		record.Version == BT_SMP_BOND_RECORD_VERSION_OLD;
	bool valid = record.Magic == BT_SMP_BOND_RECORD_MAGIC && versionValid &&
		record.Length == sizeof(record) &&
		savedCrc == BtSmpBondCrc32(&record, sizeof(record)) &&
		BtSmpBondFieldsValid(&record.Bond);
	bool migrate = valid && record.Version == BT_SMP_BOND_RECORD_VERSION_OLD;
	bool persist = false;

	uint32_t state = BtSmpBondTableEnter();
	memset(&s_BtSmpBondTable[Slot], 0, sizeof(s_BtSmpBondTable[Slot]));
	BtSmpSignStateReset(Slot, 0U, 0U);

	if (valid)
	{
		memcpy(&s_BtSmpBondTable[Slot], &record.Bond, sizeof(record.Bond));

		if (migrate)
		{
			CryptoSecureWipe(s_BtSmpBondTable[Slot].Keys.Csrk, 16);
			s_BtSmpBondTable[Slot].SignCounter = 0U;
			persist = true;
		}
		else
		{
			uint32_t high = s_BtSmpBondTable[Slot].SignCounter;
			BtSmpSignStateReset(Slot, high, high);
			persist = BtSmpSignCounterPrepare(Slot);
		}
	}
	BtSmpBondTableExit(state);

	CryptoSecureWipe(&record, sizeof(record));

	if (persist)
	{
		BtSmpBondPersist(Slot);
	}
}

static bool BtSmpAddrIsRpa(uint8_t AddrType, const uint8_t Addr[6])
{
	if (Addr == nullptr || AddrType != BTADDR_TYPE_RAND)
	{
		return false;
	}
	return (Addr[5] & 0xC0) == 0x40;
}

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

	if (BtSmpAddrIsRpa(AddrType, Addr))
	{
		for (int i = 0; i < BT_SMP_BOND_MAX; i++)
		{
			if (s_BtSmpBondTable[i].bValid &&
				BtSmpRpaResolve(s_BtSmpBondTable[i].Keys.Irk, Addr))
			{
				return i;
			}
		}
	}
	return -1;
}

static int BtSmpBondFind(uint16_t ConnHdl, uint64_t Rand, uint16_t Ediv)
{
	if (Ediv != 0U || Rand != 0U)
	{
		for (int i = 0; i < BT_SMP_BOND_MAX; i++)
		{
			if (s_BtSmpBondTable[i].bValid &&
				!s_BtSmpBondTable[i].Keys.bSc &&
				s_BtSmpBondTable[i].Keys.Ediv == Ediv &&
				s_BtSmpBondTable[i].Keys.Rand == Rand)
			{
				return i;
			}
		}
		return -1;
	}

	BtDevice_t *pPeer = BtPeerFindByHdl(ConnHdl);
	return pPeer != nullptr ?
		BtSmpBondFindByAddr(pPeer->Conn.PeerAddrType, pPeer->Conn.PeerAddr) : -1;
}

static int BtSmpBondAllocSlot(void)
{
	for (int i = 0; i < BT_SMP_BOND_MAX; i++)
	{
		if (!s_BtSmpBondTable[i].bValid)
		{
			return i;
		}
	}
	return -1;
}

void BtSmpBondAdd(uint16_t ConnHdl, const BtSmpKeys_t *pKeys)
{
	if (pKeys == nullptr || !pKeys->bValid)
	{
		return;
	}

	BtDevice_t *pPeer = BtPeerFindByHdl(ConnHdl);
	if (pPeer == nullptr)
	{
		return;
	}

	uint8_t addrType = pPeer->Conn.PeerAddrType;
	uint8_t addr[6];
	memcpy(addr, pPeer->Conn.PeerAddr, sizeof(addr));

	int slot = BtSmpBondFindByAddr(addrType, addr);
	const bool freshSlot = slot < 0;
	if (freshSlot)
	{
		slot = BtSmpBondAllocSlot();
		if (slot < 0)
		{
			return;
		}
	}

	uint32_t state = BtSmpBondTableEnter();
	bool sameCsrk = !freshSlot && s_BtSmpBondTable[slot].bValid &&
		memcmp(s_BtSmpBondTable[slot].Keys.Csrk, pKeys->Csrk, 16) == 0;

	uint32_t keepHigh = sameCsrk ?
		s_BtSmpSignState[slot].DurableHigh.load() : 0U;
	uint32_t keepNext = sameCsrk ?
		s_BtSmpSignState[slot].Next.load() : 0U;
	uint32_t keepPending = sameCsrk ?
		s_BtSmpSignState[slot].PendingHigh.load() : 0U;
	bool keepReady = sameCsrk && s_BtSmpSignState[slot].Ready.load();

	memset(&s_BtSmpBondTable[slot], 0, sizeof(s_BtSmpBondTable[slot]));
	s_BtSmpBondTable[slot].SignCounter = keepHigh;
	s_BtSmpBondTable[slot].bValid = true;
	s_BtSmpBondTable[slot].PeerAddrType = addrType;
	memcpy(s_BtSmpBondTable[slot].PeerAddr, addr, sizeof(addr));
	memcpy(&s_BtSmpBondTable[slot].Keys, pKeys, sizeof(BtSmpKeys_t));

	if (sameCsrk)
	{
		s_BtSmpSignState[slot].Next.store(keepNext);
		s_BtSmpSignState[slot].DurableHigh.store(keepHigh);
		s_BtSmpSignState[slot].PendingHigh.store(keepPending);
		s_BtSmpSignState[slot].Ready.store(keepReady);
	}
	else
	{
		BtSmpSignStateReset(slot, 0U, 0U);
	}

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

	(void)BtSmpSignCounterPrepare(slot);
	BtSmpBondTableExit(state);
	BtSmpBondPersist(slot);
}

void BtSmpBondCccdSave(uint16_t ConnHdl, uint16_t CccdHdl, uint16_t Value)
{
	BtDevice_t *pPeer = BtPeerFindByHdl(ConnHdl);
	if (pPeer == nullptr)
	{
		return;
	}

	int slot = BtSmpBondFindByAddr(pPeer->Conn.PeerAddrType,
		pPeer->Conn.PeerAddr);
	if (slot < 0)
	{
		return;
	}

	bool changed = false;
	uint32_t state = BtSmpBondTableEnter();
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

	if (Value == 0U)
	{
		if (idx >= 0)
		{
			p->Cccd[idx] = p->Cccd[p->NbCccd - 1U];
			p->NbCccd--;
			changed = true;
		}
	}
	else if (idx >= 0)
	{
		if (p->Cccd[idx].Value != Value)
		{
			p->Cccd[idx].Value = Value;
			changed = true;
		}
	}
	else if (p->NbCccd < BT_GATT_CCCD_STATE_MAX)
	{
		idx = p->NbCccd++;
		p->Cccd[idx].Hdl = CccdHdl;
		p->Cccd[idx].Value = Value;
		changed = true;
	}
	BtSmpBondTableExit(state);

	if (changed)
	{
		BtSmpBondPersist(slot);
	}
}

uint8_t BtSmpBondCccdGet(uint16_t ConnHdl, uint16_t *pHdl,
		uint16_t *pValue, uint8_t Max)
{
	BtDevice_t *pPeer = BtPeerFindByHdl(ConnHdl);
	if (pPeer == nullptr || pHdl == nullptr || pValue == nullptr)
	{
		return 0;
	}

	int slot = BtSmpBondFindByAddr(pPeer->Conn.PeerAddrType,
		pPeer->Conn.PeerAddr);
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

bool BtSmpBondKeysLookup(uint16_t ConnHdl, uint64_t Rand, uint16_t Ediv,
							BtSmpKeys_t *pKeys)
{
	if (pKeys == nullptr)
	{
		return false;
	}
	CryptoSecureWipe(pKeys, sizeof(*pKeys));
	int slot = BtSmpBondFind(ConnHdl, Rand, Ediv);
	if (slot < 0)
	{
		return false;
	}
	memcpy(pKeys, &s_BtSmpBondTable[slot].Keys, sizeof(*pKeys));
	return pKeys->bValid;
}

bool BtSmpBondLtkLookup(uint16_t ConnHdl, uint64_t Rand,
							   uint16_t Ediv, uint8_t Ltk[16])
{
	if (Ltk == nullptr)
	{
		return false;
	}
	BtSmpKeys_t keys;
	bool found = BtSmpBondKeysLookup(ConnHdl, Rand, Ediv, &keys);
	if (found)
	{
		memcpy(Ltk, keys.Ltk, 16);
	}
	else
	{
		CryptoSecureWipe(Ltk, 16);
	}
	CryptoSecureWipe(&keys, sizeof(keys));
	return found;
}

bool BtSmpBonded(uint16_t ConnHdl)
{
	return BtSmpBondFind(ConnHdl, 0U, 0U) >= 0;
}

bool BtSmpSignVerify(uint16_t ConnHdl, const uint8_t *pMsg, size_t MsgLen,
					 const uint8_t *pSig)
{
	if (pSig == nullptr || (pMsg == nullptr && MsgLen > 0U))
	{
		return false;
	}

	BtDevice_t *pPeer = BtPeerFindByHdl(ConnHdl);
	if (pPeer == nullptr)
	{
		return false;
	}

	int slot = BtSmpBondFindByAddr(pPeer->Conn.PeerAddrType,
		pPeer->Conn.PeerAddr);
	if (slot < 0)
	{
		return false;
	}

	BtSmpBond_t *pBond = &s_BtSmpBondTable[slot];
	BtSmpSignState_t &state = s_BtSmpSignState[slot];
	if (!BtSmpBondHasCsrk(pBond) || !state.Ready.load())
	{
		return false;
	}

	uint32_t cnt = (uint32_t)pSig[0] | ((uint32_t)pSig[1] << 8) |
				   ((uint32_t)pSig[2] << 16) | ((uint32_t)pSig[3] << 24);
	uint32_t next = state.Next.load();
	uint32_t high = state.DurableHigh.load();

	if (cnt < next || cnt >= high || cnt == UINT32_MAX)
	{
		return false;
	}

	uint8_t mac[8];
	BtSmpSignMac(pBond->Keys.Csrk, pMsg, MsgLen, mac);
	if (!BtSmpBondEqualCT(mac, &pSig[4], sizeof(mac)))
	{
		CryptoSecureWipe(mac, sizeof(mac));
		return false;
	}
	CryptoSecureWipe(mac, sizeof(mac));

	next = cnt + 1U;
	state.Next.store(next);
	if (next >= high)
	{
		state.Ready.store(false);
	}

	if (high - next <= BT_SMP_SIGN_COUNTER_REFILL &&
		BtSmpSignCounterPrepare(slot))
	{
		BtSmpBondPersist(slot);
	}

	return true;
}

void BtSmpBondClearAll(void)
{
	uint32_t state = BtSmpBondTableEnter();
	CryptoSecureWipe(s_BtSmpBondTable, sizeof(s_BtSmpBondTable));
	for (int i = 0; i < BT_SMP_BOND_MAX; i++)
	{
		BtSmpSignStateReset(i, 0U, 0U);
	}
	BtSmpBondTableExit(state);
	BtSmpBondErase();
}
