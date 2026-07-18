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

#include "bluetooth/bt_smp.h"
#include "bluetooth/bt_peer.h"
#include "bluetooth/bt_dev.h"

#ifndef BT_SMP_BOND_MAX
#define BT_SMP_BOND_MAX		8
#endif

#define BT_SMP_BOND_RECORD_MAGIC	0x424D5053U	// "SMPB" little-endian
#define BT_SMP_BOND_RECORD_VERSION	1U

typedef struct __Bt_Smp_Bond {
	bool		bValid;			//!< Slot in use
	uint8_t		PeerAddrType;	//!< Peer address type used on the link
	uint8_t		PeerAddr[6];	//!< Peer address used on the link (match key)
	BtSmpKeys_t	Keys;			//!< Stored key set (LTK, IRK, identity, ...)
	uint8_t		NbCccd;			//!< Number of persisted CCCD entries
	BtGattCccdState_t Cccd[BT_GATT_CCCD_STATE_MAX];
	uint32_t	SignCounter;	//!< Last accepted signed-write SignCounter
} BtSmpBond_t;

typedef struct __Bt_Smp_Bond_Record {
	uint32_t	Magic;
	uint16_t	Version;
	uint16_t	Length;
	uint32_t	Crc;
	BtSmpBond_t	Bond;
} BtSmpBondRecord_t;

static BtSmpBond_t s_BtSmpBondTable[BT_SMP_BOND_MAX];

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

// Weak persistence hooks. The default is RAM only. The blob passed to Save is
// a BtSmpBondRecord_t, but the platform treats it as opaque and obtains its
// size from BtSmpBondRecordSize.
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

static void BtSmpBondPersist(int Slot)
{
	if (Slot < 0 || Slot >= BT_SMP_BOND_MAX)
	{
		return;
	}

	BtSmpBondRecord_t record;
	memset(&record, 0, sizeof(record));
	record.Magic = BT_SMP_BOND_RECORD_MAGIC;
	record.Version = BT_SMP_BOND_RECORD_VERSION;
	record.Length = (uint16_t)sizeof(record);
	memcpy(&record.Bond, &s_BtSmpBondTable[Slot], sizeof(record.Bond));
	record.Crc = 0U;
	record.Crc = BtSmpBondCrc32(&record, sizeof(record));
	BtSmpBondSave(Slot, &record, sizeof(record));
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
	bool valid = record.Magic == BT_SMP_BOND_RECORD_MAGIC &&
		record.Version == BT_SMP_BOND_RECORD_VERSION &&
		record.Length == sizeof(record) &&
		savedCrc == BtSmpBondCrc32(&record, sizeof(record)) &&
		BtSmpBondFieldsValid(&record.Bond);

	memset(&s_BtSmpBondTable[Slot], 0, sizeof(s_BtSmpBondTable[Slot]));
	if (valid)
	{
		memcpy(&s_BtSmpBondTable[Slot], &record.Bond, sizeof(record.Bond));
	}
	CryptoSecureWipe(&record, sizeof(record));
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
	return 0;
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
	}

	uint32_t keepCounter = 0U;
	if (!freshSlot && s_BtSmpBondTable[slot].bValid &&
		memcmp(s_BtSmpBondTable[slot].Keys.Csrk, pKeys->Csrk, 16) == 0)
	{
		keepCounter = s_BtSmpBondTable[slot].SignCounter;
	}

	memset(&s_BtSmpBondTable[slot], 0, sizeof(s_BtSmpBondTable[slot]));
	s_BtSmpBondTable[slot].SignCounter = keepCounter;
	s_BtSmpBondTable[slot].bValid = true;
	s_BtSmpBondTable[slot].PeerAddrType = addrType;
	memcpy(s_BtSmpBondTable[slot].PeerAddr, addr, sizeof(addr));
	memcpy(&s_BtSmpBondTable[slot].Keys, pKeys, sizeof(BtSmpKeys_t));

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
	BtSmpBondPersist(slot);
}

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
		return;
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

	if (Value == 0U)
	{
		if (idx < 0)
		{
			return;
		}
		p->Cccd[idx] = p->Cccd[p->NbCccd - 1U];
		p->NbCccd--;
	}
	else if (idx >= 0)
	{
		if (p->Cccd[idx].Value == Value)
		{
			return;
		}
		p->Cccd[idx].Value = Value;
	}
	else
	{
		if (p->NbCccd >= BT_GATT_CCCD_STATE_MAX)
		{
			return;
		}
		idx = p->NbCccd++;
		p->Cccd[idx].Hdl = CccdHdl;
		p->Cccd[idx].Value = Value;
	}
	BtSmpBondPersist(slot);
}

uint8_t BtSmpBondCccdGet(uint16_t ConnHdl, uint16_t *pHdl, uint16_t *pValue,
							 uint8_t Max)
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

	int slot = BtSmpBondFindByAddr(pPeer->Conn.PeerAddrType, pPeer->Conn.PeerAddr);
	if (slot < 0)
	{
		return false;
	}

	BtSmpBond_t *pBond = &s_BtSmpBondTable[slot];
	bool csrk = false;
	for (int i = 0; i < 16; i++)
	{
		if (pBond->Keys.Csrk[i] != 0U)
		{
			csrk = true;
			break;
		}
	}
	if (!csrk)
	{
		return false;
	}

	uint32_t cnt = (uint32_t)pSig[0] | ((uint32_t)pSig[1] << 8) |
				   ((uint32_t)pSig[2] << 16) | ((uint32_t)pSig[3] << 24);
	if (cnt < pBond->SignCounter || cnt == UINT32_MAX)
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

	pBond->SignCounter = cnt + 1U;
	BtSmpBondPersist(slot);
	return true;
}

void BtSmpBondClearAll(void)
{
	CryptoSecureWipe(s_BtSmpBondTable, sizeof(s_BtSmpBondTable));
	BtSmpBondErase();
}
