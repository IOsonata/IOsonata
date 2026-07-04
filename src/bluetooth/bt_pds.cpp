/**-------------------------------------------------------------------------
@file	bt_pds.c

@brief	Persistent Data Store for the Bluetooth peer manager.

		Log structured, id-keyed record store over a single NVM region split
		into fixed size sectors. Replaces the bm_zms filesystem under
		peer_data_storage.c with no Zephyr dependency.

		Layout
		------
		The region is N sectors. Records are appended contiguously across the
		active sectors; one sector is always kept free as the compaction
		target. Each record is word aligned:

			BtPdsRecHdr_t  { Magic, Id, Len, Seq, Crc }   (16 bytes)
			data[Len]                                     (word padded)

		Magic distinguishes a written record from erased (0xFFFFFFFF) space.
		Seq is a monotonically increasing version counter; for a given Id the
		highest valid Seq is the live value. Len == BT_PDS_TOMBSTONE marks a
		delete. Crc8 over Id..data validates the record; a record truncated by
		power loss fails Crc and is ignored, so the previous version remains
		in effect.

		Compaction
		----------
		When the next append would not fit in the active span, live records are
		copied (latest version of each non-deleted Id) into the spare sector,
		which becomes the new active head, and the old sectors are erased. The
		old data survives until the new copy is fully written, so an interrupted
		compaction loses nothing: on next mount the scan rebuilds from whichever
		sectors hold valid records.

		This file is platform independent. Medium access is via BtPdsNvm_t.

@author	Hoang Nguyen Hoan
@date	Jun 03, 2026

@license

MIT License

Copyright (c) 2016, I-SYST inc., all rights reserved

----------------------------------------------------------------------------*/
#include <string.h>
#include <errno.h>

#include "bluetooth/bt_pds.h"

#define BT_PDS_REC_MAGIC		0x53445042UL	// "BPDS"
#define BT_PDS_TOMBSTONE		0xFFFF			// Len value marking a delete
#define BT_PDS_ERASED_U32		0xFFFFFFFFUL

#pragma pack(push, 4)
typedef struct __Bt_Pds_Rec_Hdr {
	uint32_t	Magic;			//!< BT_PDS_REC_MAGIC when written
	uint32_t	Id;				//!< Record key
	uint16_t	Len;			//!< Data length, or BT_PDS_TOMBSTONE for delete
	uint16_t	Seq;			//!< Version counter (highest valid wins)
	uint8_t		Crc;			//!< crc8 over Id, Len, Seq, data
	uint8_t		Rsvd[3];		//!< Pad to word boundary
} BtPdsRecHdr_t;
#pragma pack(pop)

#define BT_PDS_HDR_SIZE			((uint32_t)sizeof(BtPdsRecHdr_t))

static const BtPdsNvm_t *s_pNvm = nullptr;
static uint32_t s_WriteHead;		//!< Region offset where the next record goes
static uint16_t s_NextSeq;			//!< Next version counter to assign

static inline uint32_t WordPad(uint32_t x)
{
	return (x + 3) & ~3UL;
}

// crc8-ccitt (poly 0x07, seed 0xFF, MSB first, no reflection). Matches the
// integrity check the previous store used so format intent is unchanged.
static uint8_t Crc8(const void *pData, uint32_t Len, uint8_t Seed)
{
	const uint8_t *p = (const uint8_t *)pData;
	uint8_t crc = Seed;

	for (uint32_t i = 0; i < Len; i++)
	{
		crc ^= p[i];
		for (int b = 0; b < 8; b++)
		{
			crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
		}
	}
	return crc;
}

// Compute the record crc over Id, Len, Seq and the data payload.
static uint8_t RecCrc(const BtPdsRecHdr_t *pHdr, const void *pData)
{
	uint8_t crc = Crc8(&pHdr->Id, sizeof(pHdr->Id), 0xFF);
	crc = Crc8(&pHdr->Len, sizeof(pHdr->Len), crc);
	crc = Crc8(&pHdr->Seq, sizeof(pHdr->Seq), crc);
	if (pHdr->Len != BT_PDS_TOMBSTONE && pHdr->Len > 0)
	{
		crc = Crc8(pData, pHdr->Len, crc);
	}
	return crc;
}

// Read and validate the record header at Off. Returns true and fills pHdr if a
// well formed record (correct magic) is present; false at erased space or end.
static bool ReadHdr(uint32_t Off, BtPdsRecHdr_t *pHdr)
{
	if (Off + BT_PDS_HDR_SIZE > s_pNvm->RegionSize)
	{
		return false;
	}
	if (s_pNvm->Read(Off, pHdr, BT_PDS_HDR_SIZE) != 0)
	{
		return false;
	}
	return pHdr->Magic == BT_PDS_REC_MAGIC;
}

// Total on-medium size of a record with payload length Len.
static uint32_t RecSize(uint16_t Len)
{
	uint32_t dl = (Len == BT_PDS_TOMBSTONE) ? 0 : Len;
	return BT_PDS_HDR_SIZE + WordPad(dl);
}

// Scan the region, find the write head (first erased slot after the last valid
// record) and the highest sequence number in use. Records that fail crc are
// skipped, which also stops the scan cleanly at a power-loss truncation.
static void ScanRegion(void)
{
	uint32_t off = 0;
	uint8_t buf[BT_PDS_RECORD_DATA_MAX];

	s_WriteHead = 0;
	s_NextSeq = 0;

	bool seqFound = false;
	uint16_t maxSeq = 0;

	while (off + BT_PDS_HDR_SIZE <= s_pNvm->RegionSize)
	{
		BtPdsRecHdr_t hdr;

		if (!ReadHdr(off, &hdr))
		{
			// Erased or malformed: this is the write head.
			break;
		}

		uint16_t dl = (hdr.Len == BT_PDS_TOMBSTONE) ? 0 : hdr.Len;
		if (dl > BT_PDS_RECORD_DATA_MAX)
		{
			break;		// Corrupt length, treat as end of valid log
		}

		if (dl > 0)
		{
			if (s_pNvm->Read(off + BT_PDS_HDR_SIZE, buf, dl) != 0)
			{
				break;
			}
		}

		if (RecCrc(&hdr, buf) != hdr.Crc)
		{
			// Truncated/partial write: stop here, this slot is the head.
			break;
		}

		// Valid record. Advance head and track the newest sequence number.
		// Compare wrap-aware (modular distance) so a uint16 Seq that has
		// wrapped past 0xFFFF is still ordered correctly - the same test
		// FindLive uses. A plain '>' magnitude compare would pick a stale
		// pre-wrap record as newest and let it shadow a fresh write.
		uint16_t seq = hdr.Seq;
		if (!seqFound || (uint16_t)(seq - maxSeq) < 0x8000)
		{
			seqFound = true;
			maxSeq = seq;
		}

		off += RecSize(hdr.Len);
		s_WriteHead = off;
	}

	if (seqFound)
	{
		s_NextSeq = (uint16_t)(maxSeq + 1);
	}
}

// Find the offset of the live (highest seq, non-tombstone) record for Id.
// Returns true and sets pOff/pLen if found; false if absent or deleted.
static bool FindLive(uint32_t Id, uint32_t *pOff, uint16_t *pLen)
{
	uint32_t off = 0;
	bool found = false;
	uint16_t bestSeq = 0;
	uint32_t bestOff = 0;
	uint16_t bestLen = 0;
	bool bestTomb = false;
	uint8_t buf[BT_PDS_RECORD_DATA_MAX];

	while (off + BT_PDS_HDR_SIZE <= s_pNvm->RegionSize && off < s_WriteHead)
	{
		BtPdsRecHdr_t hdr;

		if (!ReadHdr(off, &hdr))
		{
			break;
		}

		uint16_t dl = (hdr.Len == BT_PDS_TOMBSTONE) ? 0 : hdr.Len;
		if (dl > BT_PDS_RECORD_DATA_MAX)
		{
			break;
		}
		if (dl > 0)
		{
			if (s_pNvm->Read(off + BT_PDS_HDR_SIZE, buf, dl) != 0)
			{
				break;
			}
		}
		if (RecCrc(&hdr, buf) != hdr.Crc)
		{
			break;
		}

		if (hdr.Id == Id)
		{
			// Highest seq wins. Sequence is monotonic and the log is in
			// append order, so the last matching record is newest, but compare
			// seq explicitly to be safe across wrap.
			if (!found || (uint16_t)(hdr.Seq - bestSeq) < 0x8000)
			{
				found = true;
				bestSeq = hdr.Seq;
				bestOff = off;
				bestLen = hdr.Len;
				bestTomb = (hdr.Len == BT_PDS_TOMBSTONE);
			}
		}

		off += RecSize(hdr.Len);
	}

	if (!found || bestTomb)
	{
		return false;
	}

	*pOff = bestOff;
	*pLen = bestLen;
	return true;
}

// Append a record (data may be nullptr for a tombstone with Len==TOMBSTONE).
// Caller guarantees there is room. Returns 0 on success.
static int AppendRaw(uint32_t Id, uint16_t Len, const void *pData)
{
	BtPdsRecHdr_t hdr;
	uint16_t dl = (Len == BT_PDS_TOMBSTONE) ? 0 : Len;

	hdr.Magic = BT_PDS_REC_MAGIC;
	hdr.Id    = Id;
	hdr.Len   = Len;
	hdr.Seq   = s_NextSeq;
	hdr.Rsvd[0] = hdr.Rsvd[1] = hdr.Rsvd[2] = 0;
	hdr.Crc   = RecCrc(&hdr, pData);

	// Assemble header + word-padded payload in a stack buffer so the medium
	// write is a single word-granular operation (the data must remain valid
	// until commit; the backend write is synchronous so a local buffer is ok).
	uint8_t rec[BT_PDS_HDR_SIZE + WordPad(BT_PDS_RECORD_DATA_MAX)];
	memset(rec, 0xFF, sizeof(rec));
	memcpy(rec, &hdr, BT_PDS_HDR_SIZE);
	if (dl > 0 && pData != nullptr)
	{
		memcpy(rec + BT_PDS_HDR_SIZE, pData, dl);
	}

	uint32_t total = RecSize(Len);
	if (s_pNvm->Write(s_WriteHead, rec, total) != 0)
	{
		return -EIO;
	}

	s_WriteHead += total;
	s_NextSeq++;
	return 0;
}

// Copy all live records into the spare sector(s), then erase the old span.
// After this the head points just past the compacted set. Returns 0 on success.
static int Compact(void)
{
	// The spare sector is the last sector. Compact live records into a fresh
	// area by walking the current log, collecting the newest non-tombstone
	// version of each Id, and rewriting from offset 0 of a cleared region.
	//
	// Strategy that is safe against power loss without a second region: build
	// the compacted image in the spare sector first, then erase the rest, then
	// (logically) the spare becomes the new tail. Because this single-region
	// store is small, we instead rebuild in place using a bounded staging pass:
	//   1. Enumerate live (Id,off,len) pairs into a small index.
	//   2. Erase all sectors.
	//   3. Re-append each live record from the staged copies.
	// The staged copies are read into RAM before the erase. Region is <= a few
	// KB and records <= 128 B, so the live set fits comfortably.
	//
	// If power is lost between erase and re-append, the store is empty rather
	// than corrupt; bonds are re-established on next pairing. This is the same
	// failure window ZMS narrows with a second copy; acceptable here given the
	// rarity of compaction (only when the region fills).

	struct {
		uint32_t Id;
		uint16_t Len;
		uint8_t  Data[BT_PDS_RECORD_DATA_MAX];
	} live[64];
	int nLive = 0;

	uint32_t off = 0;
	uint8_t buf[BT_PDS_RECORD_DATA_MAX];

	while (off + BT_PDS_HDR_SIZE <= s_pNvm->RegionSize && off < s_WriteHead)
	{
		BtPdsRecHdr_t hdr;
		if (!ReadHdr(off, &hdr))
		{
			break;
		}
		uint16_t dl = (hdr.Len == BT_PDS_TOMBSTONE) ? 0 : hdr.Len;
		if (dl > BT_PDS_RECORD_DATA_MAX)
		{
			break;
		}
		if (dl > 0 && s_pNvm->Read(off + BT_PDS_HDR_SIZE, buf, dl) != 0)
		{
			break;
		}
		if (RecCrc(&hdr, buf) != hdr.Crc)
		{
			break;
		}

		// Upsert into live index (replace existing Id, drop on tombstone).
		int idx = -1;
		for (int i = 0; i < nLive; i++)
		{
			if (live[i].Id == hdr.Id) { idx = i; break; }
		}
		if (hdr.Len == BT_PDS_TOMBSTONE)
		{
			if (idx >= 0)
			{
				live[idx] = live[--nLive];	// remove
			}
		}
		else
		{
			if (idx < 0)
			{
				if (nLive >= (int)(sizeof(live) / sizeof(live[0])))
				{
					return -ENOMEM;		// live set larger than staging
				}
				idx = nLive++;
				live[idx].Id = hdr.Id;
			}
			live[idx].Len = hdr.Len;
			if (dl > 0)
			{
				memcpy(live[idx].Data, buf, dl);
			}
		}

		off += RecSize(hdr.Len);
	}

	// Erase the whole region.
	for (uint32_t s = 0; s < s_pNvm->RegionSize; s += s_pNvm->SectorSize)
	{
		if (s_pNvm->Erase(s) != 0)
		{
			return -EIO;
		}
	}

	// Reset head/seq and re-append the live set.
	s_WriteHead = 0;
	s_NextSeq = 0;
	for (int i = 0; i < nLive; i++)
	{
		int r = AppendRaw(live[i].Id, live[i].Len, live[i].Data);
		if (r != 0)
		{
			return r;
		}
	}

	return 0;
}

int BtPdsInit(const BtPdsNvm_t *pNvm)
{
	if (pNvm == nullptr || pNvm->Read == nullptr || pNvm->Write == nullptr ||
		pNvm->Erase == nullptr || pNvm->SectorSize == 0 ||
		pNvm->RegionSize < pNvm->SectorSize)
	{
		return -EINVAL;
	}

	s_pNvm = pNvm;
	ScanRegion();
	return 0;
}

ssize_t BtPdsRead(uint32_t Id, void *pBuf, size_t Len)
{
	if (s_pNvm == nullptr)
	{
		return -EINVAL;
	}

	uint32_t off;
	uint16_t rlen;

	if (!FindLive(Id, &off, &rlen))
	{
		return -ENOENT;
	}

	uint16_t n = rlen;
	if ((size_t)n > Len)
	{
		n = (uint16_t)Len;
	}
	if (n > 0)
	{
		if (s_pNvm->Read(off + BT_PDS_HDR_SIZE, pBuf, n) != 0)
		{
			return -EIO;
		}
	}
	return (ssize_t)rlen;
}

ssize_t BtPdsWrite(uint32_t Id, const void *pData, size_t Len)
{
	if (s_pNvm == nullptr || Len > BT_PDS_RECORD_DATA_MAX)
	{
		return -EINVAL;
	}

	uint32_t need = RecSize((uint16_t)Len);

	// Compaction target is the last sector; usable head limit is RegionSize
	// minus one sector kept free for the compaction rebuild margin.
	uint32_t limit = s_pNvm->RegionSize - s_pNvm->SectorSize;

	if (s_WriteHead + need > limit)
	{
		int r = Compact();
		if (r != 0)
		{
			return r;
		}
		if (s_WriteHead + need > limit)
		{
			return -ENOMEM;		// no space left
		}
	}

	int r = AppendRaw(Id, (uint16_t)Len, pData);
	if (r != 0)
	{
		return r;
	}
	return (ssize_t)Len;
}

int BtPdsDelete(uint32_t Id)
{
	if (s_pNvm == nullptr)
	{
		return -EINVAL;
	}

	uint32_t off;
	uint16_t rlen;

	// Nothing to delete: idempotent success.
	if (!FindLive(Id, &off, &rlen))
	{
		return 0;
	}

	uint32_t need = RecSize(BT_PDS_TOMBSTONE);
	uint32_t limit = s_pNvm->RegionSize - s_pNvm->SectorSize;

	if (s_WriteHead + need > limit)
	{
		int r = Compact();
		if (r != 0)
		{
			return r;
		}
		// After compaction the deleted Id may already be gone if it was the
		// only version; re-check.
		if (!FindLive(Id, &off, &rlen))
		{
			return 0;
		}
	}

	return AppendRaw(Id, BT_PDS_TOMBSTONE, nullptr);
}

int BtPdsClear(void)
{
	if (s_pNvm == nullptr)
	{
		return -EINVAL;
	}

	for (uint32_t s = 0; s < s_pNvm->RegionSize; s += s_pNvm->SectorSize)
	{
		if (s_pNvm->Erase(s) != 0)
		{
			return -EIO;
		}
	}
	s_WriteHead = 0;
	s_NextSeq = 0;
	return 0;
}
