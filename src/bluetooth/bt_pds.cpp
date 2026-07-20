/**-------------------------------------------------------------------------
@file	bt_pds.cpp

@brief	Persistent Data Store for the Bluetooth peer manager.

		Log-structured, id-keyed record store over a fixed NVM region divided
		into sectors. One sector is kept erased for sector-at-a-time garbage
		collection. Records never cross a sector boundary.

		Garbage collection copies only the globally newest records from one
		victim sector into the erased destination sector. The victim is erased
		only after a completion marker is committed. If power is lost during the
		copy or erase, BtPdsInit resumes or safely discards the interrupted
		operation before the store accepts new writes. Until the completion
		marker commits, the victim still owns every record, so an unusable
		destination is simply erased and the collection retried later.

		Format 3. Record sequence numbers are 32 bit and never wrap within the
		endurance of any supported medium, so newest-record selection is a plain
		compare. Record and sector headers are protected by CRC16. Each sector
		header records a clear epoch: BtPdsClear commits by writing one sector
		header under the next epoch, and sectors under an older epoch are
		ignored and erased at the next init.

		The implementation uses a single record-sized scratch buffer. It does
		not allocate a live-record table or use the heap.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2016-2026 I-SYST inc.
----------------------------------------------------------------------------*/
#include <string.h>
#include <errno.h>
#include <limits.h>
#include <stdarg.h>

#include "bluetooth/bt_pds.h"

// Mount and scan tracing. Define BT_PDS_TRACE (a library build symbol) to
// route the store's mount decisions to the SysLog sink. Nothing is emitted
// unless BT_PDS_TRACE is defined.
#ifdef BT_PDS_TRACE
#include "syslog.h"
#define BT_PDS_TRC(...)		SysLogPrintf(SysLogGet(), __VA_ARGS__)
#else
#define BT_PDS_TRC(...)		((void)0)
#endif

#define BT_PDS_REC_MAGIC			0x53445042UL	// "BPDS"
#define BT_PDS_SECTOR_MAGIC		0x32534450UL	// "PDS2"
#define BT_PDS_TOMBSTONE			0xFFFFU
#define BT_PDS_FORMAT_VERSION		3U
#define BT_PDS_NO_SECTOR			0xFFFFU
#define BT_PDS_CTRL_GC_DONE_ID		0xFFFFFFFEUL
#define BT_PDS_ERASED_U32			0xFFFFFFFFUL

#ifndef BT_PDS_MAX_SECTORS
#define BT_PDS_MAX_SECTORS			16U
#endif

#pragma pack(push, 4)
typedef struct __Bt_Pds_Sector_Hdr {
	uint32_t	Magic;
	uint32_t	Generation;
	uint32_t	SourceGeneration;
	uint32_t	Epoch;
	uint16_t	SourceSector;
	uint16_t	Version;
	uint16_t	Crc;
	uint8_t		Rsvd[2];
} BtPdsSectorHdr_t;

typedef struct __Bt_Pds_Rec_Hdr {
	uint32_t	Magic;
	uint32_t	Id;
	uint32_t	Seq;
	uint16_t	Len;
	uint16_t	Crc;
} BtPdsRecHdr_t;
#pragma pack(pop)

static_assert(sizeof(BtPdsSectorHdr_t) == 24, "sector header layout");
static_assert(sizeof(BtPdsRecHdr_t) == 16, "record header layout");

#define BT_PDS_SECTOR_HDR_SIZE	((uint32_t)sizeof(BtPdsSectorHdr_t))
#define BT_PDS_REC_HDR_SIZE		((uint32_t)sizeof(BtPdsRecHdr_t))

typedef struct __Bt_Pds_Sector_Info {
	bool		Valid;
	bool		Erased;
	bool		Writable;
	bool		GcDone;
	uint32_t	Generation;
	uint32_t	SourceGeneration;
	uint16_t	SourceSector;
	uint32_t	Head;
} BtPdsSectorInfo_t;

typedef struct __Bt_Pds_Latest {
	bool		Found;
	bool		Tombstone;
	uint32_t	Seq;
	uint16_t	Len;
	uint16_t	Sector;
	uint32_t	Off;
} BtPdsLatest_t;

static NvmIO *s_pNvm;
static BtPdsSectorInfo_t s_Sectors[BT_PDS_MAX_SECTORS];
static uint16_t s_SectorCount;
static uint16_t s_ActiveSector;
static uint32_t s_NextSeq;
static uint32_t s_NextGeneration;
static uint32_t s_Epoch;
static bool s_SeqFound;
static uint32_t s_MaxSeq;

static inline uint32_t WordPad(uint32_t x)
{
	return (x + 3U) & ~3UL;
}

static inline uint32_t SectorBase(uint16_t Sector)
{
	return (uint32_t)Sector * s_pNvm->LogicalSectorSize();
}

static inline uint32_t SectorEnd(uint16_t Sector)
{
	return SectorBase(Sector) + s_pNvm->LogicalSectorSize();
}

static inline uint32_t SectorDataStart(uint16_t Sector)
{
	return SectorBase(Sector) + BT_PDS_SECTOR_HDR_SIZE;
}

static uint16_t Crc16(const void *pData, uint32_t Len, uint16_t Seed)
{
	// CRC16-CCITT, polynomial 0x1021.
	const uint8_t *p = (const uint8_t *)pData;
	uint16_t crc = Seed;

	for (uint32_t i = 0; i < Len; i++)
	{
		crc ^= (uint16_t)((uint16_t)p[i] << 8);
		for (int b = 0; b < 8; b++)
		{
			crc = (crc & 0x8000U) ?
				(uint16_t)((crc << 1) ^ 0x1021U) :
				(uint16_t)(crc << 1);
		}
	}
	return crc;
}

static uint16_t SectorHdrCrc(const BtPdsSectorHdr_t *pHdr)
{
	uint16_t crc = Crc16(&pHdr->Generation, sizeof(pHdr->Generation), 0xFFFFU);
	crc = Crc16(&pHdr->SourceGeneration, sizeof(pHdr->SourceGeneration), crc);
	crc = Crc16(&pHdr->Epoch, sizeof(pHdr->Epoch), crc);
	crc = Crc16(&pHdr->SourceSector, sizeof(pHdr->SourceSector), crc);
	crc = Crc16(&pHdr->Version, sizeof(pHdr->Version), crc);
	return crc;
}

static uint16_t RecCrc(const BtPdsRecHdr_t *pHdr, const void *pData)
{
	uint16_t crc = Crc16(&pHdr->Id, sizeof(pHdr->Id), 0xFFFFU);
	crc = Crc16(&pHdr->Len, sizeof(pHdr->Len), crc);
	crc = Crc16(&pHdr->Seq, sizeof(pHdr->Seq), crc);
	if (pHdr->Len != BT_PDS_TOMBSTONE && pHdr->Len > 0U)
	{
		crc = Crc16(pData, pHdr->Len, crc);
	}
	return crc;
}

static uint32_t RecSize(uint16_t Len)
{
	uint32_t dataLen = Len == BT_PDS_TOMBSTONE ? 0U : Len;
	return BT_PDS_REC_HDR_SIZE + WordPad(dataLen);
}

static bool IsErased(const void *pData, uint32_t Len)
{
	const uint8_t *p = (const uint8_t *)pData;

	for (uint32_t i = 0; i < Len; i++)
	{
		if (p[i] != 0xFFU)
		{
			return false;
		}
	}
	return true;
}

static bool ReadSectorHdr(uint16_t Sector, BtPdsSectorHdr_t *pHdr)
{
	if (s_pNvm->Read(SectorBase(Sector), pHdr, sizeof(*pHdr)) < 0)
	{
		return false;
	}

	return pHdr->Magic == BT_PDS_SECTOR_MAGIC &&
		pHdr->Version == BT_PDS_FORMAT_VERSION &&
		(pHdr->SourceSector == BT_PDS_NO_SECTOR ||
		 pHdr->SourceSector < s_SectorCount) &&
		SectorHdrCrc(pHdr) == pHdr->Crc;
}

static bool ReadRecord(uint16_t Sector, uint32_t Off, BtPdsRecHdr_t *pHdr,
					   uint8_t *pData)
{
	uint32_t end = SectorEnd(Sector);

	if (Off < SectorDataStart(Sector) ||
		Off + BT_PDS_REC_HDR_SIZE > end ||
		s_pNvm->Read(Off, pHdr, BT_PDS_REC_HDR_SIZE) < 0 ||
		pHdr->Magic != BT_PDS_REC_MAGIC)
	{
		return false;
	}

	uint16_t dataLen = pHdr->Len == BT_PDS_TOMBSTONE ? 0U : pHdr->Len;
	if (dataLen > BT_PDS_RECORD_DATA_MAX ||
		Off + RecSize(pHdr->Len) > end)
	{
		return false;
	}

	if (dataLen > 0U &&
		s_pNvm->Read(Off + BT_PDS_REC_HDR_SIZE, pData, dataLen) < 0)
	{
		return false;
	}

	return RecCrc(pHdr, pData) == pHdr->Crc;
}

static void SectorInfoReset(uint16_t Sector)
{
	memset(&s_Sectors[Sector], 0, sizeof(s_Sectors[Sector]));
	s_Sectors[Sector].Erased = true;
	s_Sectors[Sector].Writable = true;
	s_Sectors[Sector].SourceSector = BT_PDS_NO_SECTOR;
	s_Sectors[Sector].Head = SectorBase(Sector);
}

static int ScanSector(uint16_t Sector)
{
	BtPdsSectorHdr_t sectorHdr;
	BtPdsSectorInfo_t *pInfo = &s_Sectors[Sector];

	memset(pInfo, 0, sizeof(*pInfo));
	pInfo->SourceSector = BT_PDS_NO_SECTOR;
	pInfo->Head = SectorBase(Sector);

	if (s_pNvm->Read(SectorBase(Sector), &sectorHdr, sizeof(sectorHdr)) < 0)
	{
		BT_PDS_TRC("scan %u: hdr read -EIO\r\n", Sector);
		return -EIO;
	}

	if (IsErased(&sectorHdr, sizeof(sectorHdr)))
	{
		BT_PDS_TRC("scan %u: erased\r\n", Sector);
		SectorInfoReset(Sector);
		return 0;
	}

	if (!ReadSectorHdr(Sector, &sectorHdr))
	{
		BT_PDS_TRC("scan %u: hdr invalid (magic/ver/crc)\r\n", Sector);
		pInfo->Valid = false;
		pInfo->Erased = false;
		pInfo->Writable = false;
		pInfo->Head = SectorEnd(Sector);
		return 0;
	}

	if (sectorHdr.Epoch != s_Epoch)
	{
		BT_PDS_TRC("scan %u: epoch %lu != cur %lu\r\n", Sector,
				   (unsigned long)sectorHdr.Epoch, (unsigned long)s_Epoch);
		// Sector under an earlier epoch: cleared by BtPdsClear but not yet
		// erased. Its records are ignored and it is erased at init.
		pInfo->Valid = false;
		pInfo->Erased = false;
		pInfo->Writable = false;
		pInfo->Head = SectorEnd(Sector);
		return 0;
	}

	pInfo->Valid = true;
	pInfo->Erased = false;
	pInfo->Writable = true;
	pInfo->Generation = sectorHdr.Generation;
	pInfo->SourceGeneration = sectorHdr.SourceGeneration;
	pInfo->SourceSector = sectorHdr.SourceSector;
	pInfo->Head = SectorDataStart(Sector);
	BT_PDS_TRC("scan %u: valid gen=%lu src=%u epoch=%lu\r\n", Sector,
			   (unsigned long)sectorHdr.Generation, sectorHdr.SourceSector,
			   (unsigned long)sectorHdr.Epoch);

	uint32_t off = pInfo->Head;
	uint8_t data[BT_PDS_RECORD_DATA_MAX];

	while (off + BT_PDS_REC_HDR_SIZE <= SectorEnd(Sector))
	{
		uint32_t word;
		if (s_pNvm->Read(off, &word, sizeof(word)) < 0)
		{
			return -EIO;
		}

		if (word == BT_PDS_ERASED_U32)
		{
			pInfo->Head = off;
			return 0;
		}

		BtPdsRecHdr_t hdr;
		if (!ReadRecord(Sector, off, &hdr, data))
		{
			// Never append over a partial record on one-way programmable media.
			BT_PDS_TRC("scan %u: torn record at off=%lu, sector closed\r\n",
					   Sector, (unsigned long)off);
			pInfo->Writable = false;
			pInfo->Head = SectorEnd(Sector);
			return 0;
		}

		if (hdr.Id == BT_PDS_CTRL_GC_DONE_ID && hdr.Len == 0U)
		{
			pInfo->GcDone = true;
		}

		if (!s_SeqFound || hdr.Seq > s_MaxSeq)
		{
			s_SeqFound = true;
			s_MaxSeq = hdr.Seq;
		}

		off += RecSize(hdr.Len);
		pInfo->Head = off;
	}

	BT_PDS_TRC("scan %u: full, head at end, records=%lu\r\n", Sector,
			   (unsigned long)((pInfo->Head - SectorDataStart(Sector))));
	pInfo->Writable = false;
	pInfo->Head = SectorEnd(Sector);
	return 0;
}

static int EraseSector(uint16_t Sector)
{
	if (s_pNvm->Erase(SectorBase(Sector), s_pNvm->LogicalSectorSize()) != 0)
	{
		return -EIO;
	}
	SectorInfoReset(Sector);
	return 0;
}

static int WriteSectorHdr(uint16_t Sector, uint16_t SourceSector,
						  uint32_t SourceGeneration)
{
	BtPdsSectorHdr_t hdr;
	memset(&hdr, 0xFF, sizeof(hdr));

	hdr.Magic = BT_PDS_SECTOR_MAGIC;
	hdr.Generation = s_NextGeneration++;
	hdr.SourceGeneration = SourceGeneration;
	hdr.Epoch = s_Epoch;
	hdr.SourceSector = SourceSector;
	hdr.Version = BT_PDS_FORMAT_VERSION;
	hdr.Rsvd[0] = hdr.Rsvd[1] = 0U;
	hdr.Crc = SectorHdrCrc(&hdr);

	if (s_pNvm->Write(SectorBase(Sector), &hdr, sizeof(hdr)) < 0)
	{
		return -EIO;
	}

	BtPdsSectorInfo_t *pInfo = &s_Sectors[Sector];
	memset(pInfo, 0, sizeof(*pInfo));
	pInfo->Valid = true;
	pInfo->Erased = false;
	pInfo->Writable = true;
	pInfo->Generation = hdr.Generation;
	pInfo->SourceGeneration = SourceGeneration;
	pInfo->SourceSector = SourceSector;
	pInfo->Head = SectorDataStart(Sector);
	return 0;
}

static int AppendToSector(uint16_t Sector, uint32_t Id, uint16_t Len,
						  const void *pData)
{
	BtPdsSectorInfo_t *pInfo = &s_Sectors[Sector];
	uint16_t dataLen = Len == BT_PDS_TOMBSTONE ? 0U : Len;
	uint32_t total = RecSize(Len);

	if (!pInfo->Valid || !pInfo->Writable ||
		pInfo->Head + total > SectorEnd(Sector))
	{
		return -ENOMEM;
	}

	BtPdsRecHdr_t hdr;
	memset(&hdr, 0, sizeof(hdr));
	hdr.Magic = BT_PDS_REC_MAGIC;
	hdr.Id = Id;
	hdr.Len = Len;
	hdr.Seq = s_NextSeq;
	hdr.Crc = RecCrc(&hdr, pData);

	alignas(uint32_t) uint8_t rec[
		BT_PDS_REC_HDR_SIZE + WordPad(BT_PDS_RECORD_DATA_MAX)];
	memset(rec, 0xFF, sizeof(rec));
	memcpy(rec, &hdr, sizeof(hdr));
	if (dataLen > 0U && pData != nullptr)
	{
		memcpy(rec + BT_PDS_REC_HDR_SIZE, pData, dataLen);
	}

	if (s_pNvm->Write(pInfo->Head, rec, total) < 0)
	{
		pInfo->Writable = false;
		return -EIO;
	}

	pInfo->Head += total;
	s_NextSeq++;
	return 0;
}

static bool FindLatest(uint32_t Id, BtPdsLatest_t *pLatest)
{
	BtPdsLatest_t latest;
	memset(&latest, 0, sizeof(latest));
	uint8_t data[BT_PDS_RECORD_DATA_MAX];

	for (uint16_t sector = 0; sector < s_SectorCount; sector++)
	{
		if (!s_Sectors[sector].Valid)
		{
			continue;
		}

		uint32_t off = SectorDataStart(sector);
		while (off < s_Sectors[sector].Head &&
			   off + BT_PDS_REC_HDR_SIZE <= SectorEnd(sector))
		{
			BtPdsRecHdr_t hdr;
			if (!ReadRecord(sector, off, &hdr, data))
			{
				break;
			}

			if (hdr.Id == Id && hdr.Id != BT_PDS_CTRL_GC_DONE_ID &&
				(!latest.Found || hdr.Seq > latest.Seq))
			{
				latest.Found = true;
				latest.Tombstone = hdr.Len == BT_PDS_TOMBSTONE;
				latest.Seq = hdr.Seq;
				latest.Len = hdr.Len;
				latest.Sector = sector;
				latest.Off = off;
			}
			off += RecSize(hdr.Len);
		}
	}

	if (pLatest != nullptr)
	{
		*pLatest = latest;
	}
	return latest.Found;
}

static bool HasRecordOutsideSector(uint32_t Id, uint16_t ExcludedSector)
{
	uint8_t data[BT_PDS_RECORD_DATA_MAX];

	for (uint16_t sector = 0; sector < s_SectorCount; sector++)
	{
		if (sector == ExcludedSector || !s_Sectors[sector].Valid)
		{
			continue;
		}

		uint32_t off = SectorDataStart(sector);
		while (off < s_Sectors[sector].Head &&
			   off + BT_PDS_REC_HDR_SIZE <= SectorEnd(sector))
		{
			BtPdsRecHdr_t hdr;
			if (!ReadRecord(sector, off, &hdr, data))
			{
				break;
			}
			if (hdr.Id == Id && hdr.Id != BT_PDS_CTRL_GC_DONE_ID)
			{
				return true;
			}
			off += RecSize(hdr.Len);
		}
	}
	return false;
}

static uint32_t SectorCopyBytes(uint16_t Victim)
{
	uint32_t bytes = 0U;
	uint8_t data[BT_PDS_RECORD_DATA_MAX];
	uint32_t off = SectorDataStart(Victim);

	while (off < s_Sectors[Victim].Head &&
		   off + BT_PDS_REC_HDR_SIZE <= SectorEnd(Victim))
	{
		BtPdsRecHdr_t hdr;
		if (!ReadRecord(Victim, off, &hdr, data))
		{
			break;
		}

		if (hdr.Id != BT_PDS_CTRL_GC_DONE_ID)
		{
			BtPdsLatest_t latest;
			if (FindLatest(hdr.Id, &latest) &&
				latest.Sector == Victim && latest.Off == off &&
				(hdr.Len != BT_PDS_TOMBSTONE ||
				 HasRecordOutsideSector(hdr.Id, Victim)))
			{
				bytes += RecSize(hdr.Len);
			}
		}
		off += RecSize(hdr.Len);
	}
	return bytes;
}

static int CopyLatestFromSector(uint16_t Victim, uint16_t Destination)
{
	uint8_t data[BT_PDS_RECORD_DATA_MAX];
	uint32_t off = SectorDataStart(Victim);

	while (off < s_Sectors[Victim].Head &&
		   off + BT_PDS_REC_HDR_SIZE <= SectorEnd(Victim))
	{
		BtPdsRecHdr_t hdr;
		if (!ReadRecord(Victim, off, &hdr, data))
		{
			// An unreadable record ends the sector log, exactly as it does
			// for ScanSector, FindLatest and SectorCopyBytes. A victim that
			// was closed by a torn append ends this way and everything
			// after the tear is uncommitted, so the copy is complete here.
			break;
		}

		uint32_t next = off + RecSize(hdr.Len);
		if (hdr.Id != BT_PDS_CTRL_GC_DONE_ID)
		{
			BtPdsLatest_t latest;
			if (FindLatest(hdr.Id, &latest) &&
				latest.Sector == Victim && latest.Off == off &&
				(hdr.Len != BT_PDS_TOMBSTONE ||
				 HasRecordOutsideSector(hdr.Id, Victim)))
			{
				const void *pData =
					hdr.Len == BT_PDS_TOMBSTONE ? nullptr : data;
				int result = AppendToSector(Destination, hdr.Id, hdr.Len, pData);
				if (result != 0)
				{
					return result;
				}
			}
		}
		off = next;
	}
	return 0;
}

static int CompleteGc(uint16_t Destination)
{
	BtPdsSectorInfo_t *pDest = &s_Sectors[Destination];
	uint16_t source = pDest->SourceSector;

	if (source == BT_PDS_NO_SECTOR || source >= s_SectorCount ||
		!s_Sectors[source].Valid ||
		s_Sectors[source].Generation != pDest->SourceGeneration)
	{
		return 0;
	}

	if (!pDest->GcDone)
	{
		int result;

		if (!pDest->Writable)
		{
			// A torn record from the interrupted copy closed the destination.
			result = -EIO;
		}
		else
		{
			result = CopyLatestFromSector(source, Destination);
			if (result == 0)
			{
				result = AppendToSector(Destination, BT_PDS_CTRL_GC_DONE_ID,
										0U, nullptr);
			}
		}

		if (result != 0)
		{
			// Until the completion marker commits, the source sector still
			// owns every record and the destination holds only value-identical
			// duplicates. No user write can land in the destination before the
			// collection completes, so it is erased and the collection is
			// retried later instead of leaving the store unmountable.
			result = EraseSector(Destination);
			return result != 0 ? result : -EAGAIN;
		}
		pDest->GcDone = true;
	}

	return EraseSector(source);
}

static int ScanAll(void)
{
	s_SeqFound = false;
	s_MaxSeq = 0U;
	s_NextGeneration = 1U;
	s_Epoch = 0U;

	// The current epoch is the highest one present on the medium. Sectors
	// under an earlier epoch were cleared by BtPdsClear before power was
	// lost and must stay invisible.
	for (uint16_t sector = 0; sector < s_SectorCount; sector++)
	{
		BtPdsSectorHdr_t hdr;
		if (ReadSectorHdr(sector, &hdr) && hdr.Epoch > s_Epoch)
		{
			s_Epoch = hdr.Epoch;
		}
	}

	for (uint16_t sector = 0; sector < s_SectorCount; sector++)
	{
		int result = ScanSector(sector);
		if (result != 0)
		{
			return result;
		}
		if (s_Sectors[sector].Valid &&
			s_Sectors[sector].Generation >= s_NextGeneration)
		{
			s_NextGeneration = s_Sectors[sector].Generation + 1U;
		}
	}

	s_NextSeq = s_SeqFound ? s_MaxSeq + 1U : 0U;
	return 0;
}

static int RecoverGc(void)
{
	// Only one GC can be active because normal operation keeps one erased
	// sector. Completed historical headers are ignored when their source sector
	// has already been erased or reused with a different generation.
	for (uint16_t sector = 0; sector < s_SectorCount; sector++)
	{
		if (s_Sectors[sector].Valid &&
			s_Sectors[sector].SourceSector != BT_PDS_NO_SECTOR)
		{
			int result = CompleteGc(sector);
			if (result != 0 && result != -EAGAIN)
			{
				return result;
			}
		}
	}
	return ScanAll();
}

static uint16_t FindErasedSector(void)
{
	for (uint16_t sector = 0; sector < s_SectorCount; sector++)
	{
		if (s_Sectors[sector].Erased)
		{
			return sector;
		}
	}
	return BT_PDS_NO_SECTOR;
}

static uint16_t ErasedSectorCount(void)
{
	uint16_t count = 0U;
	for (uint16_t sector = 0; sector < s_SectorCount; sector++)
	{
		if (s_Sectors[sector].Erased)
		{
			count++;
		}
	}
	return count;
}

static void SelectActiveSector(void)
{
	s_ActiveSector = BT_PDS_NO_SECTOR;
	uint32_t newestGeneration = 0U;

	for (uint16_t sector = 0; sector < s_SectorCount; sector++)
	{
		if (s_Sectors[sector].Valid &&
			(s_ActiveSector == BT_PDS_NO_SECTOR ||
			 s_Sectors[sector].Generation > newestGeneration))
		{
			s_ActiveSector = sector;
			newestGeneration = s_Sectors[sector].Generation;
		}
	}
}

static int StartSector(uint16_t Sector)
{
	if (Sector == BT_PDS_NO_SECTOR)
	{
		return -ENOMEM;
	}
	int result = WriteSectorHdr(Sector, BT_PDS_NO_SECTOR, 0U);
	if (result == 0)
	{
		s_ActiveSector = Sector;
	}
	return result;
}

static int GarbageCollect(uint32_t Need)
{
	uint16_t destination = FindErasedSector();
	if (destination == BT_PDS_NO_SECTOR)
	{
		// Expected only after an interrupted erase. A sector owning no globally
		// newest record can be reclaimed without copying.
		for (uint16_t sector = 0; sector < s_SectorCount; sector++)
		{
			if (s_Sectors[sector].Valid && SectorCopyBytes(sector) == 0U)
			{
				int result = EraseSector(sector);
				if (result != 0)
				{
					return result;
				}
				destination = sector;
				break;
			}
		}
	}

	if (destination == BT_PDS_NO_SECTOR)
	{
		return -ENOMEM;
	}

	uint32_t capacity = s_pNvm->LogicalSectorSize() - BT_PDS_SECTOR_HDR_SIZE;
	uint32_t doneSize = RecSize(0U);
	uint16_t victim = BT_PDS_NO_SECTOR;
	uint32_t bestBytes = UINT_MAX;

	for (uint16_t sector = 0; sector < s_SectorCount; sector++)
	{
		if (!s_Sectors[sector].Valid || sector == destination)
		{
			continue;
		}

		uint32_t copyBytes = SectorCopyBytes(sector);
		if (copyBytes + doneSize + Need <= capacity &&
			copyBytes < bestBytes)
		{
			victim = sector;
			bestBytes = copyBytes;
		}
	}

	if (victim == BT_PDS_NO_SECTOR)
	{
		return -ENOMEM;
	}

	int result = WriteSectorHdr(destination, victim,
								s_Sectors[victim].Generation);
	if (result != 0)
	{
		return result;
	}

	result = CompleteGc(destination);
	if (result != 0)
	{
		// -EAGAIN reports an aborted copy. The destination was erased and the
		// store stays consistent, but the requested write did not happen.
		return result == -EAGAIN ? -EIO : result;
	}

	s_ActiveSector = destination;
	return 0;
}

static int EnsureSpace(uint32_t Need)
{
	if (Need > s_pNvm->LogicalSectorSize() - BT_PDS_SECTOR_HDR_SIZE)
	{
		return -ENOMEM;
	}

	if (s_ActiveSector != BT_PDS_NO_SECTOR &&
		s_Sectors[s_ActiveSector].Valid &&
		s_Sectors[s_ActiveSector].Writable &&
		s_Sectors[s_ActiveSector].Head + Need <= SectorEnd(s_ActiveSector))
	{
		return 0;
	}

	// Start another sector only while two erased sectors remain. This preserves
	// one spare for the next sector-at-a-time garbage collection.
	if (ErasedSectorCount() >= 2U)
	{
		return StartSector(FindErasedSector());
	}

	return GarbageCollect(Need);
}

static int FormatStore(void)
{
	for (uint16_t sector = 0; sector < s_SectorCount; sector++)
	{
		int result = EraseSector(sector);
		if (result != 0)
		{
			return result;
		}
	}

	s_NextSeq = 0U;
	s_NextGeneration = 1U;
	s_Epoch = 0U;
	return StartSector(0U);
}

int BtPdsInit(NvmIO * const pNvm)
{
	if (pNvm == nullptr)
	{
		return -EINVAL;
	}

	// The engine partitions the region into logical sectors for garbage
	// collection. On erase-write media the logical sector equals the physical
	// erase unit; on a rewritable medium it may be a larger multiple of it.
	uint32_t sectorSize = pNvm->LogicalSectorSize();
	uint32_t eraseSize = pNvm->EraseSize();
	uint64_t regionSize = pNvm->Size();
	BT_PDS_TRC("pds init: region=%lu sector=%lu erase=%lu gran=%lu hdr=%lu\r\n",
			   (unsigned long)regionSize, (unsigned long)sectorSize,
			   (unsigned long)eraseSize, (unsigned long)pNvm->WriteGran(),
			   (unsigned long)BT_PDS_SECTOR_HDR_SIZE);
	// A logical sector must hold a header, the region must fit at least two
	// sectors and divide evenly, the write unit must be a word, and a logical
	// sector must span whole physical erase units so erasing one sector is a
	// valid erase range.
	if (sectorSize <= BT_PDS_SECTOR_HDR_SIZE ||
		regionSize < (uint64_t)sectorSize * 2U ||
		(regionSize % sectorSize) != 0U ||
		eraseSize == 0U ||
		(sectorSize % eraseSize) != 0U ||
		pNvm->WriteGran() != 4U)
	{
		BT_PDS_TRC("pds init: geometry rejected -EINVAL\r\n");
		return -EINVAL;
	}

	uint32_t sectorCount = (uint32_t)(regionSize / sectorSize);
	if (sectorCount > BT_PDS_MAX_SECTORS)
	{
		BT_PDS_TRC("pds init: sectorCount=%lu > max=%lu -EINVAL\r\n",
				   (unsigned long)sectorCount, (unsigned long)BT_PDS_MAX_SECTORS);
		return -EINVAL;
	}
	BT_PDS_TRC("pds init: sectorCount=%lu\r\n", (unsigned long)sectorCount);

	s_pNvm = pNvm;
	s_SectorCount = (uint16_t)sectorCount;
	s_ActiveSector = BT_PDS_NO_SECTOR;

	// Development format 1 began directly with a record and had no
	// recoverable sector transaction; it is detected here and cleared once.
	// Development format 2 sectors fail header validation (layout and
	// version changed) and are erased by the sweep below. Neither can be
	// upgraded in place while preserving the power loss guarantee.
	uint32_t firstWord = BT_PDS_ERASED_U32;
	if (s_pNvm->Read(0U, &firstWord, sizeof(firstWord)) < 0)
	{
		BT_PDS_TRC("pds init: read word0 failed -EIO\r\n");
		return -EIO;
	}
	BT_PDS_TRC("pds init: word0=0x%08lx\r\n", (unsigned long)firstWord);
	if (firstWord == BT_PDS_REC_MAGIC)
	{
		BT_PDS_TRC("pds init: format1 detected, formatting\r\n");
		return FormatStore();
	}

	int result = ScanAll();
	if (result != 0)
	{
		BT_PDS_TRC("pds init: ScanAll failed %d\r\n", result);
		return result;
	}

	// A partial sector header contains no committed records and its source
	// sector still owns the data, so the partial destination can be erased
	// safely. Sectors under an earlier epoch were cleared by BtPdsClear and
	// are erased here as well.
	for (uint16_t sector = 0; sector < s_SectorCount; sector++)
	{
		if (!s_Sectors[sector].Valid && !s_Sectors[sector].Erased)
		{
			BT_PDS_TRC("pds init: erase invalid sector %u\r\n", sector);
			result = EraseSector(sector);
			if (result != 0)
			{
				BT_PDS_TRC("pds init: EraseSector %u failed %d\r\n",
						   sector, result);
				return result;
			}
		}
	}

	result = RecoverGc();
	if (result != 0)
	{
		BT_PDS_TRC("pds init: RecoverGc failed %d\r\n", result);
		return result;
	}

	SelectActiveSector();
	if (s_ActiveSector == BT_PDS_NO_SECTOR)
	{
		uint16_t spare = FindErasedSector();
		BT_PDS_TRC("pds init: no active sector, StartSector(%u)\r\n", spare);
		return StartSector(spare);
	}
	BT_PDS_TRC("pds init: ok, active sector=%u\r\n", s_ActiveSector);
	return 0;
}

ssize_t BtPdsRead(uint32_t Id, void *pBuf, size_t Len)
{
	if (s_pNvm == nullptr || Id == BT_PDS_CTRL_GC_DONE_ID)
	{
		return -EINVAL;
	}

	BtPdsLatest_t latest;
	if (!FindLatest(Id, &latest) || latest.Tombstone)
	{
		return -ENOENT;
	}

	uint16_t copyLen = latest.Len > Len ? (uint16_t)Len : latest.Len;
	if (copyLen > 0U)
	{
		if (pBuf == nullptr ||
			s_pNvm->Read(latest.Off + BT_PDS_REC_HDR_SIZE,
						 pBuf, copyLen) < 0)
		{
			return -EIO;
		}
	}
	return (ssize_t)latest.Len;
}

ssize_t BtPdsWrite(uint32_t Id, const void *pData, size_t Len)
{
	if (s_pNvm == nullptr || Id == BT_PDS_CTRL_GC_DONE_ID ||
		Len > BT_PDS_RECORD_DATA_MAX ||
		(Len > 0U && pData == nullptr))
	{
		return -EINVAL;
	}

	uint32_t need = RecSize((uint16_t)Len);
	int result = EnsureSpace(need);
	if (result != 0)
	{
		return result;
	}

	result = AppendToSector(s_ActiveSector, Id, (uint16_t)Len, pData);
	return result == 0 ? (ssize_t)Len : result;
}

int BtPdsDelete(uint32_t Id)
{
	if (s_pNvm == nullptr || Id == BT_PDS_CTRL_GC_DONE_ID)
	{
		return -EINVAL;
	}

	BtPdsLatest_t latest;
	if (!FindLatest(Id, &latest) || latest.Tombstone)
	{
		return 0;
	}

	int result = EnsureSpace(RecSize(BT_PDS_TOMBSTONE));
	if (result != 0)
	{
		return result;
	}

	return AppendToSector(s_ActiveSector, Id, BT_PDS_TOMBSTONE, nullptr);
}

int BtPdsClear(void)
{
	if (s_pNvm == nullptr)
	{
		return -EINVAL;
	}

	uint16_t spare = FindErasedSector();
	if (spare == BT_PDS_NO_SECTOR)
	{
		// The atomic clear needs one erased spare to commit the new epoch
		// onto. Without it the only option is a sequential format, which is
		// not power-loss atomic, so the documented all-or-nothing guarantee
		// cannot be met. Report the degraded state instead of silently
		// weakening the guarantee. BtPdsForceFormat performs the non-atomic
		// wipe when the caller accepts that risk.
		return -ENOSPC;
	}

	// Commit point of the clear. Once a sector header under the next epoch
	// is on the medium, every sector under an earlier epoch is ignored and
	// erased at the next init, so a power loss after this write cannot bring
	// cleared records back. A power loss before it leaves the store
	// unchanged.
	s_Epoch++;
	int result = StartSector(spare);
	if (result != 0)
	{
		return result;
	}

	for (uint16_t sector = 0; sector < s_SectorCount; sector++)
	{
		if (sector != spare && !s_Sectors[sector].Erased)
		{
			result = EraseSector(sector);
			if (result != 0)
			{
				return result;
			}
		}
	}
	return 0;
}

int BtPdsForceFormat(void)
{
	if (s_pNvm == nullptr)
	{
		return -EINVAL;
	}

	// Explicitly destructive: erases sectors sequentially and is not power
	// loss atomic. For use only when BtPdsClear returned -ENOSPC and the
	// caller accepts a non-atomic wipe.
	return FormatStore();
}
