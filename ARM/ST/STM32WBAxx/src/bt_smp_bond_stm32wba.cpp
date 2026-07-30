/**-------------------------------------------------------------------------
@file	bt_smp_bond_stm32wba.cpp

@brief	Persistent bond storage glue for STM32WBA (ST host) builds.

		WBA differs from the SDC path in where the bond data comes from. On
		SDC, IOsonata's SMP produces each bond record and the generic RAM
		table persists individual slots. On WBA, ST's host owns the bonding
		table: the stack keeps it in a RAM cache and asks the platform to
		flush it through BLEPLAT_NvmStore. The cache is an opaque blob to
		IOsonata, and the restore preloads it before BleStack_Init.

		The cache is larger than one bt_pds record, so the image goes down
		as a run of chunk records under consecutive keys, and a manifest
		record written last commits it: the manifest names the length and a
		CRC over the whole image, so a restore takes the image entire or
		not at all. A flush torn by power loss leaves chunks that fail the
		CRC and restores nothing, which is a lost image, never a mixed one.

		Layering:
		    ST stack NVM cache  --BLEPLAT_NvmStore-->  THIS FILE
		      -> bt_pds.cpp (KV over a region)
		        -> Nvm over the WBA flash (bt_pds_stm32wba.cpp)

@author	Hoang Nguyen Hoan
@date	July 29, 2026

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
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>

#include "crc.h"
#include "bluetooth/bt_pds.h"

extern "C" int BtPdsWbaInit(void);

// The key range holding the image. The manifest sits at the base and chunk i
// at base + 1 + i. A dedicated range so nothing added later collides.
#define BT_WBA_NVM_KEY_BASE		0x57424E00u		// 'W''B''N', low byte the index

// One chunk fills a record.
#define BT_WBA_CHUNK_SIZE		BT_PDS_RECORD_DATA_MAX

// The manifest, written after every chunk of the image it names. Length
// zero is a stored empty image, distinct from no manifest at all.
typedef struct __Bt_Wba_Nvm_Manifest {
	uint32_t	Len;			//!< Image bytes across the chunks
	uint32_t	Crc;			//!< CRC32 over the image
} BtWbaNvmManifest_t;

static bool s_PdsReady;

/* The stack's NVM cache, captured when the application registers it. The
 * store call reports a portion pointer and a valid word count that always
 * counts from the cache's beginning, so persisting needs the beginning,
 * which only the registration provides.
 */
static uint64_t *s_pCache;
static uint16_t s_CacheMaxWords;

static int PdsEnsureReady(void)
{
	if (s_PdsReady)
	{
		return 0;
	}

	int r = BtPdsWbaInit();
	if (r != 0)
	{
		return r;
	}
	s_PdsReady = true;

	return 0;
}

// ---------------------------------------------------------------------------
// BLEPLAT_NvmStore: the ST stack flushing its NVM cache. Per bleplat.h, ptr
// points at the start of the portion that changed, which can be inside the
// cache, while size counts the valid 64 bit words from the cache's
// beginning. So what goes to the medium is the cache's first size words,
// from the base the application registered; ptr only says the call is about
// that cache. Without a registered base the portion pointer is all there
// is, which is only right when the stack happened to pass the beginning.
//
// Chunks first, manifest last: the manifest is the commit point, so a flush
// that loses power mid run leaves the previous manifest naming the previous
// image, or a manifest whose CRC the torn chunks fail. Either way a restore
// never hands the stack a mixture.
// ---------------------------------------------------------------------------
extern "C" void BLEPLAT_NvmStore(const uint64_t *ptr, uint16_t size)
{
	if (ptr == NULL)
	{
		return;
	}
	if (PdsEnsureReady() != 0)
	{
		return;
	}

	const uint64_t *base = s_pCache != NULL ? s_pCache : ptr;

	if (s_pCache != NULL &&
		(ptr < s_pCache || ptr >= s_pCache + s_CacheMaxWords))
	{
		// A pointer outside the registered cache is a call this file does
		// not understand; storing from either address would persist bytes
		// nobody meant.
		return;
	}
	if (s_CacheMaxWords != 0 && size > s_CacheMaxWords)
	{
		return;
	}

	const uint8_t *img = (const uint8_t *)base;
	size_t len = (size_t)size * sizeof(uint64_t);
	uint32_t idx = 0;

	for (size_t off = 0; off < len; off += BT_WBA_CHUNK_SIZE, idx++)
	{
		size_t n = len - off < BT_WBA_CHUNK_SIZE ? len - off : BT_WBA_CHUNK_SIZE;

		if (BtPdsWrite(BT_WBA_NVM_KEY_BASE + 1U + idx, img + off, n) !=
			(ssize_t)n)
		{
			// The image on the medium is now part old, part new, and the
			// manifest still names the old one, so a restore answers with
			// the old image until a later flush completes. Nothing to roll
			// back; the manifest not moving is the roll back.
			return;
		}
	}

	BtWbaNvmManifest_t man;

	man.Len = (uint32_t)len;
	man.Crc = crc32_ieee((uint8_t *)img, (int)len);

	(void)BtPdsWrite(BT_WBA_NVM_KEY_BASE, &man, sizeof(man));
}

// ---------------------------------------------------------------------------
// Restore the image into the buffer the ST stack will use, before
// BleStack_Init. Returns bytes loaded, 0 when there is nothing valid, which
// is a first boot or a torn flush.
// ---------------------------------------------------------------------------
extern "C" size_t BtSmpBondWbaRestore(void *pCacheBuf, size_t Capacity)
{
	if (pCacheBuf == NULL || Capacity == 0)
	{
		return 0;
	}
	if (PdsEnsureReady() != 0)
	{
		return 0;
	}

	BtWbaNvmManifest_t man;

	if (BtPdsRead(BT_WBA_NVM_KEY_BASE, &man, sizeof(man)) !=
		(ssize_t)sizeof(man))
	{
		return 0;
	}
	if (man.Len == 0 || man.Len > Capacity)
	{
		return 0;
	}

	uint8_t *img = (uint8_t *)pCacheBuf;
	uint32_t idx = 0;

	for (size_t off = 0; off < man.Len; off += BT_WBA_CHUNK_SIZE, idx++)
	{
		size_t n = man.Len - off < BT_WBA_CHUNK_SIZE ?
				   man.Len - off : BT_WBA_CHUNK_SIZE;

		if (BtPdsRead(BT_WBA_NVM_KEY_BASE + 1U + idx, img, n) != (ssize_t)n)
		{
			return 0;
		}
		img += n;
	}

	if (crc32_ieee((uint8_t *)pCacheBuf, (int)man.Len) != man.Crc)
	{
		// Chunks from two flushes. The whole image is refused rather than
		// handing the stack a mixture it would treat as its own state.
		return 0;
	}

	return man.Len;
}

// ---------------------------------------------------------------------------
// Commit trigger from the pairing complete handler. The stack flushes
// through BLEPLAT_NvmStore on its own schedule, so this is a marker kept so
// the integration point is explicit and greppable.
// ---------------------------------------------------------------------------
extern "C" void BtSmpBondWbaCommit(uint16_t ConnHdl)
{
	(void)ConnHdl;
}

// ---------------------------------------------------------------------------
// Register the stack's NVM cache and preload it, before BleStack_Init:
// BleStack_init_t takes the buffer, its capacity and the words already
// valid, and nvm_cache_size is exactly what this returns. Also the entry
// that pulls this object from the archive, so the strong BLEPLAT_NvmStore
// here wins over any weak stub.
//
// Returns the preloaded 64 bit words, 0 on a first boot, a torn flush, or
// a store that would not mount; the stack then starts empty, which is the
// same recovery either way.
// ---------------------------------------------------------------------------
extern "C" uint16_t BtSmpBondWbaCacheInit(uint64_t *pCache, uint16_t MaxWords)
{
	if (pCache == NULL || MaxWords == 0)
	{
		return 0;
	}

	s_pCache = pCache;
	s_CacheMaxWords = MaxWords;

	size_t n = BtSmpBondWbaRestore(pCache, (size_t)MaxWords *
								   sizeof(uint64_t));

	// Whole words by construction: the image was stored from a word count.
	return (uint16_t)(n / sizeof(uint64_t));
}

extern "C" int BtSmpBondWbaInit(void)
{
	return PdsEnsureReady();
}
