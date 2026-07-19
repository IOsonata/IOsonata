/**-------------------------------------------------------------------------
@example	pds_test.cpp

@brief	Host test harness for the bt_pds peer data store.

Builds and runs on a PC, not on a target board. It mounts the store on a
RAM medium with NOR semantics (programming only clears bits, ascending
order) and drives it through a reference model plus power-cut injection:
the Nth programming operation (write or erase) is torn at a chosen byte
offset, then every following operation fails until reboot.

Build and run on the host:

  g++ -std=gnu++23 -O1 -I include -o test_pds \
      exemples/bluetooth/test_pds.cpp src/bluetooth/bt_pds.cpp
  ./test_pds

Coverage:
  - Basic semantics: write/read/update/delete, tombstone vs zero length,
    truncated read reporting, reserved id, oversize rejection.
  - Churn: 20K writes over 2 sectors and 100K over 4 sectors against a
    reference model, with remount checks.
  - Fill to -ENOMEM and update behavior at capacity.
  - Garbage collection cut sweep: a power cut at every programming
    operation of a collecting write, three tear sizes each. After reboot
    the store must mount and every committed value must read back.
  - Clear cut sweep: a power cut at every programming operation of
    BtPdsClear. After reboot the store must hold all records or none.

The harness prints RESULT: ALL PASS on success and a FAIL line per
violated check otherwise. This file is host-only and is not referenced
by any board project.

@author	Hoang Nguyen Hoan
@date	Jul 19, 2026

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
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <cerrno>
#include <vector>
#include <map>

#include "bluetooth/bt_pds.h"

#define SECTOR_SIZE		512U
#define MAX_REGION		(16U * SECTOR_SIZE)

static uint8_t s_Flash[MAX_REGION];
static uint32_t s_RegionSize;
static int s_CutCountdown = -1;	// -1 disabled; 0 means next op is cut
static uint32_t s_CutKeep;		// bytes of the cut op that still land
static bool s_Dead;
static int s_OpCount;

// NOR model medium as an NvmIO. Programming only clears bits, writes land in
// ascending order, and a cut at the Nth program or erase tears that operation
// and kills the medium until the next PowerOn.
class MockNvm : public NvmIO {
public:
	MockNvm() { Region(0, 0); }
	bool Enable(void) override { return true; }
	void Disable(void) override {}
	void Reset(void) override {}
	uint32_t EraseSize(void) const override { return SECTOR_SIZE; }
	uint32_t WriteGran(void) const override { return 4; }

	int Read(uint64_t Off, void *pBuf, uint32_t Len) override
	{
		if (s_Dead || Off + Len > s_RegionSize) return -EIO;
		memcpy(pBuf, &s_Flash[Off], Len);
		return (int)Len;
	}

	int Write(uint64_t Off, const void *pData, uint32_t Len) override
	{
		if (s_Dead || Off + Len > s_RegionSize) return -EIO;
		s_OpCount++;
		uint32_t n = Len;
		if (s_CutCountdown == 0)
		{
			n = s_CutKeep < Len ? s_CutKeep : Len;
			s_Dead = true;
		}
		else if (s_CutCountdown > 0)
		{
			s_CutCountdown--;
		}
		const uint8_t *p = (const uint8_t *)pData;
		for (uint32_t i = 0; i < n; i++)
		{
			s_Flash[Off + i] &= p[i];
		}
		return s_Dead ? -EIO : (int)Len;
	}

	int Erase(uint64_t Off, uint32_t Len) override
	{
		// bt_pds erases exactly one sector at a time.
		if (s_Dead || Off + Len > s_RegionSize) return -EIO;
		s_OpCount++;
		uint32_t n = Len;
		if (s_CutCountdown == 0)
		{
			n = s_CutKeep < Len ? s_CutKeep : Len;
			s_Dead = true;
		}
		else if (s_CutCountdown > 0)
		{
			s_CutCountdown--;
		}
		memset(&s_Flash[Off], 0xFF, n);
		return s_Dead ? -EIO : 0;
	}

	void SetRegion(uint32_t Size) { Region(0, Size); }
};

static MockNvm s_Nvm;

static void PowerOn(void)
{
	s_Dead = false;
	s_CutCountdown = -1;
}

static void FreshFlash(uint32_t Sectors)
{
	s_RegionSize = Sectors * SECTOR_SIZE;
	s_Nvm.SetRegion(s_RegionSize);
	memset(s_Flash, 0xFF, sizeof(s_Flash));
	PowerOn();
}

static int g_Fail;

#define CHECK(cond, ...) do { if (!(cond)) { g_Fail++; \
	printf("FAIL %s:%d: ", __func__, __LINE__); printf(__VA_ARGS__); \
	printf("\n"); } } while (0)

// Reference model of expected content.
typedef std::map<uint32_t, std::vector<uint8_t>> Model;

static void VerifyModel(const Model &m, const char *Tag)
{
	for (const auto &kv : m)
	{
		uint8_t buf[BT_PDS_RECORD_DATA_MAX];
		ssize_t r = BtPdsRead(kv.first, buf, sizeof(buf));
		CHECK(r == (ssize_t)kv.second.size(),
			  "%s id %u len %zd want %zu", Tag, kv.first, r, kv.second.size());
		if (r == (ssize_t)kv.second.size() && r > 0)
		{
			CHECK(memcmp(buf, kv.second.data(), r) == 0,
				  "%s id %u data mismatch", Tag, kv.first);
		}
	}
}

static std::vector<uint8_t> Val(uint32_t Id, uint32_t Rev, size_t Len)
{
	std::vector<uint8_t> v(Len);
	for (size_t i = 0; i < Len; i++)
	{
		v[i] = (uint8_t)(Id * 7U + Rev * 13U + i);
	}
	return v;
}

static void TestBasic(void)
{
	FreshFlash(2);
	CHECK(BtPdsInit(&s_Nvm) == 0, "init");
	uint8_t buf[BT_PDS_RECORD_DATA_MAX];

	CHECK(BtPdsRead(1, buf, sizeof(buf)) == -ENOENT, "absent");
	auto v = Val(1, 0, 20);
	CHECK(BtPdsWrite(1, v.data(), v.size()) == 20, "write");
	CHECK(BtPdsRead(1, buf, sizeof(buf)) == 20, "read len");
	CHECK(memcmp(buf, v.data(), 20) == 0, "read data");
	CHECK(BtPdsRead(1, buf, 4) == 20, "truncated read reports full len");

	auto v2 = Val(1, 1, 128);
	CHECK(BtPdsWrite(1, v2.data(), v2.size()) == 128, "update max len");
	CHECK(BtPdsRead(1, buf, sizeof(buf)) == 128, "read updated");
	CHECK(memcmp(buf, v2.data(), 128) == 0, "updated data");

	CHECK(BtPdsWrite(2, nullptr, 0) == 0, "zero length write");
	CHECK(BtPdsRead(2, buf, sizeof(buf)) == 0, "zero length read");

	CHECK(BtPdsDelete(1) == 0, "delete");
	CHECK(BtPdsRead(1, buf, sizeof(buf)) == -ENOENT, "deleted");
	CHECK(BtPdsDelete(1) == 0, "delete absent");

	CHECK(BtPdsWrite(3, buf, BT_PDS_RECORD_DATA_MAX + 1) == -EINVAL, "oversize");
	CHECK(BtPdsWrite(0xFFFFFFFEUL, buf, 4) == -EINVAL, "reserved id");

	// Remount keeps everything.
	CHECK(BtPdsInit(&s_Nvm) == 0, "remount");
	CHECK(BtPdsRead(1, buf, sizeof(buf)) == -ENOENT, "deleted after remount");
	CHECK(BtPdsRead(2, buf, sizeof(buf)) == 0, "zero len after remount");
}

static void TestChurn(uint32_t Sectors, int Writes)
{
	FreshFlash(Sectors);
	CHECK(BtPdsInit(&s_Nvm) == 0, "init");
	Model m;

	// One cold id that is written once and rarely touched.
	auto cold = Val(100, 0, 60);
	CHECK(BtPdsWrite(100, cold.data(), cold.size()) == 60, "cold write");
	m[100] = cold;

	for (int i = 0; i < Writes; i++)
	{
		uint32_t id = 1 + (i % 3);
		auto v = Val(id, i, 8 + (i % 5) * 24);
		ssize_t r = BtPdsWrite(id, v.data(), v.size());
		CHECK(r == (ssize_t)v.size(), "churn write %d ret %zd", i, r);
		m[id] = v;
		if (i % 977 == 0)
		{
			VerifyModel(m, "churn");
		}
	}
	VerifyModel(m, "churn end");
	CHECK(BtPdsInit(&s_Nvm) == 0, "remount");
	VerifyModel(m, "churn remount");
}

// Fill until the store refuses, confirm -ENOMEM behavior is stable.
static void TestFull(void)
{
	FreshFlash(2);
	CHECK(BtPdsInit(&s_Nvm) == 0, "init");
	Model m;
	uint32_t id = 1;
	for (;;)
	{
		auto v = Val(id, 0, 100);
		ssize_t r = BtPdsWrite(id, v.data(), v.size());
		if (r == -ENOMEM)
		{
			break;
		}
		CHECK(r == 100, "fill write ret %zd", r);
		m[id] = v;
		id++;
		CHECK(id < 100, "no full condition reached");
		if (id >= 100) return;
	}
	VerifyModel(m, "full");
	CHECK(BtPdsInit(&s_Nvm) == 0, "remount full store");
	VerifyModel(m, "full remount");
	// At hard capacity an update may not fit either; the store must keep
	// its data intact in both outcomes.
	auto v = Val(1, 9, 100);
	ssize_t ur = BtPdsWrite(1, v.data(), v.size());
	CHECK(ur == 100 || ur == -ENOMEM, "update at capacity ret %zd", ur);
	if (ur == 100)
	{
		m[1] = v;
	}
	VerifyModel(m, "full update");
}

// Defect 1: cut power at every programming op of a write whose EnsureSpace
// runs a real garbage collection (op burst >= 5; StartSector costs 2).
// After reboot, init must succeed and all committed values must read back.
static void TestGcCutSweep(uint32_t Sectors)
{
	FreshFlash(Sectors);
	CHECK(BtPdsInit(&s_Nvm) == 0, "init");
	Model m;
	std::vector<uint8_t> snap;
	Model snapModel;
	std::vector<uint8_t> trig;
	uint32_t trigId = 0;
	int totalOps = 0;

	// Live set: 3 hot ids + 1 cold id at 72 bytes each keeps the whole set
	// plus GC overhead inside one sector.
	uint32_t hotIds = 3;
	size_t recLen = 72;
	// A cold id keeps at least one live record in the oldest sector so a
	// garbage collection always copies something.
	auto coldV = Val(99, 0, recLen);
	CHECK(BtPdsWrite(99, coldV.data(), coldV.size()) == (ssize_t)recLen,
		  "cold write");
	m[99] = coldV;
	for (uint32_t i = 1; i <= 400; i++)
	{
		// A fresh never-updated id every 5th write leaves one live record
		// in every filled sector, so no victim allows a zero-copy
		// collection and the swept collection has a real copy phase.
		uint32_t id = (i % 5) == 0 ? 1000 + i : 1 + (i % hotIds);
		auto v = Val(id, i, recLen);
		snap.assign(s_Flash, s_Flash + s_RegionSize);
		snapModel = m;
		s_OpCount = 0;
		ssize_t r = BtPdsWrite(id, v.data(), v.size());
		CHECK(r == (ssize_t)recLen, "prefill ret %zd", r);
		m[id] = v;
		if (s_OpCount >= 5)
		{
			trig = v;
			trigId = id;
			totalOps = s_OpCount;
			break;
		}
	}
	CHECK(totalOps >= 5, "gc never triggered");
	if (totalOps < 5) return;

	int mountFail = 0;
	for (int cut = 0; cut < totalOps; cut++)
	{
		for (uint32_t keep : {0U, 6U, 12U})
		{
			memcpy(s_Flash, snap.data(), s_RegionSize);
			PowerOn();
			CHECK(BtPdsInit(&s_Nvm) == 0, "mount before cut run");
			s_CutCountdown = cut;
			s_CutKeep = keep;
			(void)BtPdsWrite(trigId, trig.data(), trig.size());

			PowerOn();	// reboot
			int r = BtPdsInit(&s_Nvm);
			if (r != 0)
			{
				mountFail++;
				printf("  cut %d keep %u: init %d\n", cut, keep, r);
				continue;
			}
			// Every value committed before the interrupted write must read
			// back, except the interrupted id, which may be old or new.
			for (const auto &kv : snapModel)
			{
				uint8_t buf[BT_PDS_RECORD_DATA_MAX];
				ssize_t rr = BtPdsRead(kv.first, buf, sizeof(buf));
				if (kv.first == trigId)
				{
					CHECK(rr == (ssize_t)kv.second.size() ||
						  rr == (ssize_t)recLen,
						  "torn id state %zd", rr);
					continue;
				}
				CHECK(rr == (ssize_t)kv.second.size(),
					  "gc cut id %u len %zd want %zu",
					  kv.first, rr, kv.second.size());
				if (rr == (ssize_t)kv.second.size() && rr > 0)
				{
					CHECK(memcmp(buf, kv.second.data(), rr) == 0,
						  "gc cut id %u data", kv.first);
				}
			}
			auto post = Val(777, 1, 40);
			ssize_t pr = BtPdsWrite(777, post.data(), post.size());
			CHECK(pr == 40, "post-recovery write cut %d keep %u ret %zd",
				  cut, keep, pr);
		}
	}
	CHECK(mountFail == 0, "%d cut points failed to mount", mountFail);
	printf("  gc cut sweep %u sectors: %d cut points x 3 tears\n",
		   Sectors, totalOps);
}

// Defect 3: cut power at every programming op of BtPdsClear.
// After reboot the store must hold all old records or none.
static void TestClearCutSweep(uint32_t Sectors)
{
	FreshFlash(Sectors);
	CHECK(BtPdsInit(&s_Nvm) == 0, "init");
	Model m;
	for (uint32_t i = 1; i <= 6; i++)
	{
		auto v = Val(i, 0, 40);
		CHECK(BtPdsWrite(i, v.data(), v.size()) == 40, "populate");
		m[i] = v;
	}
	// Force at least one GC so records spread across sectors.
	for (int i = 0; i < 30; i++)
	{
		auto v = Val(3, i + 1, 40);
		CHECK(BtPdsWrite(3, v.data(), v.size()) == 40, "spread");
		m[3] = v;
	}
	std::vector<uint8_t> snap(s_Flash, s_Flash + s_RegionSize);

	s_OpCount = 0;
	CHECK(BtPdsClear() == 0, "clear");
	int totalOps = s_OpCount;
	CHECK(totalOps >= 2, "clear ops %d", totalOps);

	int mixed = 0, mountFail = 0;
	for (int cut = 0; cut < totalOps; cut++)
	{
		for (uint32_t keep : {0U, 8U, 200U})
		{
			memcpy(s_Flash, snap.data(), s_RegionSize);
			PowerOn();
			CHECK(BtPdsInit(&s_Nvm) == 0, "mount before clear run");
			s_CutCountdown = cut;
			s_CutKeep = keep;
			(void)BtPdsClear();

			PowerOn();
			int r = BtPdsInit(&s_Nvm);
			if (r != 0)
			{
				mountFail++;
				printf("  clear cut %d keep %u: init %d\n", cut, keep, r);
				continue;
			}
			int present = 0;
			for (const auto &kv : m)
			{
				uint8_t buf[BT_PDS_RECORD_DATA_MAX];
				if (BtPdsRead(kv.first, buf, sizeof(buf)) >= 0)
				{
					present++;
				}
			}
			if (present != 0 && present != (int)m.size())
			{
				mixed++;
				printf("  clear cut %d keep %u: %d of %zu resurrected\n",
					   cut, keep, present, m.size());
			}
		}
	}
	CHECK(mountFail == 0, "%d clear cuts failed to mount", mountFail);
	CHECK(mixed == 0, "%d clear cuts resurrected a partial set", mixed);
	printf("  clear cut sweep %u sectors: %d cut points x 3 tears\n",
		   Sectors, totalOps);

	// A completed clear stays empty after remount.
	memcpy(s_Flash, snap.data(), s_RegionSize);
	PowerOn();
	CHECK(BtPdsInit(&s_Nvm) == 0, "mount");
	CHECK(BtPdsClear() == 0, "clean clear");
	CHECK(BtPdsInit(&s_Nvm) == 0, "remount");
	for (const auto &kv : m)
	{
		uint8_t buf[BT_PDS_RECORD_DATA_MAX];
		CHECK(BtPdsRead(kv.first, buf, sizeof(buf)) == -ENOENT,
			  "id %u after clean clear", kv.first);
	}
}

int main(void)
{
	TestBasic();
	TestChurn(2, 20000);
	TestChurn(4, 100000);
	TestFull();
	TestGcCutSweep(2);
	TestGcCutSweep(4);
	TestClearCutSweep(2);
	TestClearCutSweep(4);
	printf(g_Fail ? "RESULT: %d FAILURES\n" : "RESULT: ALL PASS\n", g_Fail);
	return g_Fail ? 1 : 0;
}
