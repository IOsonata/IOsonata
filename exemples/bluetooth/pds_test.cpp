/**-------------------------------------------------------------------------
@example	pds_test.cpp

@brief	Host test for the bt_pds record store over a real Nvm.

		The store runs on the same Nvm driver a target uses, over a mock
		memory controller interface (DEVINTRF_TYPE_MEMCTRL) rather than a
		fabricated medium layer, so the driver's own chunking, addressing and
		erase dispatch are in the path.

		The mock enforces what a real controller enforces, so a store mistake
		shows up as a violation rather than as wrong data: a program that
		starts off a write unit boundary or is not a whole number of units is
		refused and counted, and programming can only clear bits.

		The whole suite runs twice, at a write unit of 4 and of 16. Four is
		what the nRF parts report and keeps the on medium layout that is
		already in the field. Sixteen is the STM32WBA quad word, where the
		24 byte sector header no longer fills a whole number of units and the
		first record therefore moves.

		Power loss is injected at the medium: the Nth programming operation
		lands only its first few bytes and every operation after it fails
		until the next mount. The sweeps walk that N across a whole run.

Build and run on the host:

  g++ -std=gnu++23 -O1 -I include -I include/storage -I Linux/include \
	  exemples/bluetooth/pds_test.cpp src/bluetooth/bt_pds.cpp \
	  src/storage/nvm.cpp src/device.cpp src/device_intrf.cpp -o pds_test
  ./pds_test

		Like nvm_test.cpp this needs a do nothing iopinctrl.h on the include
		path, because nvm.cpp includes that per target header for the write
		protect pin and no host one exists yet.

@author	Hoang Nguyen Hoan
@date	Jul 19, 2026

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
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cerrno>
#include <atomic>
#include <map>
#include <vector>

#include "device_intrf.h"
#include "coredev/spi.h"
#include "storage/nvm.h"
#include "bluetooth/bt_pds.h"

// ---------------------------------------------------------------------------
// Test bookkeeping
// ---------------------------------------------------------------------------
static int g_Fail = 0;
static int g_Checks = 0;

#define CHECK(cond, ...) do { \
	g_Checks++; \
	if (!(cond)) { \
		g_Fail++; \
		printf("FAIL %s:%d: ", __func__, __LINE__); \
		printf(__VA_ARGS__); \
		printf("\n"); \
	} \
} while (0)

// Where the run has got to. Printed as it goes and flushed, so a long suite
// and a stalled one do not look the same from outside.
static void Step(const char *pWhat, uint32_t Done, uint32_t Total)
{
	printf("\r    %-18s %6u / %-6u", pWhat, (unsigned)Done, (unsigned)Total);
	fflush(stdout);

	if (Done >= Total)
	{
		printf("\n");
	}
}

// The pin driver is per architecture; the driver only toggles a protect pin.
extern "C" {
void IOPinConfig(int, int, int, IOPINDIR, IOPINRES, IOPINTYPE) {}
void IOPinSet(int, int) {}
void IOPinClear(int, int) {}
}

// The two functions a target defines in its QSPI implementation. This memory
// is not on a bus at all, so neither is ever reached; they are here because
// the driver serves every transport from one object.
void QuadSPISetMemSize(SPIDev_t * const, uint32_t) {}

bool QuadSPISendCmd(SPIDev_t * const, uint8_t, uint32_t, uint8_t, uint32_t,
					uint8_t)
{
	return false;
}

// ---------------------------------------------------------------------------
// The medium. NOR semantics: an erase sets every bit, a program can only
// clear bits, and a unit is programmed whole or not at all in normal running.
// ---------------------------------------------------------------------------
#define PDS_SECTOR_SIZE		4096U
#define PDS_MAX_SECTORS		4U
#define PDS_MAX_REGION		(PDS_MAX_SECTORS * PDS_SECTOR_SIZE)
#define PDS_PAGE_SIZE		64U

static uint8_t s_Mem[PDS_MAX_REGION];
static uint32_t s_RegionSize;

// What the controller takes. Nothing derives this; it is the part talking.
static uint32_t s_WriteGran;

// Programming operations counted since the medium was wiped. A program of a
// page chunk is one, an erase of a unit is one. The address phase is not a
// programming operation and is not counted.
static uint32_t s_OpCount;

// Power loss. -1 disables it; otherwise the operation this many away lands
// only s_CutKeep bytes and everything after it fails until the next boot.
static int s_CutCountdown;
static uint32_t s_CutKeep;
static bool s_Dead;

// Programs the controller would have refused. Any of these is a store fault,
// not a driver one, because the driver passes the store's offsets through.
static uint32_t s_AlignFault;

static void MediumWipe(uint32_t RegionSize, uint32_t Gran)
{
	memset(s_Mem, 0xFF, sizeof(s_Mem));
	s_RegionSize = RegionSize;
	s_WriteGran = Gran;
	s_OpCount = 0;
	s_CutCountdown = -1;
	s_CutKeep = 0;
	s_Dead = false;
	s_AlignFault = 0;
}

static void MediumArmCut(uint32_t OpIndex, uint32_t Keep)
{
	s_CutCountdown = (int)OpIndex;
	s_CutKeep = Keep;
}

static void MediumPowerUp(void)
{
	s_CutCountdown = -1;
	s_Dead = false;
}

// How much of this operation lands. Everything, unless the cut is due.
static uint32_t MediumTake(uint32_t Len)
{
	s_OpCount++;

	if (s_CutCountdown == 0)
	{
		s_Dead = true;

		return s_CutKeep < Len ? s_CutKeep : Len;
	}
	if (s_CutCountdown > 0)
	{
		s_CutCountdown--;
	}

	return Len;
}

// The erase a target port supplies. Nvm calls this rather than framing a
// command, because a memory reached through a controller has no bus to put
// one on. Overrides the weak answer in nvm.cpp, the way a real port does.
extern "C" int NvmMcuErase(uintptr_t Addr)
{
	if (s_Dead)
	{
		return -EIO;
	}
	if ((Addr % PDS_SECTOR_SIZE) != 0U || Addr + PDS_SECTOR_SIZE > s_RegionSize)
	{
		s_AlignFault++;

		return -EINVAL;
	}

	memset(&s_Mem[Addr], 0xFF, MediumTake(PDS_SECTOR_SIZE));

	return s_Dead ? -EIO : 0;
}

// ---------------------------------------------------------------------------
// The controller as a device interface. The frame is the address and nothing
// in front of it, the way the memctrl case in nvm_test.cpp takes it.
// ---------------------------------------------------------------------------
static std::vector<uint8_t> s_Tx;
static uint32_t s_Addr;
static bool s_AddrSet;
static uint32_t s_CmdBytes;

static bool MemStartTx(DevIntrf_t *, uint32_t)
{
	s_Tx.clear();
	s_AddrSet = false;

	return true;
}

static void MemStopTx(DevIntrf_t *) {}

// The address phase already latched where to read, so starting the read half
// of the transaction must not throw it away.
static bool MemStartRx(DevIntrf_t *, uint32_t)
{
	return true;
}

static void MemStopRx(DevIntrf_t *) {}

static int MemTxData(DevIntrf_t *, const uint8_t *pData, int Len)
{
	for (int i = 0; i < Len; i++) { s_Tx.push_back(pData[i]); }

	if (s_AddrSet == false)
	{
		if (s_Tx.size() < 4)
		{
			return Len;
		}
		if (s_Tx.size() > 4)
		{
			s_CmdBytes += (uint32_t)s_Tx.size() - 4U;
		}
		s_Addr = ((uint32_t)s_Tx[0] << 24) | ((uint32_t)s_Tx[1] << 16) |
				 ((uint32_t)s_Tx[2] << 8) | s_Tx[3];
		s_AddrSet = true;
		s_Tx.clear();

		return Len;
	}

	s_Tx.clear();

	if (s_Dead)
	{
		return -1;
	}

	// A quad word controller programs whole units on unit boundaries. Anything
	// else is refused, which is how a store that pads its records wrongly is
	// caught here rather than by reading the data back.
	if ((s_Addr % s_WriteGran) != 0U || ((uint32_t)Len % s_WriteGran) != 0U)
	{
		s_AlignFault++;

		return -1;
	}

	uint32_t n = MediumTake((uint32_t)Len);

	for (uint32_t i = 0; i < n; i++)
	{
		if (s_Addr + i < s_RegionSize)
		{
			s_Mem[s_Addr + i] &= pData[i];
		}
	}
	s_Addr += (uint32_t)Len;

	return s_Dead ? -1 : Len;
}

static int MemRxData(DevIntrf_t *, uint8_t *pBuff, int Len)
{
	if (s_AddrSet == false)
	{
		// A status poll would land here. The driver must not make one.
		s_CmdBytes++;

		return 0;
	}
	if (s_Dead)
	{
		return -1;
	}

	for (int i = 0; i < Len; i++)
	{
		pBuff[i] = s_Addr < s_RegionSize ? s_Mem[s_Addr] : 0xFF;
		s_Addr++;
	}

	return Len;
}

static void MockEnable(DevIntrf_t *) {}
static void MockDisable(DevIntrf_t *) {}
static void MockPowerOff(DevIntrf_t *) {}
static uint32_t MockGetRate(DevIntrf_t *) { return 0; }
static uint32_t MockSetRate(DevIntrf_t *, uint32_t r) { return r; }
static void *MockGetHandle(DevIntrf_t *) { return nullptr; }

class MemCtrlIntrf : public DeviceIntrf {
public:
	DevIntrf_t vDev;

	MemCtrlIntrf()
	{
		memset(&vDev, 0, sizeof(vDev));
		vDev.pDevData = this;
		vDev.Type = DEVINTRF_TYPE_MEMCTRL;
		vDev.Disable = MockDisable;
		vDev.Enable = MockEnable;
		vDev.GetRate = MockGetRate;
		vDev.SetRate = MockSetRate;
		vDev.StartRx = MemStartRx;
		vDev.RxData = MemRxData;
		vDev.StopRx = MemStopRx;
		vDev.StartTx = MemStartTx;
		vDev.TxData = MemTxData;
		vDev.TxSrData = MemTxData;
		vDev.StopTx = MemStopTx;
		vDev.PowerOff = MockPowerOff;
		vDev.GetHandle = MockGetHandle;
		vDev.MaxRetry = 5;
		vDev.EnCnt = 1;
		atomic_flag_clear(&vDev.bBusy);
	}

	operator DevIntrf_t * const () override { return &vDev; }
	uint32_t Rate(uint32_t r) override { return r; }
	uint32_t Rate(void) override { return 1000000; }
};

static MemCtrlIntrf s_Bus;
static Nvm s_Nvm;

static NvmCfg_t MemCfg(uint32_t Gran)
{
	NvmCfg_t cfg;

	memset(&cfg, 0, sizeof(cfg));
	cfg.TotalSize = s_RegionSize;
	cfg.EraseSize = PDS_SECTOR_SIZE;
	cfg.PageSize = PDS_PAGE_SIZE;
	cfg.WriteGran = Gran;
	cfg.AddrSize = 4;
	cfg.WrProtPin = { -1, -1, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL };

	return cfg;
}

// True when the store mounted. A suite that cannot mount stops there: going
// on to drive an unmounted store hides the reason behind whatever it does
// next, and there is nothing left to test.
static bool Mounted(int Res, const char *pWhat)
{
	g_Checks++;

	if (Res != 0)
	{
		g_Fail++;
		printf("FAIL %s: mount returned %d\n", pWhat, Res);

		return false;
	}

	return true;
}

// Bring the memory up and mount the store on it, the way a reset does.
static int Boot(uint32_t Gran)
{
	MediumPowerUp();

	NvmCfg_t cfg = MemCfg(Gran);

	if (s_Nvm.Init(cfg, &s_Bus) == false)
	{
		return -EIO;
	}

	return BtPdsInit(&s_Nvm);
}

// ---------------------------------------------------------------------------
// The reference model
// ---------------------------------------------------------------------------
typedef std::map<uint32_t, std::vector<uint8_t> > Model_t;

static std::vector<uint8_t> MakeVal(uint32_t Seed, uint32_t Len)
{
	std::vector<uint8_t> v(Len);

	for (uint32_t i = 0; i < Len; i++)
	{
		// Never the record magic, so the layout scan below cannot match data.
		v[i] = (uint8_t)((Seed * 31U + i * 7U) & 0x7FU);
	}

	return v;
}

static bool ModelMatches(const Model_t &Model, const char *pWhat)
{
	uint8_t buf[BT_PDS_RECORD_DATA_MAX];
	bool ok = true;

	for (Model_t::const_iterator it = Model.begin(); it != Model.end(); ++it)
	{
		ssize_t n = BtPdsRead(it->first, buf, sizeof(buf));

		if (n != (ssize_t)it->second.size() ||
			memcmp(buf, it->second.data(), it->second.size()) != 0)
		{
			printf("FAIL %s: id %08X read %d want %u\n", pWhat, it->first,
				   (int)n, (unsigned)it->second.size());
			ok = false;
			break;
		}
	}

	return ok;
}

// ---------------------------------------------------------------------------
// What the medium should look like. The store keeps these to itself, so a
// layout test has to restate them; that is the point of the test.
// ---------------------------------------------------------------------------
#define PDS_REC_MAGIC		0x53445042UL	// "BPDS"
#define PDS_SECTOR_MAGIC	0x32534450UL	// "PDS2"
#define PDS_SECTOR_HDR		24U
#define PDS_REC_HDR			16U

static uint32_t MemWord(uint32_t Off)
{
	uint32_t w;

	memcpy(&w, &s_Mem[Off], sizeof(w));

	return w;
}

static uint32_t PadTo(uint32_t x, uint32_t u)
{
	return (x + u - 1U) & ~(u - 1U);
}

// Offset of the Nth record in sector 0, or 0 when there is no such record.
static uint32_t FindRecord(uint32_t Nth, uint32_t Gran)
{
	uint32_t off = PadTo(PDS_SECTOR_HDR, Gran);
	uint32_t seen = 0;

	while (off + PDS_REC_HDR <= PDS_SECTOR_SIZE)
	{
		if (MemWord(off) != PDS_REC_MAGIC)
		{
			return 0;
		}
		if (seen == Nth)
		{
			return off;
		}

		uint16_t len;

		memcpy(&len, &s_Mem[off + 12], sizeof(len));

		uint32_t dataLen = len == 0xFFFFU ? 0U : len;

		off += PadTo(PDS_REC_HDR + dataLen, Gran);
		seen++;
	}

	return 0;
}

// ---------------------------------------------------------------------------
// The layout the write unit produces. This is the whole of what changes
// between a 4 byte unit and a 16 byte one, so it is checked directly rather
// than only through data that happens to read back.
// ---------------------------------------------------------------------------
static void TestLayout(uint32_t Gran)
{
	printf("--- layout, unit %u\n", Gran);

	MediumWipe(2U * PDS_SECTOR_SIZE, Gran);
	if (Mounted(Boot(Gran), "layout") == false)
	{
		return;
	}

	CHECK(MemWord(0) == PDS_SECTOR_MAGIC, "layout: sector header at the base");

	std::vector<uint8_t> a = MakeVal(1, 4);
	std::vector<uint8_t> b = MakeVal(2, 4);

	CHECK(BtPdsWrite(0x11, a.data(), a.size()) == (ssize_t)a.size(),
		  "layout: first write");

	uint32_t first = FindRecord(0, Gran);

	CHECK(first == PadTo(PDS_SECTOR_HDR, Gran),
		  "layout: first record at %u, sector header padded to the unit",
		  (unsigned)PadTo(PDS_SECTOR_HDR, Gran));

	CHECK(BtPdsWrite(0x22, b.data(), b.size()) == (ssize_t)b.size(),
		  "layout: second write");

	uint32_t second = FindRecord(1, Gran);

	CHECK(second == first + PadTo(PDS_REC_HDR + 4U, Gran),
		  "layout: record stride is the header plus data padded once, got %u",
		  (unsigned)(second - first));

	// The bytes between the header and the first record are never programmed.
	for (uint32_t i = PDS_SECTOR_HDR; i < first; i++)
	{
		CHECK(s_Mem[i] == 0xFFU, "layout: padding byte %u left erased",
			  (unsigned)i);
		break;
	}

	CHECK(s_AlignFault == 0, "layout: %u programs the controller refused",
		  (unsigned)s_AlignFault);
	CHECK(s_CmdBytes == 0, "layout: no command traffic on a controller");
}

// ---------------------------------------------------------------------------
// Basic semantics
// ---------------------------------------------------------------------------
static void TestBasics(uint32_t Gran)
{
	printf("--- semantics, unit %u\n", Gran);

	MediumWipe(2U * PDS_SECTOR_SIZE, Gran);
	if (Mounted(Boot(Gran), "basics") == false)
	{
		return;
	}

	uint8_t buf[BT_PDS_RECORD_DATA_MAX];

	CHECK(BtPdsRead(0x10, buf, sizeof(buf)) == -ENOENT, "basics: absent id");

	std::vector<uint8_t> v1 = MakeVal(7, 40);

	CHECK(BtPdsWrite(0x10, v1.data(), v1.size()) == (ssize_t)v1.size(),
		  "basics: write");
	CHECK(BtPdsRead(0x10, buf, sizeof(buf)) == (ssize_t)v1.size() &&
		  memcmp(buf, v1.data(), v1.size()) == 0, "basics: read back");

	// A short destination still reports the stored length.
	CHECK(BtPdsRead(0x10, buf, 8) == (ssize_t)v1.size(),
		  "basics: truncated read reports the full length");

	std::vector<uint8_t> v2 = MakeVal(9, 100);

	CHECK(BtPdsWrite(0x10, v2.data(), v2.size()) == (ssize_t)v2.size(),
		  "basics: update");
	CHECK(BtPdsRead(0x10, buf, sizeof(buf)) == (ssize_t)v2.size() &&
		  memcmp(buf, v2.data(), v2.size()) == 0, "basics: update read back");

	CHECK(BtPdsDelete(0x10) == 0, "basics: delete");
	CHECK(BtPdsRead(0x10, buf, sizeof(buf)) == -ENOENT, "basics: deleted id");
	CHECK(BtPdsDelete(0x10) == 0, "basics: deleting an absent id succeeds");

	// Zero length is a value, not a delete.
	CHECK(BtPdsWrite(0x11, nullptr, 0) == 0, "basics: zero length write");
	CHECK(BtPdsRead(0x11, buf, sizeof(buf)) == 0,
		  "basics: zero length is present");

	std::vector<uint8_t> big = MakeVal(3, BT_PDS_RECORD_DATA_MAX + 1U);

	CHECK(BtPdsWrite(0x12, big.data(), big.size()) < 0,
		  "basics: oversize refused");
	CHECK(BtPdsWrite(0xFFFFFFFEUL, v1.data(), v1.size()) < 0,
		  "basics: reserved id refused");

	// Everything survives a mount.
	if (Mounted(Boot(Gran), "basics remount") == false)
	{
		return;
	}
	CHECK(BtPdsRead(0x10, buf, sizeof(buf)) == -ENOENT,
		  "basics: delete survived the remount");
	CHECK(BtPdsRead(0x11, buf, sizeof(buf)) == 0,
		  "basics: zero length survived the remount");

	CHECK(s_AlignFault == 0, "basics: %u programs the controller refused",
		  (unsigned)s_AlignFault);
}

// ---------------------------------------------------------------------------
// The mount refuses what it cannot lay out
// ---------------------------------------------------------------------------
static void TestMountRefuses(void)
{
	printf("--- mount checks\n");

	MediumWipe(2U * PDS_SECTOR_SIZE, 4U);

	NvmCfg_t cfg = MemCfg(32U);

	CHECK(s_Nvm.Init(cfg, &s_Bus), "refuse: memory init at unit 32");
	CHECK(BtPdsInit(&s_Nvm) == -EINVAL,
		  "refuse: a unit past BT_PDS_MAX_WRITE_GRAN");

	// One sector leaves nothing to collect into.
	MediumWipe(PDS_SECTOR_SIZE, 4U);
	cfg = MemCfg(4U);
	CHECK(s_Nvm.Init(cfg, &s_Bus), "refuse: memory init at one sector");
	CHECK(BtPdsInit(&s_Nvm) == -EINVAL, "refuse: a region of one sector");
}

// ---------------------------------------------------------------------------
// Churn and capacity
// ---------------------------------------------------------------------------
static void TestChurn(uint32_t Gran)
{
	printf("--- churn, unit %u\n", Gran);

	MediumWipe(PDS_MAX_REGION, Gran);
	if (Mounted(Boot(Gran), "churn") == false)
	{
		return;
	}

	Model_t model;
	bool ok = true;

	const uint32_t rounds = 2000U;

	for (uint32_t i = 0; i < rounds && ok; i++)
	{
		if ((i % 100U) == 0U)
		{
			Step("operations", i, rounds);
		}

		uint32_t id = 0x100U + (i % 9U);
		uint32_t len = 8U + (i % 11U) * 16U;

		if ((i % 17U) == 16U)
		{
			ok = BtPdsDelete(id) == 0;
			model.erase(id);
			continue;
		}

		std::vector<uint8_t> v = MakeVal(i, len);
		ssize_t w = BtPdsWrite(id, v.data(), v.size());

		if (w != (ssize_t)v.size())
		{
			printf("FAIL churn: write %u returned %d\n", (unsigned)i, (int)w);
			ok = false;
			break;
		}
		model[id] = v;

		if ((i % 250U) == 249U)
		{
			ok = Boot(Gran) == 0 && ModelMatches(model, "churn remount");
		}
	}

	Step("operations", rounds, rounds);

	CHECK(ok, "churn: %u operations over %u sectors", (unsigned)rounds,
		  PDS_MAX_SECTORS);
	if (Mounted(Boot(Gran), "churn final mount") == false)
	{
		return;
	}
	CHECK(ModelMatches(model, "churn final"), "churn: every value read back");
	CHECK(s_AlignFault == 0, "churn: %u programs the controller refused",
		  (unsigned)s_AlignFault);
}

static void TestFill(uint32_t Gran)
{
	printf("--- fill, unit %u\n", Gran);

	MediumWipe(2U * PDS_SECTOR_SIZE, Gran);
	if (Mounted(Boot(Gran), "fill") == false)
	{
		return;
	}

	Model_t model;
	std::vector<uint8_t> v = MakeVal(5, BT_PDS_RECORD_DATA_MAX);
	uint32_t id = 0x200U;
	ssize_t w = 0;

	while (id < 0x300U)
	{
		w = BtPdsWrite(id, v.data(), v.size());
		if (w < 0)
		{
			break;
		}
		model[id] = v;
		id++;
		Step("records", (uint32_t)model.size(), 0x100U);
	}
	Step("records", (uint32_t)model.size(), (uint32_t)model.size());

	CHECK(w == -ENOMEM, "fill: the store fills up and says so, got %d", (int)w);
	CHECK(model.size() > 4U, "fill: %u records fit", (unsigned)model.size());

	// Updating an existing id at capacity still works, because collecting the
	// victim sector releases the stale copy.
	Model_t::iterator it = model.begin();
	std::vector<uint8_t> u = MakeVal(6, 16);

	CHECK(BtPdsWrite(it->first, u.data(), u.size()) == (ssize_t)u.size(),
		  "fill: update at capacity");
	it->second = u;

	if (Mounted(Boot(Gran), "fill remount") == false)
	{
		return;
	}
	CHECK(ModelMatches(model, "fill"), "fill: every value read back");
	CHECK(s_AlignFault == 0, "fill: %u programs the controller refused",
		  (unsigned)s_AlignFault);
}

// How much of a torn programming operation is left behind. A programming
// operation here is a whole page chunk, so the interesting sizes run past one
// write unit: half a unit tears a unit in the middle, and a whole chunk is
// the operation that finished and lost power before its result was read.
#define BT_PDS_TEAR_POINTS	5U
#define BT_PDS_TEARS(g)		{ 0U, (g) / 2U, (g), (g) * 2U, PDS_PAGE_SIZE }

// ---------------------------------------------------------------------------
// Power loss sweeps
//
// The sequence is driven with the cut armed at one programming operation
// after another. A write that returns an error is not counted as committed,
// so what the store must still hold after the next mount is exactly what it
// told the caller it had taken.
// ---------------------------------------------------------------------------
// The write that was in flight when the power went. Its record may be whole
// on the medium or not, and the store never said which, so the id it names is
// allowed to read back as either value afterwards.
typedef struct __Pending {
	bool					Valid;
	uint32_t				Id;
	std::vector<uint8_t>	Val;
} Pending_t;

static uint32_t RunSequence(uint32_t Count, Model_t &Model, Pending_t &Pend)
{
	Pend.Valid = false;

	for (uint32_t i = 0; i < Count; i++)
	{
		uint32_t id = 0x300U + (i % 5U);
		uint32_t len = 16U + (i % 7U) * 24U;
		std::vector<uint8_t> v = MakeVal(i + 1U, len);

		if (BtPdsWrite(id, v.data(), v.size()) != (ssize_t)len)
		{
			// The medium can take the last unit of a record and lose power
			// before the result is read, so a refused write is not a write
			// that certainly did not happen.
			Pend.Valid = true;
			Pend.Id = id;
			Pend.Val = v;

			return i;
		}
		Model[id] = v;
	}

	return Count;
}

// Every acknowledged record reads back, except the one that was in flight,
// which may read back as either the acknowledged value or the new one.
static bool ModelMatchesPend(const Model_t &Model, const Pending_t &Pend)
{
	uint8_t buf[BT_PDS_RECORD_DATA_MAX];

	for (Model_t::const_iterator it = Model.begin(); it != Model.end(); ++it)
	{
		ssize_t n = BtPdsRead(it->first, buf, sizeof(buf));

		if (n == (ssize_t)it->second.size() &&
			memcmp(buf, it->second.data(), it->second.size()) == 0)
		{
			continue;
		}
		if (Pend.Valid && it->first == Pend.Id &&
			n == (ssize_t)Pend.Val.size() &&
			memcmp(buf, Pend.Val.data(), Pend.Val.size()) == 0)
		{
			continue;
		}

		printf("FAIL cut: id %08X read %d, wanted %u", it->first, (int)n,
			   (unsigned)it->second.size());
		if (Pend.Valid && it->first == Pend.Id)
		{
			printf(" or the %u in flight", (unsigned)Pend.Val.size());
		}
		printf("\n");

		return false;
	}

	return true;
}

static void TestCutSweep(uint32_t Gran)
{
	printf("--- power cut sweep, unit %u\n", Gran);

	const uint32_t seqLen = 40U;
	const uint32_t keeps[BT_PDS_TEAR_POINTS] = BT_PDS_TEARS(Gran);

	// One clean run says how many programming operations there are to walk.
	MediumWipe(PDS_MAX_REGION, Gran);
	if (Mounted(Boot(Gran), "cut reference") == false)
	{
		return;
	}

	Model_t ref;
	Pending_t pend;

	CHECK(RunSequence(seqLen, ref, pend) == seqLen,
		  "cut: the reference run finishes");

	uint32_t ops = s_OpCount;
	uint32_t cuts = 0;
	bool ok = true;

	for (uint32_t at = 0; at < ops && ok; at++)
	{
		Step("cut points", at, ops);

		for (uint32_t k = 0; k < BT_PDS_TEAR_POINTS && ok; k++)
		{
			MediumWipe(PDS_MAX_REGION, Gran);
			if (Boot(Gran) != 0)
			{
				printf("FAIL cut: mount before the cut at %u\n", (unsigned)at);
				ok = false;
				break;
			}

			Model_t model;
			Pending_t cutPend;

			MediumArmCut(at, keeps[k]);
			RunSequence(seqLen, model, cutPend);
			cuts++;

			// Reboot on to whatever reached the medium.
			if (Boot(Gran) != 0)
			{
				printf("FAIL cut: mount after the cut at %u keep %u\n",
					   (unsigned)at, (unsigned)keeps[k]);
				ok = false;
				break;
			}
			if (ModelMatchesPend(model, cutPend) == false)
			{
				printf("FAIL cut: at %u keep %u lost a committed record\n",
					   (unsigned)at, (unsigned)keeps[k]);
				ok = false;
				break;
			}

			// The store has to be usable again, not merely readable.
			std::vector<uint8_t> v = MakeVal(99, 32);

			if (BtPdsWrite(0x400U, v.data(), v.size()) != (ssize_t)v.size())
			{
				printf("FAIL cut: at %u keep %u left the store unwritable\n",
					   (unsigned)at, (unsigned)keeps[k]);
				ok = false;
			}
		}
	}

	Step("cut points", ops, ops);

	CHECK(ok, "cut: %u cut points over %u programming operations",
		  (unsigned)cuts, (unsigned)ops);
	CHECK(s_AlignFault == 0, "cut: %u programs the controller refused",
		  (unsigned)s_AlignFault);
}

static void TestClearCutSweep(uint32_t Gran)
{
	printf("--- clear cut sweep, unit %u\n", Gran);

	const uint32_t seqLen = 12U;
	const uint32_t keeps[BT_PDS_TEAR_POINTS] = BT_PDS_TEARS(Gran);

	MediumWipe(PDS_MAX_REGION, Gran);
	if (Mounted(Boot(Gran), "clear reference") == false)
	{
		return;
	}

	Model_t ref;
	Pending_t pend;

	CHECK(RunSequence(seqLen, ref, pend) == seqLen,
		  "clear: the setup run finishes");

	uint32_t before = s_OpCount;

	CHECK(BtPdsClear() == 0, "clear: a clean clear succeeds");

	uint32_t ops = s_OpCount - before;
	uint32_t cuts = 0;
	bool ok = true;

	for (uint32_t at = 0; at < ops && ok; at++)
	{
		Step("cut points", at, ops);

		for (uint32_t k = 0; k < BT_PDS_TEAR_POINTS && ok; k++)
		{
			MediumWipe(PDS_MAX_REGION, Gran);

			Model_t model;
			Pending_t setupPend;

			if (Boot(Gran) != 0 ||
				RunSequence(seqLen, model, setupPend) != seqLen)
			{
				printf("FAIL clear: setup before the cut at %u\n", (unsigned)at);
				ok = false;
				break;
			}

			MediumArmCut(at, keeps[k]);
			BtPdsClear();

			if (Boot(Gran) != 0)
			{
				printf("FAIL clear: mount after the cut at %u keep %u\n",
					   (unsigned)at, (unsigned)keeps[k]);
				ok = false;
				break;
			}
			cuts++;

			// All of them or none of them, never a part of the set.
			uint8_t buf[BT_PDS_RECORD_DATA_MAX];
			uint32_t found = 0;

			for (Model_t::iterator it = model.begin(); it != model.end(); ++it)
			{
				ssize_t n = BtPdsRead(it->first, buf, sizeof(buf));

				if (n == (ssize_t)it->second.size() &&
					memcmp(buf, it->second.data(), it->second.size()) == 0)
				{
					found++;
				}
				else if (n != -ENOENT)
				{
					printf("FAIL clear: at %u keep %u id %08X read %d\n",
						   (unsigned)at, (unsigned)keeps[k], it->first, (int)n);
					ok = false;
					break;
				}
			}

			if (ok && found != 0U && found != model.size())
			{
				printf("FAIL clear: at %u keep %u left %u of %u records\n",
					   (unsigned)at, (unsigned)keeps[k], (unsigned)found,
					   (unsigned)model.size());
				ok = false;
			}
		}
	}

	Step("cut points", ops, ops);

	CHECK(ok, "clear: %u cut points over %u programming operations",
		  (unsigned)cuts, (unsigned)ops);
	CHECK(s_AlignFault == 0, "clear: %u programs the controller refused",
		  (unsigned)s_AlignFault);
}

// ---------------------------------------------------------------------------
// The record sequence past 16 bits
//
// A sequence that wraps at 65535 makes an older record compare newer, so a
// stale value wins and the store hands back data the caller replaced long
// ago. The field is 32 bits wide; this is what says so. Nothing else in this
// file writes anywhere near far enough to reach the boundary.
//
// The width has nothing to do with the write unit, so this runs once.
// ---------------------------------------------------------------------------
static void TestSequenceWidth(void)
{
	printf("--- sequence width\n");

	const uint32_t gran = 4U;
	const uint32_t rounds = 70000U;
	const uint32_t ids[2] = { 0x500U, 0x501U };

	MediumWipe(PDS_MAX_REGION, gran);
	if (Mounted(Boot(gran), "sequence") == false)
	{
		return;
	}

	std::vector<uint8_t> last[2];
	bool ok = true;

	for (uint32_t i = 0; i < rounds && ok; i++)
	{
		if ((i % 5000U) == 0U)
		{
			Step("writes", i, rounds);
		}

		uint32_t k = i & 1U;
		std::vector<uint8_t> v = MakeVal(i + 1U, 32U);

		if (BtPdsWrite(ids[k], v.data(), v.size()) != (ssize_t)v.size())
		{
			printf("FAIL sequence: write %u refused\n", (unsigned)i);
			ok = false;
			break;
		}
		last[k] = v;

		// Straddle the boundary with a mount, because the scan is what has
		// to pick the newest record out of what is on the medium.
		if (i == 65540U)
		{
			ok = Boot(gran) == 0;
		}
	}
	Step("writes", rounds, rounds);

	CHECK(ok, "sequence: %u writes past the 16 bit boundary", (unsigned)rounds);

	Model_t model;

	model[ids[0]] = last[0];
	model[ids[1]] = last[1];

	CHECK(ModelMatches(model, "sequence"), "sequence: the newest record wins");

	if (Mounted(Boot(gran), "sequence remount") == false)
	{
		return;
	}

	CHECK(ModelMatches(model, "sequence remount"),
		  "sequence: the newest record still wins after a mount");
	CHECK(s_AlignFault == 0, "sequence: %u programs the controller refused",
		  (unsigned)s_AlignFault);
}

// ---------------------------------------------------------------------------

static void RunAll(uint32_t Gran)
{
	printf("\n=== write unit %u ===\n", Gran);

	TestLayout(Gran);
	TestBasics(Gran);
	TestChurn(Gran);
	TestFill(Gran);
	TestCutSweep(Gran);
	TestClearCutSweep(Gran);
}

int main(void)
{
	TestMountRefuses();

	RunAll(4U);
	RunAll(16U);
	TestSequenceWidth();

	printf("\nChecks run: %d\n", g_Checks);
	printf("RESULT: %s\n", g_Fail == 0 ? "ALL PASS" : "FAIL");

	return g_Fail == 0 ? 0 : 1;
}
