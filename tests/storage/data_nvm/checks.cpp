// Execute the production Nvm and target port under the register model.
#include <string.h>
#include <stddef.h>
#include "storage/nvm_intrf.h"
#ifdef RA4M1
#include "ra4m1xxx.h"
extern "C" uint32_t SystemFlashClockGet(void) { return 24000000; }
extern "C" { uint32_t SystemMicroSecLoopCnt = 3; }
#define REG(member) (uintptr_t)&R_FACI_LP->member
extern "C" const uintptr_t g_Regs[] = {
	REG(FENTRYR), REG(FPMCR), REG(FPR), REG(FISR), REG(FCR),
	REG(FSTATR1), REG(FSTATR2), REG(FSTATR00), REG(FRESETR),
	REG(FSARH), REG(FSARL), REG(FEARH), REG(FEARL), REG(FWBL0), REG(DFLCTL)
};
#else
#include "stm32l1xx.h"
extern "C" const uintptr_t g_Regs[] = {
	(uintptr_t)&FLASH->PECR, (uintptr_t)&FLASH->PEKEYR, (uintptr_t)&FLASH->SR
};
#endif

static int Arbiter(NvmIntrfOp_t *) { return -EBUSY; }
#define CHECK(x) do { if (!(x)) return __LINE__; } while (0)
extern "C" int RunTest(int Test)
{
	NvmIntrf intrf;
	Nvm mem;
	NvmCfg_t cfg = {};
	NvmMcuCfg(cfg);
	CHECK(!intrf.Init(nullptr, true));
	CHECK(intrf.Init());
	CHECK(cfg.WriteGran == 1 && cfg.AddrSize == 4);
	CHECK(mem.Init(cfg, &intrf));
	uint8_t data[131], back[131];
	for (unsigned i = 0; i < sizeof(data); i++) data[i] = i * 37;
	if (Test == 1) // injected program error/timeout; do not report success
	{
		CHECK(mem.Write(31, data, sizeof(data)) < 0);
		return 0;
	}
	if (Test == 2) // subsequent operation after recovery
	{
		if (cfg.EraseSize) CHECK(mem.Erase(0, cfg.EraseSize) == 0);
		CHECK(mem.Write(31, data, sizeof(data)) == sizeof(data));
		CHECK(mem.Read(31, back, sizeof(back)) == sizeof(back));
		CHECK(memcmp(data, back, sizeof(data)) == 0);
		return 0;
	}
	if (cfg.EraseSize) CHECK(mem.Erase(0, 2 * cfg.EraseSize) == 0);
	uint32_t off = cfg.EraseSize ? cfg.EraseSize - 63 : 63;
	CHECK(mem.Write(off, data, sizeof(data)) == sizeof(data));
	CHECK(mem.Read(off, back, sizeof(back)) == sizeof(back));
	CHECK(memcmp(data, back, sizeof(data)) == 0);
	memset(data, 0xFF, sizeof(data));
	if (cfg.EraseSize)
	{
		CHECK(mem.Write(off, data, sizeof(data)) < 0);
		CHECK(mem.Erase(1, cfg.EraseSize) == -EINVAL);
		CHECK(mem.Erase(0, 2 * cfg.EraseSize) == 0);
	}
	CHECK(mem.Write(off, data, sizeof(data)) == sizeof(data));
	CHECK(mem.Read(off, back, sizeof(back)) == sizeof(back));
	CHECK(memcmp(data, back, sizeof(data)) == 0);
	CHECK(mem.Read(cfg.TotalSize, back, 1) == -EINVAL);
	CHECK(mem.Write(cfg.TotalSize, data, 1) == -EINVAL);
	// Even a raw DeviceIntrf caller cannot leave the data-memory range.
	uint8_t frame[] = { 3, 0, 0, 0, 0 };
	CHECK(intrf.Read(0, frame, 5, back, 1) == 0);
	frame[0] = 2;
	CHECK(intrf.Write(0, frame, 5, data, 1) == 0);
	CHECK(intrf.StartTx(0));
	CHECK(!intrf.Init()); // don't reset an active transfer's busy latch
	intrf.StopTx();
	NvmIntrfSetArbiter(Arbiter);
	CHECK(mem.Write(0, data, 1) < 0);
	NvmIntrfSetArbiter(nullptr);
	if (!cfg.EraseSize)
	{
		memset(data, 0, sizeof(data));
		CHECK(mem.Write(off, data, sizeof(data)) == sizeof(data));
		CHECK(mem.Read(off, back, sizeof(back)) == sizeof(back));
		CHECK(memcmp(data, back, sizeof(data)) == 0);
	}
	return 0;
}
