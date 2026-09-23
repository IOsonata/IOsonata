// Host support for the DFU tests: simulated internal memory, the stage 0
// target port, an Nvm over the simulated memory, and image files.
//
// The memory is mapped at a fixed low address so the 32 bit vector table
// words of a test payload can point into slot 0, the way they do on target.
// The layout symbols come from the link line (--defsym), the way a target
// link gets them from dfu_layout_*.ld.
//
// Memory kinds, as on target:
//	NOR		nRF52, 4 byte program unit, a write only clears bits, an erase sets
//			a unit to 0xFF
//	RRAM	nRF54L, 4 byte unit, a write overwrites, there is no erase
//	ECC16	STM32L4/WBA like: NOR with a 16 byte program unit that takes one
//			program between erases
//	BIG256	LPC like: NOR with a 256 byte program unit, also once per erase
//	SECT	STM32F4 like: NOR, 4 byte unit, slot 0 in sectors of different
//			sizes: 16 KB, 16 KB, then 32 KB
// On NOR every write over bits that are not erased is counted, and on ECC16
// and BIG256 every second program of a unit or unaligned write, so a missing
// erase or a state word sharing a unit shows as a count, not as a hash
// failure somewhere later.

#ifndef __DFU_HOST_H__
#define __DFU_HOST_H__

#include <stdint.h>
#include <stddef.h>
#include <setjmp.h>
#include <vector>
#include <string>

#include "storage/nvm.h"
#include "dfu/dfu_boot.h"

#define HOST_FLASH_BASE			0x10000000UL
#define HOST_FLASH_SIZE			0x00040000UL
#define HOST_UNIT				0x1000U

// Must match the --defsym values in the Makefile.
#define HOST_SLOT0				(HOST_FLASH_BASE + 0x00000)
#define HOST_SLOT0_SIZE			0x10000U
#define HOST_REC				(HOST_FLASH_BASE + 0x10000)
#define HOST_REC_SIZE			0x1000U
#define HOST_SLOT1				(HOST_FLASH_BASE + 0x11000)
#define HOST_SLOT1_SIZE			0x11000U
#define HOST_FLAG				(HOST_FLASH_BASE + 0x3FFF0)

enum HostMem { HOST_MEM_NOR, HOST_MEM_RRAM, HOST_MEM_ECC16, HOST_MEM_BIG256,
			   HOST_MEM_SECT };

// Name of a kind, for test output.
const char *HostMemName(HostMem Kind);

// Program unit of the current kind.
uint32_t HostUnit(void);

// Map the memory, all erased. Call once before anything else.
void HostFlashInit(HostMem Kind);
void HostFlashErase(void);
uint8_t *HostFlash(uintptr_t Addr);
HostMem HostFlashKind(void);

// Writes over bits that were not erased, NOR kinds only, and second programs
// of a unit or unaligned writes on ECC16 and BIG256.
extern int g_HostNorViolations;

// Power loss: after this many more DfuTgtWrite or DfuTgtErase calls, the
// next one writes half and longjmps to g_HostPowerJmp. Negative disables.
extern int g_HostFailAfter;

// Count of DfuTgtWrite and DfuTgtErase calls, every one.
extern int g_HostMemOps;
extern jmp_buf g_HostPowerJmp;

// DfuTgtStart longjmps here with 1 and leaves the address in g_HostStartAddr.
extern jmp_buf g_HostStartJmp;
extern uintptr_t g_HostStartAddr;

// DfuTgtReset longjmps here with 1.
extern jmp_buf g_HostResetJmp;

// The layout the link line declares.
DfuLayout_t HostLayout(void);

// Slot 1 trailer words.
uint32_t HostPending(void);
uint32_t HostDone(void);

std::vector<uint8_t> HostReadFile(const std::string &Path);

// Put an image file into slot 1 as an upload would, trailer cleared, then
// optionally mark it pending.
void HostPutSlot1(const std::vector<uint8_t> &Img, bool bPending);

// A byte stream DeviceIntrf for tests, with its C interface filled in, the
// way a UART or USB CDC driver fills it: layered interfaces (Slip) call that
// one, not the C++ methods. A subclass gives RxBytes and TxBytes.
class HostByteIntrf : public DeviceIntrf {
public:
	HostByteIntrf();
	operator DevIntrf_t * () override { return &vDevIntrf; }
	uint32_t Rate(uint32_t) override { return 0; }
	uint32_t Rate(void) override { return 0; }

	virtual int RxBytes(uint8_t *pBuf, int Len) = 0;
	virtual int TxBytes(const uint8_t *pData, int Len) = 0;

private:
	DevIntrf_t vDevIntrf;
};

// Nvm over a region of the simulated memory, with the geometry of the kind.
class HostNvm : public Nvm {
public:
	HostNvm(uintptr_t Addr, uint32_t Size);

	uint64_t Size(void) const override { return vSize; }
	uint32_t EraseSize(void) const override;
	uint32_t WriteGran(void) const override { return HostUnit(); }
	int Read(uint64_t Off, void *pBuf, uint32_t Len) override;
	int Write(uint64_t Off, const void *pData, uint32_t Len) override;
	int Erase(uint64_t Off, uint32_t Len) override;

	int vWrites;
	int vErases;
	int vBusyLeft;		// Answer -EBUSY this many times first

private:
	uintptr_t vAddr;
	uint32_t vSize;
};

#endif
