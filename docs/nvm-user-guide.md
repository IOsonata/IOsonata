# NVM User Guide

`Nvm` provides byte-addressed non-volatile storage through a supplied
`DeviceIntrf`. The same driver uses SPI/I2C memory or the MCU's internal
memory through `NvmIntrf`. Geometry and commands are configuration data.
See [NVM architecture](architecture/nvm.md) for implementation boundaries.

## Choose the interface and example

| Use | Starting point | What to configure |
|---|---|---|
| Internal memory | [nvm_demo.cpp](../exemples/storage/nvm_demo.cpp), medium 0 | MCU library, reserved region, controller interface |
| Serial NOR | Same demo, medium 1 | SPI pins, chip select, geometry and commands |
| I2C EEPROM | Same demo, medium 2 | I2C address, geometry and write delay |
| Block storage | [nvm_diskio_test.cpp](../exemples/storage/nvm_diskio_test.cpp) | NvmDiskIO sector geometry and cache |
| littlefs integration | [nvm_littlefs_test.cpp](../exemples/storage/nvm_littlefs_test.cpp) | External littlefs, block callbacks and buffers |
| Generic driver tests | [nvm_test.cpp](../exemples/storage/nvm_test.cpp) | Desktop compiler and supplied mock interfaces |

The shared demo selects its medium with `NVM_DEMO_MEDIUM`, operation mode
with `NVM_DEMO_ASYNC`, and Bluetooth activity with `NVM_DEMO_BLE`.
It is source to integrate into a matching target application; this repository
does not provide a dedicated NvmDemo `ioc/` project.

The hardware demo writes and erases scratch storage. Its current internal
setup chooses space below `NvmMcuCeiling()`, prints linker regions, but does
not use those regions to choose its scratch window. Verify the printed range
against the application map and other storage before running it. Product
storage should use an explicitly reserved region.

Build the MCU library and application with matching target/stack settings.
Use [Getting Started](getting-started.md) for the IOcomposer workflow.

## Geometry and addressing

Initialize the interface first, then call
`Nvm::Init(cfg, &interface, regionOffset, regionSize)`.
The interface, NVM object and any asynchronous buffers must outlive operations.

| Field | Meaning |
|---|---|
| `DevNo` | Interface selector: for example SPI chip-select index or I2C address |
| `BaseAddr` | Memory's mapped base; normally zero for a serial chip |
| `TotalSize` | Configured device capacity in bytes |
| `EraseSize` | Erase unit; zero for a medium that overwrites directly |
| `SectorSize` | Logical sector size, if supplied |
| `PageSize` | Programming boundary / maximum configured chunk size |
| `WriteGran` | Required write alignment and length granularity; zero defaults to one |
| `AddrSize` | Address bytes; current implementation supports one through four |
| `RdCmd`, `WrCmd` | Command and dummy-cycle settings |
| `WriteDelayUs` | Settling delay for a medium without a busy-status protocol |
| `bIntEn` | NVM operation mode, independent of the interface's transfer mode |

The addressed location is:

`BaseAddr + regionOffset + operationOffset`

`Read()`, `Write()` and `Erase()` offsets are relative to the configured
region. A zero region size means the remainder of the configured device,
not an empty region. Reject an absent linker region before initialization.

Use `NvmMcuCfg()` to fill internal-memory geometry; zero-initialize the
configuration first because this helper leaves non-geometry fields untouched.
Do not copy nRF52 erase geometry to nRF54 RRAM, which overwrites without an
erase step. STM32 internal flash has a nonzero mapped base.

For external memory, use the actual part's geometry and supported protocol.
The generic serial-NOR erase path currently selects 4 KiB, 32 KiB or 64 KiB
commands. A valid-looking geometry alone does not add another command set.
The address frame is currently 32 bits even though public offsets are 64 bits.

If no write-protect pin is used, set its port and pin to -1; an all-zero
configuration would identify GPIO 0 instead.

## Reserve internal storage with the linker

Carve storage out of the application's FLASH allocation, including any
bootloader, stack image, DFU and bond-store reservations. A `NOLOAD` section
alone does not shrink an overlapping FLASH region.

The project linker script supplies `__start_nvm0` and `__stop_nvm0`
(and optionally the corresponding `nvm1` symbols). The section pattern is:

```ld
nvm0 (NOLOAD) :
{
	__start_nvm0 = .;
	KEEP(*(nvm0))
	. = ORIGIN(NVM0) + LENGTH(NVM0);
	__stop_nvm0 = .;
} > NVM0
```

This belongs inside `SECTIONS`; define a non-overlapping `NVM0` entry in
`MEMORY` using the selected target's addresses. See
[nvm_region.h](../include/storage/nvm_region.h). Check the final map file.

`NvmRegionAddr(0)` and `NvmRegionSize(0)` read those symbols. Missing or
invalid bounds yield a zero size. `NvmMcuCeiling()` identifies the upper
application boundary; it does not prove that a proposed range is unoccupied.

The following polling setup uses a linker region as the entire NVM device
window. It avoids treating a mapped STM32 address as an offset a second time:

```cpp
#include "storage/nvm.h"
#include "storage/nvm_intrf.h"
#include "storage/nvm_region.h"

static NvmIntrf s_MemoryInterface;
static Nvm s_Memory;

static bool StorageInit()
{
	NvmCfg_t cfg = {};
	NvmMcuCfg(cfg);
	const uint64_t start = NvmRegionAddr(0);
	const uint64_t size = NvmRegionSize(0);
	const uint64_t ceiling = NvmMcuCeiling();

	if (size == 0 || start < cfg.BaseAddr ||
		start - cfg.BaseAddr > cfg.TotalSize ||
		size > cfg.TotalSize - (start - cfg.BaseAddr) ||
		start > ceiling || size > ceiling - start)
	{
		return false;
	}
	if (!s_MemoryInterface.Init())
	{
		return false;
	}
	return s_Memory.Init(cfg, &s_MemoryInterface,
		start - cfg.BaseAddr, size);
}
```

Align the reserved region to the target's erase and write requirements.
`NvmIntrf` wraps the port's single controller interface; declaring another
wrapper does not create an independent controller or callback registration.

## Read, write, erase and synchronization

| Operation | Successful return | Meaning |
|---|---|---|
| `Read(offset, buffer, length)` | Requested byte count | Data copied to the destination |
| `Write(offset, data, length)` | Requested byte count | Completed in polling mode; accepted in deferred mode |
| `Erase(offset, length)` | Zero | Completed in polling mode; accepted in deferred mode |
| `Sync()` | Zero | Pending operation drained, with no unreported error |
| `IsBusy()` | Boolean | Advances deferred work and reports whether work remains |

Failures use negative errno values. Check exact byte counts, not only whether
a result is positive. A write splits at configured page boundaries but does
not automatically erase raw NOR flash. Erase the intended units first where
the medium requires it.

Write offsets and lengths must meet `WriteGran()`. For erase-capable media,
erase offsets and nonzero lengths must be multiples of `EraseSize()`.
On a no-erase medium, an in-range `Erase()` is a no-op; it does not fill
storage with 0xFF.

`MassErase()` is a whole-device command. It is refused for a restricted
window and unsupported for internal/address-only or no-erase media.
`SetWriteProtect()` also applies to the whole device and rejects a window.
Serial NOR protection uses configured status bits; its WP pin must not be
assumed to protect array contents directly.

## Deferred operations and buffer lifetime

There are two independent settings:

1. The interface's interrupt/DMA mode determines transport completion.
2. `NvmCfg_t::bIntEn` determines whether the NVM operation is completed in
   the call or advanced later.

The application owns interface callback setup. For an event-driven interface,
forward its events to the appropriate `Nvm::IntrfEvent()`, as
`MemIntrfEvtCB` does in the demo. NVM initialization does not replace the
interface callback or change its transfer mode.

A transport completion only ends that transfer. It does not prove the memory
finished programming or erasing. `IntrfEvent()` recognizes completion and
timeout events; TX-ready or FIFO-empty events are not medium completion.

After an accepted deferred write, keep the complete source buffer valid and
unchanged until the matching completion event or `Sync()` returns.
Continue calling `IsBusy()` from foreground work, or call `Sync()` where
blocking is appropriate. The next operation also drains prior work and can
report its error.

`IsBusy() == false` does not establish success: a failure observed while
advancing work is retained for `Sync()` or the next operation. The event
handler reports `NVM_EVT_WRITE_DONE`, `NVM_EVT_ERASE_DONE` or
`NVM_EVT_ERROR`, with region-relative offset, length and result.

Deferred mode is not a zero-latency guarantee. A step can issue a transport
transaction, poll status or incur the configured settling delay.
`pWaitCB` can abort a polling wait; polling budgets are iteration counts,
not milliseconds. Keep controller/radio completion processing able to run
while waiting. Drain work before replacing buffers or shutting down storage.

## Internal-memory ports and radio coexistence

The current implementations are
[Nordic](../ARM/Nordic/src/nvm_nrfx.cpp) and
[STM32](../ARM/ST/src/nvm_stm32.cpp). They have different capabilities:

- Nordic selects internal-controller access, SoftDevice submission or an
  MPSL-timeslot route according to its build and active stack integration.
- The current STM32 source supports WBA/L4 selections. Its operations finish
  in the call; requesting interface interrupt mode does not make them
  asynchronously arbitrated.
- A declared arbiter hook does not prove every port uses it. Do not infer
  support for another MCU family from the generic header.

Keep the stack's arbiter installed while the radio owns scheduling. Do not
bypass it with raw flash-register writes. For Nordic testing,
`NvmIntrfGetStat()` exposes direct, SoftDevice and timeslot counts plus
completion statistics. Record the route actually used with radio activity.

## NvmDiskIO and filesystems

`NvmDiskIO` adapts initialized NVM to `DiskIO` sectors:

```cpp
#include "storage/diskio_nvm.h"

static NvmDiskIO s_Disk;

static bool DiskInit(Nvm &memory)
{
	return s_Disk.Init(memory); // Or supply an explicit valid sector size.
}
```

The selected sector must divide the region and, for erase-capable media, be
a whole number of erase units. It must fit the DiskIO 16-bit sector-size API.
A 512-byte sector over a 4 KiB erase unit is therefore not accepted by this
adapter. Supply an explicit size when the medium reports no logical sector.

`Nvm::Size()` is bytes; `DiskIO::GetSize()` is KiB.
`SectWrite()` erases when required, writes a complete sector and calls
`Sync()` before returning, so the caller can reuse its sector buffer.

Buffered DiskIO access has a separate cache. Supply static cache storage as
shown in the block test and flush it before expecting cached writes to reach
the medium. Filesystem sync/close must also drain its own buffers.
The void DiskIO erase methods cannot return an error to a filesystem; an
adapter that requires error propagation must account for that limitation.

littlefs and FatFs remain application dependencies. The littlefs example is
an integration test with simplified erase/sync callbacks, not a general
power-loss validation or production error-handling template.
`FlashDiskIO` remains available for existing applications; adding this
guide does not migrate them automatically to `NvmDiskIO`.

## Bond storage, DFU and shared consumers

Bluetooth's optional PDS bond backend and DFU storage can use NVM. Keep their
regions disjoint from application files and any USB MSC volume. Reserve space
in the linker/partition layout; do not let each consumer choose the same
apparently unused top-of-memory range independently.

Use the owning subsystem's persistence and completion API. A successful
pairing, queued write or filesystem cache update alone is not proof of
durability. See the [Bluetooth guide](bluetooth-user-guide.md) and
[DFU architecture](architecture/dfu.md).

## Host tests and hardware checks

From the repository root, this driver test uses mock memory interfaces and
the existing host GPIO shim:

```bash
g++ -std=gnu++23 -O1 -I include -I include/storage -I Linux/include \
  -I tests/dfu/hostport \
  exemples/storage/nvm_test.cpp src/storage/nvm.cpp \
  src/device.cpp src/device_intrf.cpp -o /tmp/iosonata_nvm_test
/tmp/iosonata_nvm_test
```

The block and littlefs test sources show their respective model/integration
coverage; they are separate from the generic driver test and require their
own dependencies. Host tests do not exercise physical endurance, radio
arbitration, MCU cache behavior or power loss.

For hardware validation, record the region, geometry, medium, firmware
revision, stack configuration and completion mode. Check page crossings,
alignment rejection, neighboring data, readback, reset persistence and
radio-active operation. Power cycling must include a completed write before
interpreting the persistence result.

| Symptom | Check |
|---|---|
| Init fails | Geometry, region alignment/range, ID, interface initialization |
| Region size is zero | Linker section and exact start/stop symbols |
| Write stalls | Event forwarding, foreground progress and stack arbitration |
| IsBusy clears but data is wrong | Sync/error result, erase requirement and source lifetime |
| Block adapter rejects geometry | Sector divisibility, erase unit and sector-size limit |
| Data vanishes after reset | Cache flush, operation completion and reserved storage layout |
