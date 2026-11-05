# NVM User Guide

`Nvm` provides byte-addressed non-volatile storage through a supplied
`DeviceIntrf`. The same driver uses SPI/I2C memory or the MCU's internal
memory through `NvmMcuIntrf`. Geometry and commands are configuration data.
See [NVM architecture](architecture/nvm.md) for implementation boundaries.

**0.13 naming change:** The MCU internal-memory interface is now
`NvmMcuIntrf` (header `storage/nvm_mcu_intrf.h`). Replace the former
`NvmIntrf` class and header references in applications. MCU-memory
arbitration functions and related data types use the `NvmMcuIntrf` prefix.
The generic `Nvm`, `DeviceIntrf`, and `NvmDiskIO` APIs are unchanged.

## Choose the interface and example

| Use | Starting point | What to configure |
|---|---|---|
| Internal memory | [nvm_demo.cpp](../exemples/storage/nvm_demo.cpp), medium 0 | MCU library, reserved region, controller interface |
| Serial NOR | Same demo, medium 1 | SPI pins, chip select, geometry and commands |
| I2C EEPROM | Same demo, medium 2 | I2C address, geometry and write delay |
| Block storage | [nvm_diskio_test.cpp](../exemples/storage/nvm_diskio_test.cpp) | NvmDiskIO sector geometry and cache |
| USB disk on internal memory | [usb_msc_flashdisk.cpp](../exemples/usb/usb_msc_flashdisk.cpp) | Linker region, NvmDiskIO and UsbdMsc |
| littlefs integration | [nvm_littlefs_test.cpp](../exemples/storage/nvm_littlefs_test.cpp) | External littlefs, block callbacks and buffers |
| Generic driver tests | [nvm_test.cpp](../exemples/storage/nvm_test.cpp) | Desktop compiler and supplied mock interfaces |

The shared demo selects its medium with `NVM_DEMO_MEDIUM`, operation mode
with `NVM_DEMO_ASYNC`, and Bluetooth activity with `NVM_DEMO_BLE`.
Supplied `NvmDemo/ioc` projects include STM32L476, LPC54628, nRF9160,
nRF91x1 and RA4M1. Inspect each project's linked source and medium selection:
the RA4M1 project uses [data_nvm_demo.cpp](../exemples/storage/data_nvm_demo.cpp)
for data flash, while other projects may use the shared demo or target-specific
setup. A matching project name does not imply the same physical memory.

The hardware demo writes and erases scratch storage. Its internal-memory
mode uses the linker-reserved NVM1 test region and fails if it is absent.
Never run the destructive checks against NVM0 bond storage or program flash.

Build the MCU library and application with matching target/stack settings.
Use [Getting Started](getting-started.md) for the IOcomposer workflow.

## Geometry and addressing

For MCU memory, initialize the reserved region with `NvmMcuCfgInit(cfg, RegionNo)`,
then initialize the controller interface and call `Nvm::Init(cfg, &interface)`.
The interface, NVM object and any asynchronous buffers must outlive operations.
External memory may also use `Nvm::Init(cfg, &interface, regionOffset, regionSize)`
to select a smaller area of an otherwise dedicated memory chip.

| Field | Meaning |
|---|---|
| `DevNo` | Interface selector: for example SPI chip-select index or I2C address |
| `BaseAddr` | Reserved MCU region's mapped address; normally zero for a serial chip |
| `TotalSize` | Capacity allocated to this Nvm device, not the MCU's program flash |
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

Use `NvmMcuCfgInit(cfg, RegionNo)` to fill the complete configuration for a
linker-reserved MCU NVM region. It returns `false` if that region is absent,
outside the MCU memory controller's address range, or incorrectly aligned.
For a separately reserved allocation, such as a DFU upload slot,
`NvmMcuCfgInit(cfg, BaseAddr, Size)` provides the same geometry and range
checks without misidentifying the allocation as NVM0/NVM1.
The function clears the config before filling it. Do not copy nRF52 erase
geometry to nRF54 RRAM, which overwrites without an erase step.

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

`NvmMcuCfgInit(cfg, 0)` reads those linker symbols and checks the reserved
region against the MCU's physical geometry. Missing or invalid bounds cause
initialization to fail. Only the reserved address and capacity are placed in
`NvmCfg_t`; no other program flash is exposed to the Nvm device.

The following polling setup uses a linker region as the entire NVM device
window. It avoids treating a mapped STM32 address as an offset a second time:

```cpp
#include "storage/nvm.h"
#include "storage/nvm_mcu_intrf.h"
#include "storage/nvm_region.h"

static NvmMcuIntrf s_MemoryInterface;
static Nvm s_Memory;

static bool StorageInit()
{
	NvmCfg_t cfg = {};

	if (NvmMcuCfgInit(cfg, 0) == false)
	{
		return false;
	}
	if (s_MemoryInterface.Init() == false)
	{
		return false;
	}
	return s_Memory.Init(cfg, &s_MemoryInterface);
}
```

Align the reserved region to the target's erase and write requirements.
`NvmMcuIntrf` wraps the port's single controller interface; declaring another
wrapper does not create an independent controller or callback registration.

## DiskIO initialization

An MCU internal-memory disk uses `Nvm::Init()` and `NvmDiskIO::Init()`.
`NvmMcuCfgInit()` fills the geometry, address and size from the
linker-reserved NVM region:

```cpp
#include "storage/diskio_nvm.h"
#include "storage/nvm_mcu_intrf.h"
#include "storage/nvm_region.h"

static NvmMcuIntrf s_NvmMcuIntrf;
static Nvm s_Nvm;
static NvmDiskIO s_Disk;

// At function entry:
NvmCfg_t cfg = {};

if (!NvmMcuCfgInit(cfg, 1) || !s_NvmMcuIntrf.Init() ||
	!s_Nvm.Init(cfg, &s_NvmMcuIntrf) || !s_Disk.Init(s_Nvm))
{
	return false;
}
```

The reserved region is presented to `Nvm` as configured memory.
The linker must reserve a disjoint area with correct erase and write
alignment. External memory uses the same `Nvm` and `NvmDiskIO`
with an injected SPI, I2C, QSPI or OSPI interface. Mounting and formatting
are described in [Firmware FatFs volumes](#firmware-fatfs-volumes).
USB MSC uses the initialized `DiskIO` directly.

The disk's sector size is a whole number of physical erase units.
For a 4 KiB raw-flash medium, FatFs and USB MSC both use 4 KiB sectors.
Writing a raw-flash sector erases and programs that erase unit, with
no automatic wear levelling.

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

Internal-memory implementations differ by MCU and memory type:

| Port | Memory |
|---|---|
| [Nordic](../ARM/Nordic/src/nvm_nrfx.cpp) | Target flash or RRAM, with stack-specific arbitration |
| [STM32 WBA/L4/H753](../ARM/ST/src/nvm_stm32.cpp) | Internal flash |
| [STM32L152](../ARM/ST/STM32L1xx/src/nvm_stm32l1xx.cpp) | True data EEPROM; byte overwrite, no erase step |
| [RA4M1](../ARM/Renesas/RA4/RA4M1/src/nvm_ra4m1.cpp) | Data flash; byte programming, explicit 1 KiB erase |
| [LPC546xx](../ARM/NXP/LPC546xx/src/nvm_lpc546xx.cpp) | Internal flash through IAP |

Use the geometry from the selected port; internal data EEPROM and data flash
are separate from application code flash. The code-flash linker example above
must not be copied unchanged to those media.

- Nordic selects internal-controller access, SoftDevice submission or an
  MPSL-timeslot route according to its build and active stack integration.
- The shared STM32 flash source supports WBA, L4 and H753. Its operations finish
  in the call; requesting interface interrupt mode does not make them
  asynchronously arbitrated.
- A declared arbiter hook does not prove every port uses it. Do not infer
  support for another MCU family from the generic header.

Keep the stack's arbiter installed while the radio owns scheduling. Do not
bypass it with raw flash-register writes. For Nordic testing,
`NvmMcuIntrfGetStat()` exposes direct, SoftDevice and timeslot counts plus
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

Bluetooth's optional PDS bond storage and DFU storage can use NVM. Keep their
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

## Firmware FatFs volumes

FatFs mounts the existing `DiskIO` without USB MSC. Use `NvmDiskIO` with
`Nvm` for internal program/data flash, serial NOR flash, EEPROM, FRAM or
other addressed memory. `SDCard` and other `DiskIO` devices work through
the same `FatFsMount` interface declared in `ff_diskio.h`.

The FatFs configuration supports 512, 1024, 2048 and 4096-byte sectors
(`FF_MAX_SS=4096`). FatFs reads and writes whole sectors reported by
`DiskIO::GetSectSize()`, matching the physical sector size USB MSC exposes
for the same disk. In particular, a 4 KiB `NvmDiskIO` volume uses 4 KiB FAT
sectors, not a separate emulated 512-byte layout.

```cpp
#include "ff.h"
#include "ff_diskio.h"
#include "storage/diskio_nvm.h"

static NvmDiskIO s_Disk;
static FATFS s_Fs;
alignas(4) static uint8_t s_FormatWork[4096];

// Initialize the Nvm transport, Nvm and NvmDiskIO first:
// s_Disk.Init(s_Nvm);
FatFsMountCfg_t mount = {};
mount.pDisk = &s_Disk;
mount.pFatFs = &s_Fs;
mount.pPath = "0:";
mount.bFormatIfBlank = true;
mount.pWork = s_FormatWork;
mount.WorkSize = sizeof(s_FormatWork);
FRESULT res = FatFsMount(mount);
```

Format work memory must be at least one physical sector; a disk that is
already formatted can be mounted with no work buffer. The volume is only
formatted if `bFormatIfBlank` is explicitly set, `f_mount` reports
`FR_NO_FILESYSTEM` and a complete read of the disk confirms all bytes are
`0xFF`. Neither disk errors nor unrecognized nonblank contents trigger
formatting. `FatFsUnmount()` releases FatFs before the application transfers
a writable volume to a USB host. Firmware and the USB host must not mount
the same writable filesystem concurrently.

`DiskIO::Flush()` is void, so `CTRL_SYNC` cannot report a failed cache
flush. Avoid unverified write-back caches for critical file data. A raw NOR
FAT volume also needs wear management and power-failure testing for
production logging. Increasing `FF_MAX_SS` to 4096 increases the space
used by FatFs' sector buffers; budget static RAM accordingly.
