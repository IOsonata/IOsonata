# USB Mass Storage device class

`UsbdMsc` implements one USB Mass Storage Bulk-Only Transport interface with a
SCSI transparent command set and one LUN. The class uses static storage only.

The application supplies a statically owned `DiskIO` object and a sector
buffer. The buffer must be at least `DiskIO::GetSectSize()` bytes. The class
does not own the medium.

```cpp
static MyDiskIO s_Disk;
alignas(4) static uint8_t s_Sector[512];
static UsbdMsc s_Msc;

static const UsbdMscCfg_t s_MscCfg = {
	.DevNo = 0,
	.pDisk = &s_Disk,
	.pSectorBuffer = s_Sector,
	.SectorBufferSize = sizeof(s_Sector),
	.bReadOnly = false,
	.bRemovable = true,
	.InterfaceString = 4,
	.FsMps = 0,
	.HsMps = 0,
	.pVendor = "I-SYST",
	.pProduct = "IOsonata Disk",
	.pRevision = "1.00",
};
```

Initialize the USB core before the class, then connect the controller:

```cpp
UsbInit(&usbCfg);
s_Msc.Init(s_MscCfg);
UsbEnable(0);

while (1)
{
	UsbProcess(0);
}
```

`UsbProcess()` dispatches `UsbdMsc::Process()`. Sector reads, sector writes and
SCSI command processing therefore stay outside the USB interrupt.

The initial command set is:

- `INQUIRY`
- `TEST UNIT READY`
- `REQUEST SENSE`
- `READ CAPACITY (10)`
- `MODE SENSE (6)`
- `START STOP UNIT`
- `READ (10)`
- `WRITE (10)`
- `PREVENT/ALLOW MEDIUM REMOVAL`
- `VERIFY (10)` without compare data
- `SYNCHRONIZE CACHE`

The nRF52840 managed project is
`ARM/Nordic/nRF52/nRF52840/exemples/UsbMscRamDisk/ioc`. It exposes a dedicated
64 KiB FAT12 RAM disk. No firmware or persistent storage region is exported.

After flashing, unmount the volume before running the raw BOT write test:

```bash
python3 Python/usb_msc_test.py --write-test
```

The write test saves, overwrites, hash-verifies and restores a 16-sector range
ending at the final RAM-disk sector. The runner also verifies prevented removal,
logical eject and reload, performs a BOT reset, and sends 50 repeated
`TEST UNIT READY` commands. Do not use `--write-test` with firmware that maps
MSC to persistent or shared storage.

Logical eject remains effective across BOT and USB bus reset. Removing USB bus
power reloads the statically configured medium so it is available after the
cable is reconnected.
