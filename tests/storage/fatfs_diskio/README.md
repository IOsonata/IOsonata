# Generic FatFs host regression

Run from the repository root:

```sh
make -C tests/storage/fatfs_diskio test
```

These host tests exercise the production generic `FatFsMount()` and
`DiskIO` implementations using simulated 512-byte and 4096-byte
sector media. They check blank-media formatting, file I/O, remount
persistence, optional volume labels, sector geometry and I/O errors.
The 832 KiB simulation verifies that a nonblank disk is not
automatically formatted, then checks a newly initialized FAT
volume, its label, and single-sector writes after explicit test reset.

The nRF52840 `UsbMscFlashDisk` example is separate: it uses its
original small direct FAT12 initializer and 4096-byte USB sectors
for both 256 KiB and 832 KiB disks. It does not call `FatFsMount()`.
The generic FatFs implementation remains available for firmware
file access and has not been modified by this restoration.

The `exemples/storage/nvm_diskio_test.cpp` host test covers the
NVM block adapter with optional smaller logical sectors. Host
tests do not replace target compilation or physical USB and flash
validation.
