# Generic FatFs host regression

Run from the repository root:

```sh
make -C tests/storage/fatfs_diskio test
```

This builds the production FatFs and DiskIO code against simulated 512-byte
sector media (SD-style) and 4096-byte sector media (raw flash). It checks
blank formatting, file I/O, remount persistence, geometry, error propagation
and refusal to format partially used or unreadable storage.

Both the firmware and the USB MSC flash example use the generic FatFs
mount/format operation for initialized DiskIO media. The example unmounts
the volume before USB MSC takes ownership. The physical sector size
reported by DiskIO matches the sector size exposed through USB MSC.

This is a host simulation; actual MCU memory endurance, power loss and USB
ownership handoff require separate hardware validation.
