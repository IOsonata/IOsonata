# Generic FatFs host regression

Run from the repository root:

```sh
make -C tests/storage/fatfs_diskio test
```

This builds the production FatFs and DiskIO code against simulated 512-byte
sector media (SD-style) and 4096-byte sector media (raw flash). It checks
blank formatting, file I/O, remount persistence, geometry, error propagation
and refusal to format partially used or unreadable storage.

The USB MSC flash example has a separate, pre-existing FAT12 formatter.
The FatFs disk adapter uses the same physical sector sizes as MSC rather
than translating the disk to a second incompatible 512-byte layout.

This is a host simulation; actual MCU memory endurance, power loss and USB
ownership handoff require separate hardware validation.
