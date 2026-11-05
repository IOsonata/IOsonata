# FatFs over generic DiskIO - host regression

Run from the repository root:

```sh
make -C tests/storage/fatfs_diskio test
```

Uses the production `src/storage/ff_diskio.cpp`, the existing `DiskIO`
implementation and the FatFs sources. An in-memory `DiskIO` models a 512-byte
direct-write medium and a 4096-byte erase-before-write medium. Covers sector
translation, preservation of adjacent data, read/write failures, bounds,
geometry, safe mount and explicit blank-only formatting, file write/read,
remount persistence, and the legacy 512-byte attachment function.

This is a host software test. MCU builds, flash endurance and USB-host ownership
transfers require separate validation.
