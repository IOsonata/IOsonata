# Internal data-memory port tests

From the repository root:

```sh
python3 -m pip install unicorn pyelftools
ARM_PREFIX=arm-none-eabi- python3 tests/storage/data_nvm/run.py
```

`ARM_PREFIX` can include an absolute toolchain path. The script compiles the
production L152/RA4M1 controller port, generic Nvm and DeviceIntrf code against
the vendor CMSIS headers at `-O0` and `-Os`, then runs the ARM code in Unicorn.
It does not replace controller accesses with test-only driver hooks.

Checks cover byte writes, EEPROM overwrite with zeros/ones, crossing transfer
and erase boundaries, RA4M1 blank checking and explicit erase, invalid ranges,
misaligned erase, the interface busy latch, unsupported interrupt mode and
registered-arbiter refusal. The model checks EEPROM unlock/lock and FACI
protected mode writes, P/E addresses and command completion. Injected program
errors and stuck-busy conditions must fail; a subsequent operation must work
after recovery. GPIO and the flash-clock query are stubbed because these tests
exercise memory access, not pin routing or system-clock selection.

The model is not silicon validation and does not establish analog flash timing,
endurance, ECC behavior or power-loss recovery. Run EepromDemo on NUCLEO-L152RE
and NvmDemo on RA4M1 for write/readback and power-cycle persistence checks.
Both use `exemples/storage/data_nvm_demo.cpp` and UART at 1000000 baud.
Their destructive ranges are documented in the corresponding MCU README.
