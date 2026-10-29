# SD Card User Guide

`SDCard` (`include/storage/sdcard.h`, `src/storage/sdcard_impl.cpp`) is the SD
memory card driver: a `DiskIO` of 512 byte sectors on the interface given to
`Init`, the same object for both buses of the card.

| Interface | Type | Bus | Selector |
| --- | --- | --- | --- |
| `SPI` | `DEVINTRF_TYPE_SPI` | SPI mode | chip select index |
| SD/MMC host (`Sdmmc`, `include/coredev/sdmmc.h`) | `DEVINTRF_TYPE_SDMMC` | SD bus, 1 or 4 data lines | slot of the host |

Cards: SDSC version 1 and 2, SDHC and SDXC, MMC in SPI mode only.

## Use

```cpp
#include "coredev/sdmmc.h"
#include "storage/sdcard.h"

static Sdmmc s_SdHost;
static SDCard s_Card;
alignas(4) static uint8_t s_Data[8 * SDCARD_SECT_SIZE];

// Call after s_SdHost.Init(boardHostCfg) succeeds.
static bool ReadCard()
{
	if (!s_Card.Init(&s_SdHost))
	{
		return false;
	}
	return s_Card.ReadSect(100, s_Data, 8) == 8;
}
```

`DeviceAddress()` sets the chip select or slot before `Init`, 0 by default.
`Init` identifies the card at 400 kHz, then runs the transfers at the rate the
interface had before `Init`, or at the card maximum (TRAN_SPEED) when lower:
set the rate on the interface first. A cache of sectors for the `DiskIO`
byte access (`Read`, `Write` by offset) is given to `Init` as memory or as
`DiskIOCache_t` blocks, 2 sectors at most.

`ReadSect` and `WriteSect` return the number of sectors moved, 0 on failure.
`Status()` reads the card status (CMD13). `Type()`, `GetNbSect()`,
`GetSize()` (KB), `Rate()`, `Cid()` and `Csd()` give the card found.

For a write, `WriteSect(sector, data, count)` must return the requested sector
count. It replaces existing contents. Use a disposable card for the supplied
write test; it attempts restoration but cannot guarantee it after a card error,
power loss or reset. Keep the host interface, card and any cache storage alive
for all operations.

## Build and run

1. Build `ARM/NXP/LPC546xx/LPC54628/lib/ioc` in Debug or Release.
2. Import `ARM/NXP/LPC546xx/LPC54628/exemples/SdCardTest/ioc`.
3. Check `src/board.h` for the card wiring and selected bus.
4. Build the matching configuration and insert a disposable card.
5. Run Debug with semihosting enabled, or use the Release UART console at
   1000000 baud, 8N1. Inspect every check and the final `ALL PASS` result.

For another target, select a supported SPI or SD/MMC interface and supply its
board pin map. SD/MMC host mode currently has an LPC546xx implementation.
The existence of `SDCard` alone does not add an SD/MMC host to another MCU.

## SPI mode

The SPI configuration sends 0xFF while it reads (`DummyByte`), keeps the chip
select low from `StartTx` to the stop (`SPICSEL_AUTO` or `SPICSEL_MAN`), and
its transfers end in the call. A command and its data are one transaction:
the chip select stays low from the command token to 8 clocks after the last
byte. The data blocks are checked with their CRC16. A multiple block read
ends with CMD12, a multiple block write with the stop token. An out of range
error in the response of CMD12 after the last sector of the card is not an
error (SD physical layer specification).

## SD bus: the SD/MMC host interface

The host is a `DeviceIntrf` of type `DEVINTRF_TYPE_SDMMC`: C data
`SdmmcDev_t`, C init `SdmmcInit()` in the MCU port, C++ class `Sdmmc`. A
transfer runs one card command. The command frame (`SDMMC_FRAME_SIZE`, 7
bytes) is sent by `TxData` after `StartTx`: the 6 byte command token of the
bus (the same as in SPI mode), then a byte of `SDMMC_FRAME_` flags:

| Flags | Meaning |
| --- | --- |
| `SDMMC_FRAME_RESP_NONE`, `_R1`, `_R1B`, `_R2`, `_R3` | response of the command; R1b waits the card busy |
| `SDMMC_FRAME_DATA_READ`, `_WRITE` | data blocks after the command |
| `SDMMC_FRAME_INIT` | 74 or more initialization clocks first, the bus back to 1 data line |
| `SDMMC_FRAME_STOP` | stop command (CMD12) at the end of the data |
| `SDMMC_FRAME_BUS_WIDTH` | the command sets the data bus width (ACMD6): argument 2 for 4 lines |

- No data: `Tx(Slot, Frame, 7)` when the response is not needed,
  `Read(Slot, Frame, 7, pResp, Len)` to get it: 4 bytes (the 32 bits of a 48
  bit response), 16 for R2 (the CID or CSD register), most significant byte
  first.
- Data read: `Read(Slot, Frame, 7, pData, Len)`.
- Data write: `StartTx`, `TxData(Frame)`, `TxData(pData, Len)`, `StopTx`,
  `bNoStop` set between them. The transfer returns once the card released
  its busy signal.

The block size is `Len` below 512 bytes, else 512 and `Len` a multiple of 512.
A command that fails moves nothing: `RxData` and the `TxData` of the data
return 0. A data command also fails when its R1, or the response of the stop
at its end, has an error bit (`SDMMC_R1_ERRORS`); the host ends the data with
a stop command. `Rate` sets the card clock, the highest the host makes at or
below the rate asked.

`SDCard` sends CMD0 with the initialization clocks, CMD8, ACMD41 until the
card is ready, CMD2, CMD3, CMD9, CMD7, ACMD6 when the host has 4 data lines
(`SdmmcCfg_t::BusWidth`), and CMD16 for the byte addressed cards. A write is
followed by CMD13 for the programming errors.

| MCU port | Host | Notes |
| --- | --- | --- |
| LPC546xx | SDIF (`ARM/NXP/LPC546xx/src/sdmmc_lpc546xx.cpp`) | slot 0, polling through the FIFO, auto stop, card detect and card supply switch |

## Example

`exemples/storage/sdcard_test.cpp` builds the interface and the card from
`board.h`: `SDMMC_PINS` (with `SDMMC_BUS_WIDTH`, `SDMMC_RATE`,
`SDMMC_CARD_DETECT`, `SDMMC_POWER_CTRL`) for an SD/MMC host, else
`SDCARD_SPI_PINS` (with `SDCARD_SPI_DEVNO`, `SDCARD_SPI_RATE`) for SPI mode.
It saves 16 sectors, tests single and multiple sector writes and reads, then
puts them back. See the LPCXpresso54628
[SdCardTest](../ARM/NXP/LPC546xx/LPC54628/exemples/SdCardTest/README.md).

## Validation

`tests/storage/sdcard/run.py` runs `SDCard` at -O0 and -Os with the address
and undefined behavior sanitizers on two models: an SPI interface with a card
in SPI mode answering byte by byte, and an SD/MMC host taking the frames
above with a card on the SD bus. The models check the clocks before CMD0, the
CRC7 of every command and the CRC16 of the data written, the chip select held
for a whole command and 8 clocks after it, the identification at 400 kHz,
the transfers at the card maximum, the response and data direction asked for
each command, the bus width, byte addresses on the standard capacity cards,
and the card faults (no card, no response, CMD8 pattern, a card never ready,
CSD fields, R1 errors, data CRC errors, missing start token, data refused,
programming errors, a card that stays busy). 135 of 152 mutants of the
driver are killed, the others are equivalent.

`tests/lpc/lpc546xx/sdif_test.cpp` runs the LPC546xx host with `SDCard` on a
register model of the SDIF and a card model.

No hardware result yet.
