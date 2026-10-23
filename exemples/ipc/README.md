# IpcIntrf test firmware

`ipc_intrf_test.cpp` checks IpcIntrf alone on the two cores of the nRF5340.
No Bluetooth, no controller. One source builds both images, board.h picks the
role.

| Core | Project | Role |
|---|---|---|
| Network | `ARM/Nordic/nRF53/nRF5340_Net/exemples/IpcIntrfTest` | owner: formats the block, echoes every message |
| Application | `ARM/Nordic/nRF53/nRF5340_App/exemples/IpcIntrfTest` | peer: starts the network core, sends a pseudo random stream, checks the echo |

## Run

1. Build the two projects (they use the IOsonata_nRF5340_App and
   IOsonata_nRF5340_Net lib projects).
2. Flash the network core image, then the application core image. The
   application image holds the network core in reset until the block is
   cleared and attached, then releases it.
3. Open both virtual COM ports at 1000000 8N1. On the nRF5340 Audio DK
   (the default board) the first port is the application core (P1.05 TXD,
   P1.04 RXD) and the second the network core (P1.09 TXD, P1.08 RXD); on
   the nRF5340 DK (build with NRF5340_DK_PCA10095) P0.20/P0.22 and
   P1.01/P1.00. Both cores trace through SysLog: the peer logs the block
   clearing, the pin hand-over, the release of the network core and the
   link; the owner logs its boot, the formatting of the block, the first
   message and every 20000 echoes. The application core hands the network
   core's UART pins over (GPIO MCUSEL) before releasing it, so the owner's
   trace appears once the peer has run that far.

```
IpcIntrf test, peer, packet mode, window 6, 200000 messages
link up
run   sent=20000 echo=20000 txb=... rxb=... badlen=0 baddata=0 full=0 stall=0 maxin=6
...
done  sent=200000 echo=200000 ...
RESULT PASS
```

| LED | Meaning |
|---|---|
| LED1 | link is up |
| LED2 | toggles at each report while messages come back |
| LED3 | an error was seen, stays on |
| LED4 | the run finished without error |

`RESULT FAIL` follows `done` when `badlen`, `baddata` or `stall` is not zero.
`STALL` is printed when nothing came back for 2 s while messages were
outstanding. The network core has no console, its counters are in
`g_IpcTestStat` for a debugger.

## Settings (board.h of each project, both cores the same)

| Define | Default | Meaning |
|---|---|---|
| `IPCTEST_PACKET` | 1 | 1 packet slots, 0 byte stream |
| `IPCTEST_WINDOW` | 6 | messages in flight, 1 is lock step. A value above `IPCTEST_NB_PKT` (8) fills the packet ring. In byte mode the window counts 64 byte units, so 6 fills the 256 byte ring |
| `IPCTEST_MEM_ADDR` | 0x20070000 | shared block, top 64 KB of the application RAM |
| `IPCTEST_MEM_SIZE` | 0x2000 | size of the block, checked against the ring geometry at build time |
| `IPCTEST_COUNT` | 200000 | messages of one run |

The application linker script `nrf5340_xxaa_application.ld` declares the
block as its `IPC0` region (0x20070000 to 0x20080000, exported as
`__start_ipc0` and `__stop_ipc0`), so the linker never places data on it.

## Host run of the same source

`make -C tests/ipc test-fw` builds the two roles of this file as two
translation units, shares the block through the synchronous IpcIntrf port and
lets the owner run during every delay of the peer. It covers packet and byte
mode with a window of 1, 6 and 12, under the address and undefined behavior
sanitizers. It checks the logic of the test, not the nRF5340 port.

## Not covered here

The nRF5340 port (`ARM/Nordic/nRF53/src/ipc_ctrlr_nrf53.cpp`): the SPU opening
of the RAM block, the IPC channels and the interrupt are only checked by
building and linking, not by running on a DK.
