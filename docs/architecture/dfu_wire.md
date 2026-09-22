# DFU wire protocol (stage 0 recovery)

Status: implemented, host tested; not yet on hardware. Sep. 21, 2026.

This replaces SMP/CBOR in the stage 0 boot. SMP stays in the application
for over the air updates (nRF Connect Device Manager, nRF52/53/54, STM32WBA).
The image format stays MCUboot (imgtool). Everything below is about the wire
path in stage 0: UART, or USB CDC ACM on parts with a USB device port.

## 1. Goals

1. Nothing is erased or written before a manifest signed by the boot key
   has come in. Before that, stage 0 only decodes fixed size frames. The
   payload is proven at FINISH, against the digest that was signed (see
   section 8 for what that leaves).
2. No general purpose parser in stage 0. No CBOR, no protobuf, no base64,
   no nesting. Fixed opcodes, fixed field layout, little endian.
3. Works without a driver install on macOS, Windows 10+ and Linux, from a
   Python script and from a web page.
4. Stage 0 fits 32 KB over UART and 40 KB over USB CDC, software crypto.
5. Same code on every IOsonata MCU port; only the byte transport differs.

Not goals: reading memory back, erasing arbitrary regions, jumping to an
address, running code from RAM. Stage 0 offers none of these.

## 2. Why these choices

- Every other vendor ROM or boot loader (NXP blhost, ST AN3155, Espressif
  esptool, Silicon Labs XMODEM, Infineon DFU) uses a fixed binary protocol
  in the boot. Only Zephyr MCUboot serial recovery parses CBOR there.
- The nRF5 SDK Secure DFU was small because its protocol was fixed opcodes
  and objects checked by CRC. Its weak spot was a protobuf init packet
  decoded before the signature check. This design keeps the first part and
  removes the second.
- MCUboot signs the SHA-256 digest of header + payload + protected TLVs.
  The signature can therefore be checked against the digest the image
  claims before any payload arrives; the payload is then hashed as it
  streams and must end at that digest.
- USB CDC ACM and USB serial bridges have class drivers built into macOS,
  Windows 10+ and Linux. USB DFU class needs WinUSB on Windows (Zadig) and
  libusb elsewhere, so it is not the default.

## 3. Transport

DfuWire reads and writes bytes through an IOsonata DeviceIntrf and knows
nothing else about the link. UART, UsbdCdc and BtIntrf are all DeviceIntrf,
so they are interchangeable and the DFU core does not change with the link.
The interface is set at Init and can be replaced at run time with SetIntrf
(state kept, so an upload started on one link can continue on another after
INFO).

The stage 0 boot links one interface, chosen by the MCU: UsbdCdc on a part
with USB and a complete IOsonata USB device port (nRF52840 today), UART
on every other part. A part with USB but no IOsonata USB driver yet uses
UART until the driver exists. BLE is not in stage 0 (stack size); the same
core can run in the application over BtIntrf.

## 4. Layers: transport, protocol, core

Three layers, each swappable without touching the others:

    protocol   DfuWire (this document)   DfuNrfDfu (nRF5 SDK DFU opcodes)
               DfuSmp (SMP)              ... any other
                 |  calls DfuMgr only, never the flash or the crypto
    core       DfuMgr: manifest check, image writer, hash, record, reset
                 |  DfuStore_t, DfuTgt*, HashEngine, SignEngine
    target     dfu_<mcu>, dfu_layout_*.ld

    transport  DeviceIntrf under each protocol: UART, UsbdCdc, BtIntrf

DfuMgr (include/dfu/dfu_mgr.h), over DfuWriter and the boot checks:

    Init(DfuMgrCfg_t)                store, direct or slot 1, keys, hash,
                                     bManifestOnly, bAllowDowngrade
    BeginManifest(p, len)            header + TLVs, signature checked first
    BeginImage(len)                  image in order, checked at Finish
    Write(off, p, len)               payload or image offset; a resend of
                                     bytes taken is skipped
    Offset(), Crc()                  where the upload is, CRC-32 so far
    Finish(&info)                    hash from memory, entry, signature
    Commit()                         record (direct) or pending mark
    Abort()
    Reset(bRecovery)                 once the protocol has sent its reply

Two entry modes because the protocols differ:

- Manifest first (DfuWire; nRF5 DFU with our signed init object): nothing
  is erased or written before the signature checks.
- Image in order (SMP): the header arrives with the first chunk and the
  TLVs last, so the signature can only be checked at Finish. Writes before
  that go to slot 1 (application) or to slot 0 with the record erased
  (stage 0), never startable until Finish.

bManifestOnly makes BeginImage refuse. Stage 0 with the wire protocol sets
it, and does not link DfuSmp at all, so nothing in it writes before the
signature check.

DfuMgr is the one DFU API common to every protocol, transport and MCU.
Applications and boots talk to DfuMgr and to a protocol object; nothing
above the target layer differs between MCUs.

A protocol is an object initialised with a DfuMgr and a DeviceIntrf, with
SetIntrf and Poll (DfuWire; DfuSmp goes through DfuSerial or BtDfuSmp). Each MCU target selects its protocol and transport in the board.h of
its DfuBoot (and OTA) project:

    DFU_PROTO       DFU_PROTO_WIRE, DFU_PROTO_SMP, DFU_PROTO_NRFDFU
    DFU_TRANSPORT   DFU_TRANSPORT_UART, DFU_TRANSPORT_CDC, DFU_TRANSPORT_BLE

Stage 0 defaults: DFU_PROTO_WIRE, with DFU_TRANSPORT_CDC on parts with an
IOsonata USB device driver and DFU_TRANSPORT_UART elsewhere. The boot
region must hold the build: 32 KB for UART, 40 KB for CDC (section 12).

Which ones a build has:

- Stage 0: one protocol, one transport, chosen at build time (size).
- Application: any set. More than one can share a transport: the first
  frame picks the handler (0xC0 SLIP start for DfuWire and nRF5 DFU over
  serial, SMP serial marker 0x06 0x09, GATT service for BLE).
- Swapping at run time: Poll of the active protocol stops, SetIntrf or a
  new protocol object takes over; DfuMgr state (offset, hash) is kept.

## 5. Framing

SLIP (RFC 1055) over the byte stream, through the library Slip interface
(include/slip_intrf.h) layered over the UART or USB CDC DeviceIntrf; the
frame CRC is crc32_ieee from crc.h:

    END      0xC0   frame delimiter, sent before and after every frame
    ESC      0xDB
    ESC_END  0xDB 0xDC  stands for 0xC0 in the frame
    ESC_ESC  0xDB 0xDD  stands for 0xDB in the frame

As in RFC 1055, the byte after ESC is taken as data whatever it is, END
included: a frame that follows line noise ending in ESC joins the noise and
is dropped (bad CRC), and the host's resend is served. The library Slip is
used unchanged.

Decoded frame:

    offset size field
    0      1    Op      request opcode, response = Op | 0x80
    1      1    Seq     copied from request to response
    2      n    Body    per opcode, n <= max(MaxBody + 4, ManifestMax)
    2+n    4    Crc     CRC-32 (IEEE 802.3, zlib crc32) over Op..Body, LE

Rules:

- A frame with a bad CRC, a bad escape, a length over the limit, or an
  unknown opcode is dropped with no response. Bytes outside frames (console
  text, line noise, a terminal left open) are dropped the same way.
- One request in flight. The host resends after its timeout with the same
  Seq; every request is idempotent (see WRITE), so a resend is safe.
- The device never depends on DTR, RTS or a break. Host tools still set DTR
  on open, because the IOsonata CDC class treats DTR as "port open".
- MaxBody is reported by INFO, 512 by default, a power of 2 and a multiple
  of the target write unit.

Frame CRC is only for the link. Image integrity comes from SHA-256 and the
signature.

## 6. Operations

All fields little endian. Every response body starts with a Status byte.

### 0x01 INFO

Request body: empty. Allowed in every state.

Response body:

    Status      u8    0
    ProtoVer    u8    1
    State       u8    0 idle, 1 receiving, 2 complete
    Flags       u8    bit 0 downgrade allowed
    MaxBody     u16
    WriteUnit   u16   DfuTgtWriteUnit
    EraseMaxMs  u16   worst case one erase unit takes, for host timeouts
    ManifestMax u16   largest BEGIN body, header + TLVs
    SlotSize    u32   largest payload + header + TLVs accepted
    Offset      u32   payload bytes taken so far (State 1)
    CurVer      8     version of the image in slot 0, zero when none
    DevId       8     unique device id, as in the USB serial number
    BootVer     4     stage 0 build version

### 0x02 BEGIN

Request body: Total u16, Off u16, then manifest bytes Off to Off + n. The
manifest is the whole image header (HdrSize bytes, padding included)
followed by both TLV areas, exactly as imgtool wrote them, at most
ManifestMax bytes. It goes in pieces of at most MaxBody bytes, in order,
Off 0 first; a piece at Off 0 starts a new upload and drops any other. The
device gathers the pieces in the record body buffer (no second buffer) and
checks the manifest once Total bytes are in. Each piece before the last
gets Status OK.

Device steps, in order, stopping on the first failure:

1. Header: magic, 32 <= HdrSize <= request length, ImgSize > 0, flags
   known, the whole image fits SlotSize.
2. TLV walk over the bytes received, bounded by the TLV info lengths and
   by the request length: exactly one SHA-256 (0x10), one key hash (0x01),
   one ECDSA P-256 signature (0x22). Other types skipped by length.
3. Key hash equals SHA-256 of the boot key.
4. ECDSA P-256 verify of the signature over the SHA-256 TLV value.
5. Version: header version >= CurVer unless downgrade is allowed.

Only after step 5: the header and TLVs stay in RAM (record body), the
record is erased so slot 0 is no longer startable, State becomes
receiving, Offset 0.

Response body: Status, then Offset (u32) and Crc (u32), as WRITE.

The signature check in software takes from tens of milliseconds (Cortex-M4F)
to about a second (Cortex-M0); hosts wait up to 5 s for the BEGIN response.

### 0x03 WRITE

Request body: Offset u32, then 1..MaxBody payload bytes. Only in receiving.

- Offset == device Offset: erase ahead as needed, program, hash, advance.
- Offset < device Offset: nothing written (a resend of data already taken).
- Offset > device Offset: error ERR_OFFSET.
- Offset + length > ImgSize: error ERR_SIZE.

Response body: Status, then device Offset (u32) and Crc (u32, CRC-32 of
payload bytes 0..Offset). The host resumes from Offset after comparing Crc
with its own.

### 0x04 FINISH

Request body: empty. Only in receiving with Offset == ImgSize.

The device hashes the image back from memory (header, payload, protected
TLVs), compares it with the SHA-256 value already signed, checks the entry
against slot 0 (DfuTgtEntryValid), then writes the record, magic unit last.
State becomes complete. A FINISH resent after a lost response gets OK
again.

Response body: Status.

### 0x05 RESET

Request body: Mode u8, 0 reset, 1 start the application if valid.
Response is sent first, then the device resets or starts the image after
the transport has drained.

### 0x06 ABORT

Back to idle. What was written stays, the record stays erased.

## 7. Status codes

    0   OK
    1   ERR_STATE      operation not valid in this state
    2   ERR_LEN        body length wrong for the operation
    3   ERR_HDR        header magic, sizes or flags
    4   ERR_SIZE       does not fit, or past ImgSize
    5   ERR_TLV        TLV area malformed or a TLV missing or repeated
    6   ERR_KEY        key hash not ours
    7   ERR_SIG        signature bad
    8   ERR_VERSION    older than slot 0, downgrade not allowed
    9   ERR_OFFSET     offset past what the device holds
    10  ERR_FLASH      erase or program failed
    11  ERR_HASH       payload hash differs from the signed digest
    12  ERR_ENTRY      entry not for slot 0

## 8. Session

    host                                device
    INFO            ------------------> state, MaxBody, CurVer
    BEGIN manifest  ------------------> checks signature, no flash touched
                    <------------------ OK, Offset 0
    WRITE 0, n      ------------------> erase ahead, program, hash
                    <------------------ OK, Offset n, Crc
    ...
    FINISH          ------------------> hash, entry, record written last
                    <------------------ OK
    RESET 1         ------------------> starts the new image

Resume: after a port loss (USB replug renames the port on macOS), the host
sends INFO; if State is receiving it sends WRITE at the device Offset once
the Crc matches, else it starts again with BEGIN. After a device reset
everything starts again from BEGIN; the image was never startable meanwhile.

Power loss at any point leaves either the old record intact (before BEGIN
finished) or no record (slot 0 not startable, stage 0 stays in recovery).

What the signature check at BEGIN proves, and what it does not. It proves
the manifest was made with the boot key: random bytes, a fuzzer, a wrong
product's image or a forged signature never erase or write anything (the
host tests count every memory operation). It cannot prove the payload
before the payload is there: a genuine manifest, freely available in any
signed release, followed by other bytes, erases the record and slot 0 and
is refused at FINISH, leaving no startable image until a good one is sent.
Whoever holds the wire can wipe the application, never replace it; this
is the case of every wired recovery. The same holds for a genuine manifest
with its header edited, the edit is found at FINISH; editing the version
field cannot make an older image install.

## 9. Entering recovery

As today: no valid image, the button (DFU_BUT_PORT/PIN), or the RAM flag
DFU_FLAG_RECOVERY set by the application before a reset. Optional: the
application answers INFO and a 0x07 ENTER request on its own CDC/UART port
by setting the flag and resetting, so the host tool needs no button.

## 10. USB identity

- CDC ACM, one interface pair, bulk 64 bytes full speed.
- iSerialNumber: meant to be the device id in hex, as INFO DevId. Not
  filled yet: stage 0 passes no serial string and DevId is zero until the
  target layer gives a device id.
- The host tool finds the device by VID, PID and serial number, never by
  port name, so a macOS rename (usbmodem1101 to usbmodem1102) or a Windows
  COM number change does not matter.
- Examples use the pid.codes test VID/PID; a product sets its own.

## 11. Host tools

- Python/dfu_wire.py: pyserial only. Reads a signed .bin or .hex, splits
  manifest and payload itself, no package file. Opens /dev/cu.* on macOS
  (not /dev/tty.*, which waits for carrier), sets DTR, raw mode.
  Timeout per WRITE = EraseMaxMs + transfer time + margin.
- docs/tools/dfu_web.html (not written yet): one static page, Web Serial
  API (Chrome, Edge on macOS, Windows, Linux). Same steps as the script.
  Safari and Firefox have no Web Serial; the script is the fallback there.

## 12. Size, measured, -Os

    nRF52840 stage 0, USB CDC     SMP 44.5 KB   wire 36.2 KB
    nRF52832 stage 0, UART        SMP 35.3 KB   wire 26.6 KB
    nRF54L15 stage 0, UART                      wire 26.9 KB

What went: dfu_smp, cbor, dfu_serial (5.5 KB); key generation, agreement,
signing and HMAC, which the full crypto engines pulled in through their
tables of virtual functions (CryptoUeccVerify and CryptoSoftSha256Hash
carry only the check); exit handlers and the heap. What came: DfuWire
1.4 KB, DfuMgr 1.3 KB.

## 13. Code

- include/dfu/dfu_mgr.h, src/dfu/dfu_mgr.cpp: the API every protocol uses.
- include/dfu/dfu_wire.h, src/dfu/dfu_wire.cpp: this protocol over any
  DeviceIntrf.
- DfuSmp (was DfuMgr) and DfuHttp run over DfuMgr, images in order.
- exemples/dfu/dfu_boot_main.cpp: DFU_PROTO and DFU_TRANSPORT from board.h.
- include/crypto: CryptoUeccVerify, CryptoSoftSha256Hash.
- Python/dfu_wire.py: the host tool.
- Target layers, layouts, slot 1 install, record format: unchanged.

## 14. Tests

- tests/dfu/dfu_mgr_test: every bad manifest touches no memory, downgrade,
  tampered payload, power lost at every memory operation, image in order,
  slot 1; every memory kind.
- tests/dfu/dfu_wire_test: every operation and error, damaged, oversized,
  escaped and byte by byte frames, manifest in pieces, resends, SetIntrf,
  200000 random frames with no memory operation, header edits refused.
  No libFuzzer runtime in this environment, so the fuzzing is a seeded
  random loop under ASan and UBSan.
- make wire: dfu_wire.py against dfu_wire_host on a pty, every memory kind,
  a link lost halfway and taken up again.
- Still to do: hardware, nRF52840 USB CDC and UART on macOS, Windows 11,
  Linux; one STM32 and one LPC over a USB serial bridge.

## 15. Open decisions

1. Downgrade: refuse by default, allow by build option?
2. Application side ENTER handler: include in the examples?
3. USB DFU class as a later option for dfu-util users, or never?
4. MaxBody default 512, or 1024 on USB parts?
