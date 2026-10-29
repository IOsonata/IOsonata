# DFU wire protocol (stage 0 recovery)

Status: implemented with host tests. Hardware validation is not established
by this document. Source references describe the 0.13 implementation.

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
   Python script. A Web Serial client is not supplied.
4. Stage 0 fits 32 KB over UART and 40 KB over USB CDC, software crypto.
5. Same code on every IOsonata MCU port; only the byte transport differs.

Not goals: reading memory back, erasing arbitrary regions, jumping to an
address, running code from RAM. Stage 0 offers none of these.

## 2. Why these choices

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

The stage 0 boot links one interface selected by `DFU_TRANSPORT` in
`board.h`. The supplied nRF52840 DfuBoot selects CDC; the other supplied
DfuBoot configurations select UART, including LM20. USB driver availability
does not automatically change this selection. BLE is not accepted by the
stage 0 example; the same protocol can use BtIntrf in an application.

## 4. Layers: transport, protocol, core

Three layers, each swappable without touching the others:

    protocol   DfuWire (this document), DfuSmp (SMP)
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

- Manifest first (DfuWire): nothing
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

    DFU_PROTO       DFU_PROTO_WIRE, DFU_PROTO_SMP
    DFU_TRANSPORT   DFU_TRANSPORT_UART, DFU_TRANSPORT_CDC, DFU_TRANSPORT_BLE

`DFU_PROTO_NRFDFU` is reserved in the header and has no implementation.
The stage 0 example rejects it. Stage 0 defaults to DFU_PROTO_WIRE and UART
unless board configuration selects CDC. The boot
region must hold the build: 32 KB for UART, 40 KB for CDC (section 12).

Which ones a build has:

- Stage 0: one protocol, one transport, chosen at build time (size).
- Application: initializes the chosen protocol objects and routes their
  input. There is no automatic multi-protocol stream dispatcher.
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

The existing Slip decoder handles escaping; DfuWire then checks frame size,
opcode and CRC. There is no separate strict escape-validation stage in
DfuWire. Do not rely on malformed escape bytes alone causing rejection.

Decoded frame:

    offset size field
    0      1    Op      request opcode, response = Op | 0x80
    1      1    Seq     copied from request to response
    2      n    Body    per opcode, within the configured receive buffer
    2+n    4    Crc     CRC-32 (IEEE 802.3, zlib crc32) over Op..Body, LE

Rules:

- A frame with a bad CRC, a length over the receive-buffer limit, or an
  unknown opcode is dropped with no response. Bytes outside frames (console
  text, line noise, a terminal left open) are dropped the same way.
- One request in flight. The host retries with the same Seq. Seq is echoed,
  not used as a duplicate cache. WRITE skips accepted bytes and FINISH
  remembers success. BEGIN at offset zero restarts manifest collection;
  retrying BEGIN after final acceptance is not universally idempotent.
- The device never depends on DTR, RTS or a break. Host tools still set DTR
  on open, because the IOsonata CDC class treats DTR as "port open".
- MaxBody is reported by INFO. The stage 0 default is 512 bytes, overridden
  by `DFU_BOOT_BUFSIZE`. DfuWire requires a power of two of at least 16;
  the writer handles target program-unit staging.

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
- Offset < device Offset: skip the accepted prefix; write any remaining
  suffix. A fully accepted resend writes nothing.
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

Request body: Mode u8, 0 reset into recovery, 1 reset for normal boot.
The response is handed to the transport before the reset callback. The
stage 0 callback waits `DFU_BOOT_RESET_DELAY_MS` (20 ms by default), then
resets. This is a fixed delay, not a check that the transport has drained.

### 0x06 ABORT

Back to idle. What was written stays, the record stays erased.

### 0x07 ENTER

Request body: empty. Send OK, then request a reset into recovery through the
same callback as RESET. An application must initialize and poll DfuWire to
serve this operation; declaring the opcode alone does not add an application
recovery service. The host script exposes it with `--enter`.

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
sends INFO. If State is receiving, it probes with an already accepted WRITE
to obtain Offset and Crc; INFO itself contains no Crc field. It resumes at
Offset only when that checksum matches its payload, otherwise it starts
again with BEGIN. After a device reset
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
A peer with access to recovery and a valid manifest can invalidate the
application. Starting replacement code still requires successful image
verification. The same holds for a genuine manifest
with its header edited, the edit is found at FINISH; editing the version
field cannot make an older image install.

## 9. Entering recovery

As today: no valid image, the button (DFU_BUT_PORT/PIN), or the RAM flag
DFU_FLAG_RECOVERY set by the application before a reset. An application may initialize DfuWire on its CDC/UART port to answer INFO
and the implemented 0x07 ENTER request, allowing recovery without a button.

## 10. USB identity

- CDC ACM, one interface pair, bulk 64 bytes full speed.
- iSerialNumber: meant to be the device id in hex, as INFO DevId. Not
  filled yet: stage 0 passes no serial string and DevId is zero until the
  target layer gives a device id.
- The host accepts either `--port` or `--vid` / `--pid`, with optional
  `--serial`. USB identity lookup can rediscover a renamed port; a fixed
  port path cannot. Without a unique serial number, identical devices
  cannot be distinguished by VID/PID alone.
- The supplied nRF52840 DfuBoot uses VID 0x1915 / PID 0x5300. A product
  supplies its own USB identity.

## 11. Host tools

- Python/dfu_wire.py: requires pyserial; .hex input also requires intelhex.
  Reads a signed .bin or .hex, splits
  manifest and payload itself, no package file. Opens /dev/cu.* on macOS
  (not /dev/tty.*, which waits for carrier), sets DTR, raw mode.
  Timeout per WRITE = EraseMaxMs + transfer time + margin.
No Web Serial client is supplied. Use the Python tool.

## 12. Previously recorded size, -Os

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

## 15. Configuration and remaining work

- Downgrade is refused by default. Stage 0 maps `DFU_BOOT_ALLOW_DOWNGRADE`
  to `DfuMgrCfg_t::bAllowDowngrade`.
- ENTER is implemented; applications must supply its transport and polling.
- MaxBody defaults to 512 through `DFU_BOOT_BUFSIZE`.
- A USB DFU-class protocol and Web Serial client are not supplied.
- The recorded sizes and host results do not replace target builds and
  hardware recovery tests for the selected MCU, transport and memory layout.
