# Bluetooth PTS / Interop Matrix

This file tracks implementation and validation status. It is not a qualification
claim. A row should only be marked PTS tested after running the relevant
Bluetooth SIG Profile Tuning Suite cases for that backend.

| Area | Generic/SDC | SoftDevice | STM32WBA | Unit tested | Phone interop | PTS tested | Notes |
|---|---:|---:|---:|---:|---:|---:|---|
| GAP Peripheral | Partial | Vendor stack | Vendor stack | No | Partial | No | Adv/connect works per port; privacy still incomplete. |
| GAP Central | Partial | Vendor stack | Vendor stack | No | Partial | No | Discovery paths exist; multi-peer behavior still needs coverage. |
| ATT Server | Partial | Vendor stack | Vendor stack | No | Partial | No | Generic ATT supports core read/write/discovery subset. |
| ATT Client | Partial | Vendor stack | Vendor stack | No | Partial | No | Discovery/read/write client paths need state-machine coverage. |
| GATT Server | Partial | Vendor stack | Vendor stack | No | Partial | No | Generic service model plus per-peer CCCD on native host. |
| GATT Client | Partial | Vendor stack | Vendor stack | No | Partial | No | Needs descriptor discovery/subscription tests. |
| L2CAP LE Signaling | Partial | N/A | Vendor stack | No | No | No | Generic/SDC has deterministic signaling fallback. |
| L2CAP LE Credit Based Channels | No | N/A | Vendor stack | No | No | No | Dynamic CoC data path not implemented. |
| SMP Peripheral | Partial | Vendor stack | Vendor stack | No | Partial | No | Just Works/Bonding work exists; MITM/Passkey/OOB incomplete. |
| SMP Central | Partial | Vendor stack | Vendor stack | No | No | No | Needs role-specific validation. |
| Signed Write Command | Verified via bond CSRK | Vendor stack | Vendor stack | No | Partial | No | Signed Write verified through BtSmpSignVerify using the bond CSRK; sign counter and signature checked, rejected on mismatch. |
| Device Information Service | Yes | Vendor/Generic | Vendor/Generic | No | Partial | No | Generic DIS module exists. |
| Battery Service | Yes | Generic module | Generic module | No | No | No | Generic BAS module exposes Battery Level read/notify. |
| Bond Management Service | No | No | No | No | No | No | Requires security/bond-delete policy first. |
| Current Time Service | No | No | No | No | No | No | Candidate simple adopted service. |
| HID Service / HOGP | No | No | No | No | No | No | Do after security/CCCD/PTS baseline. |

## Immediate validation checklist

1. Build generic/SDC target with `src/bluetooth/bt_l2cap.cpp`, `bt_bas.cpp`, and `bt_dis.cpp`.
2. Confirm ATT Find Information returns multiple handles until MTU or UUID-format break.
3. Confirm Signed Write Command verifies the signature against the bond CSRK and rejects a bad signature or stale sign counter.
4. Confirm `BtAttDBEntrySetPermission()` rejects access when security hooks report insufficient link state.
5. Confirm Battery Level read returns one byte and notification only sends after CCCD subscribe.
