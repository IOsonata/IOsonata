#!/usr/bin/env python3
"""One-shot source patcher for the bt_conformance review fixes.

The script is intentionally assertion-heavy: it refuses to continue when the
branch no longer matches the reviewed source instead of making a fuzzy edit.
"""

from __future__ import annotations

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def load(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def save(path: str, text: str) -> None:
    (ROOT / path).write_text(text, encoding="utf-8")


def replace_once(path: str, old: str, new: str) -> None:
    text = load(path)
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{path}: expected one occurrence, found {count}: {old[:80]!r}")
    save(path, text.replace(old, new, 1))


def replace_all(path: str, old: str, new: str, expected: int | None = None) -> None:
    text = load(path)
    count = text.count(old)
    if expected is not None and count != expected:
        raise RuntimeError(f"{path}: expected {expected} occurrences, found {count}: {old[:80]!r}")
    if count == 0:
        raise RuntimeError(f"{path}: no occurrences: {old[:80]!r}")
    save(path, text.replace(old, new))


def sub_once(path: str, pattern: str, repl: str, flags: int = re.S) -> None:
    text = load(path)
    text2, count = re.subn(pattern, repl, text, count=1, flags=flags)
    if count != 1:
        raise RuntimeError(f"{path}: regex expected one match, found {count}: {pattern[:100]!r}")
    save(path, text2)


# ---------------------------------------------------------------------------
# Compile blocker and per-link state used by the corrected transaction paths.
# ---------------------------------------------------------------------------
replace_once(
    "include/bluetooth/bt_dev.h",
    "\tBtDevDiscState_t Discovery;\t\t\t\t\t//!< Per-peer discovery cursor (remote role only)\n",
    "\tBtDevDiscState_t Discovery;\t\t\t\t\t//!< Per-peer discovery cursor (remote role only)\n"
    "\tvoid\t\t\t*pIndChar;\t\t\t\t\t//!< Vendor-host indication owner completed by HVC, null on raw HCI\n"
    "\tbool\t\t\tbAttReqPending;\t\t\t\t//!< One ATT client request may be outstanding per bearer\n"
    "\tbool\t\t\tbAttTimedOut;\t\t\t\t//!< ATT bearer timed out and may not start another request\n"
    "\tuint8_t\t\tAttReqOpcode;\t\t\t\t//!< Outstanding ATT request opcode\n"
    "\tuint8_t\t\tAttRspOpcode;\t\t\t\t//!< Expected ATT response opcode\n"
    "\tuint32_t\t\tAttReqTime;\t\t\t\t//!< Request start tick for the 30 second ATT timeout\n",
)

replace_once(
    "src/bluetooth/bt_app.cpp",
    "\t\t.Discovery  = {},\n\t\t.TxPend     = {},\n\t\t.TxPendHead = 0,\n\t\t.TxPendCount = 0,\n\t\t.TxUntracked = 0,\n",
    "\t\t.Discovery  = {},\n"
    "\t\t.pIndChar = NULL,\n"
    "\t\t.bAttReqPending = false,\n"
    "\t\t.bAttTimedOut = false,\n"
    "\t\t.AttReqOpcode = 0,\n"
    "\t\t.AttRspOpcode = 0,\n"
    "\t\t.AttReqTime = 0,\n"
    "\t\t.TxPend     = {},\n"
    "\t\t.TxPendHead = 0,\n"
    "\t\t.TxPendCount = 0,\n",
)

# ---------------------------------------------------------------------------
# Ordered ACL completion API: every caller must be able to detect saturation.
# ---------------------------------------------------------------------------
replace_once(
    "include/bluetooth/bt_gatt.h",
    "void BtGattTxPendUntracked(uint16_t ConnHdl, uint16_t NbPkt);\nvoid BtGattTxPendingAdd(uint16_t ConnHdl, BtGattChar_t *pChar);",
    "bool BtGattTxPendUntracked(uint16_t ConnHdl, uint16_t NbPkt);\nbool BtGattTxPendingAdd(uint16_t ConnHdl, BtGattChar_t *pChar);",
)

replace_once(
    "src/bluetooth/bt_gatt.cpp",
    "void BtGattTxPendUntracked(uint16_t ConnHdl, uint16_t NbPkt)\n{\n\t(void)BtGattTxPendReserve(ConnHdl, nullptr, NbPkt);\n}\n\nvoid BtGattTxPendingAdd(uint16_t ConnHdl, BtGattChar_t *pChar)\n{\n\tBtGattTxPendReserve(ConnHdl, pChar, 1);\n}",
    "bool BtGattTxPendUntracked(uint16_t ConnHdl, uint16_t NbPkt)\n{\n\treturn BtGattTxPendReserve(ConnHdl, nullptr, NbPkt);\n}\n\nbool BtGattTxPendingAdd(uint16_t ConnHdl, BtGattChar_t *pChar)\n{\n\treturn BtGattTxPendReserve(ConnHdl, pChar, 1);\n}",
)

# Share callback dispatch between Number Of Completed Packets and vendor HVC.
replace_once(
    "src/bluetooth/bt_gatt.cpp",
    "void BtGattHandleValueConfirm(uint16_t ConnHdl)\n{\n\tBtDevice_t *pConn = BtPeerFindByHdl(ConnHdl);\n\tif (pConn != nullptr)\n\t{\n\t\tpConn->Conn.bIndCfmPending = false;\n\t}\n}",
    "static void BtGattTxCompleteChar(BtGattChar_t *pChar)\n"
    "{\n"
    "\tif (pChar == nullptr || pChar->TxCompleteCB == nullptr)\n"
    "\t{\n"
    "\t\treturn;\n"
    "\t}\n\n"
    "\tint idx = 0;\n"
    "\tif (pChar->pSrvc != nullptr)\n"
    "\t{\n"
    "\t\tfor (int k = 0; k < pChar->pSrvc->NbChar; k++)\n"
    "\t\t{\n"
    "\t\t\tif (&pChar->pSrvc->pCharArray[k] == pChar)\n"
    "\t\t\t{\n"
    "\t\t\t\tidx = k;\n"
    "\t\t\t\tbreak;\n"
    "\t\t\t}\n"
    "\t\t}\n"
    "\t}\n"
    "\tpChar->TxCompleteCB(pChar, idx);\n"
    "}\n\n"
    "void BtGattHandleValueConfirm(uint16_t ConnHdl)\n"
    "{\n"
    "\tBtDevice_t *pConn = BtPeerFindByHdl(ConnHdl);\n"
    "\tif (pConn == nullptr)\n"
    "\t{\n"
    "\t\treturn;\n"
    "\t}\n\n"
    "\tpConn->Conn.bIndCfmPending = false;\n"
    "\tBtGattChar_t *pChar = (BtGattChar_t *)pConn->pIndChar;\n"
    "\tpConn->pIndChar = nullptr;\n"
    "\tBtGattTxCompleteChar(pChar);\n"
    "}",
)

sub_once(
    "src/bluetooth/bt_gatt.cpp",
    r"\n\t\tif \(c == nullptr \|\| c->TxCompleteCB == nullptr\)\n\t\t\{\n\t\t\tcontinue;\n\t\t\}\n\n\t\tint idx = 0;\n\t\tif \(c->pSrvc != nullptr\)\n\t\t\{.*?\n\t\tc->TxCompleteCB\(c, idx\);",
    "\n\t\tBtGattTxCompleteChar(c);",
)

replace_once(
    "src/bluetooth/bt_gatt.cpp",
    "\tpConn->Conn.bIndCfmPending = false;\n\n\tif (pConn->pHciDev != nullptr",
    "\tpConn->Conn.bIndCfmPending = false;\n\tpConn->pIndChar = nullptr;\n\n\tif (pConn->pHciDev != nullptr",
)

# ---------------------------------------------------------------------------
# Reserve response entries before transmission. A full ring now back-pressures
# ATT/L2CAP instead of silently losing completion order.
# ---------------------------------------------------------------------------
sub_once(
    "src/bluetooth/bt_hci_host.cpp",
    r"\t\t\t\tuint16_t nbpkt = BtHciAclPktCount\(pDev, acl->Hdr.Len\);\n\t\t\t\tuint32_t sent = BtHciSendAcl\(pDev, acl\);\n\t\t\t\tif \(sent == 0\)\n\t\t\t\t\{\n\t\t\t\t\tSysLogPrintf\(SysLogGet\(\),\n\t\t\t\t\t\t\"ATT TX FAILED opcode=0x%02x rsp=0x%02x len=%u\\r\\n\",\n\t\t\t\t\t\tl2rcv->Att.OpCode, l2pdu->Att.OpCode, acl->Hdr.Len\);\n\t\t\t\t\}\n\t\t\t\telse\n\t\t\t\t\{\n\t\t\t\t\tBtGattTxPendUntracked\(pPkt->Hdr.ConnHdl, nbpkt\);\n\t\t\t\t\}",
    "\t\t\t\tuint16_t nbpkt = BtHciAclPktCount(pDev, acl->Hdr.Len);\n"
    "\t\t\t\tif (BtGattTxPendUntracked(pPkt->Hdr.ConnHdl, nbpkt) == false)\n"
    "\t\t\t\t{\n"
    "\t\t\t\t\tSysLogPrintf(SysLogGet(),\n"
    "\t\t\t\t\t\t\"ATT TX DEFERRED completion ring full opcode=0x%02x len=%u\\r\\n\",\n"
    "\t\t\t\t\t\tl2pdu->Att.OpCode, acl->Hdr.Len);\n"
    "\t\t\t\t}\n"
    "\t\t\t\telse if (BtHciSendAcl(pDev, acl) == 0)\n"
    "\t\t\t\t{\n"
    "\t\t\t\t\tBtGattTxPendRelease(pPkt->Hdr.ConnHdl);\n"
    "\t\t\t\t\tSysLogPrintf(SysLogGet(),\n"
    "\t\t\t\t\t\t\"ATT TX FAILED opcode=0x%02x rsp=0x%02x len=%u\\r\\n\",\n"
    "\t\t\t\t\t\tl2rcv->Att.OpCode, l2pdu->Att.OpCode, acl->Hdr.Len);\n"
    "\t\t\t\t}",
)

sub_once(
    "src/bluetooth/bt_hci_host.cpp",
    r"\t\t\t\tuint16_t nbpkt = BtHciAclPktCount\(pDev, acl->Hdr.Len\);\n\t\t\t\tuint32_t sent = BtHciSendAcl\(pDev, acl\);\n\t\t\t\tif \(sent == 0\)\n\t\t\t\t\{\n\t\t\t\t\tSysLogPrintf\(SysLogGet\(\),\n\t\t\t\t\t\t\"L2CAP SIG TX FAILED code=0x%02x len=%u\\r\\n\",\n\t\t\t\t\t\tl2rcv->CFrame.Code, acl->Hdr.Len\);\n\t\t\t\t\}\n\t\t\t\telse\n\t\t\t\t\{\n\t\t\t\t\tBtGattTxPendUntracked\(pPkt->Hdr.ConnHdl, nbpkt\);\n\t\t\t\t\}",
    "\t\t\t\tuint16_t nbpkt = BtHciAclPktCount(pDev, acl->Hdr.Len);\n"
    "\t\t\t\tif (BtGattTxPendUntracked(pPkt->Hdr.ConnHdl, nbpkt) == false)\n"
    "\t\t\t\t{\n"
    "\t\t\t\t\tSysLogPrintf(SysLogGet(),\n"
    "\t\t\t\t\t\t\"L2CAP SIG TX DEFERRED completion ring full code=0x%02x len=%u\\r\\n\",\n"
    "\t\t\t\t\t\tl2rcv->CFrame.Code, acl->Hdr.Len);\n"
    "\t\t\t\t}\n"
    "\t\t\t\telse if (BtHciSendAcl(pDev, acl) == 0)\n"
    "\t\t\t\t{\n"
    "\t\t\t\t\tBtGattTxPendRelease(pPkt->Hdr.ConnHdl);\n"
    "\t\t\t\t\tSysLogPrintf(SysLogGet(),\n"
    "\t\t\t\t\t\t\"L2CAP SIG TX FAILED code=0x%02x len=%u\\r\\n\",\n"
    "\t\t\t\t\t\tl2rcv->CFrame.Code, acl->Hdr.Len);\n"
    "\t\t\t\t}",
)

# ---------------------------------------------------------------------------
# ATT public primitive safety and legal Attribute Type encoding.
# ---------------------------------------------------------------------------
replace_once(
    "src/bluetooth/bt_att.cpp",
    "\t\t\t*(uint16_t*) pBuff = d->CccVal;\n\t\t\tDEBUG_PRINTF(\"Hdl = %d, CCC val = %d\\r\\n\", pEntry->Hdl, d->CccVal);\n\t\t\tlen = 2;",
    "\t\t\tif (Offset < 2 && Len > 0)\n"
    "\t\t\t{\n"
    "\t\t\t\tuint8_t value[2] = { (uint8_t)(d->CccVal & 0xFF), (uint8_t)(d->CccVal >> 8) };\n"
    "\t\t\t\tsize_t l = min((size_t)(2 - Offset), Len);\n"
    "\t\t\t\tmemcpy(pBuff, value + Offset, l);\n"
    "\t\t\t\tlen = l;\n"
    "\t\t\t}\n"
    "\t\t\tDEBUG_PRINTF(\"Hdl = %d, CCC val = %d\\r\\n\", pEntry->Hdl, d->CccVal);",
)

replace_once(
    "src/bluetooth/bt_att.cpp",
    "\t\t\t\t\tif (Len < 2)\n\t\t\t\t\t{\n\t\t\t\t\t\tbreak;\n\t\t\t\t\t}\n\n\t\t\t\t\tp->CccVal = *(uint16_t*)pData;",
    "\t\t\t\t\tif (Len != 2 || pData == nullptr)\n"
    "\t\t\t\t\t{\n"
    "\t\t\t\t\t\tbreak;\n"
    "\t\t\t\t\t}\n\n"
    "\t\t\t\t\tp->CccVal = (uint16_t)(pData[0] | ((uint16_t)pData[1] << 8));",
)

for fn in ("BtAttReadByTypeRequest", "BtAttReadByGroupTypeRequest"):
    path = "src/bluetooth/bt_attreq.cpp"
    text = load(path)
    marker = f"bool {fn}"
    start = text.find(marker)
    if start < 0:
        raise RuntimeError(f"{path}: missing {fn}")
    end = text.find("\n}\n", start)
    if end < 0:
        raise RuntimeError(f"{path}: cannot locate end of {fn}")
    end += 3
    block = text[start:end]
    old = "\telse if (pUuid->Type == BT_UUID_TYPE_32)\n\t{\n\t\tl2pdu->Att."
    if old not in block:
        raise RuntimeError(f"{path}: missing 32-bit branch in {fn}")
    # Replace the complete four-byte branch regardless of request union member.
    block2, n = re.subn(
        r"\telse if \(pUuid->Type == BT_UUID_TYPE_32\)\n\t\{\n\t\tl2pdu->Att\.(ReadByTypeReq|ReadByGroupTypeReq)\.Uuid\.Uuid32 = pUuid->Uuid32;\n\t\tl2pdu->Hdr.Len \+= 4;\n\t\}",
        lambda m: (
            "\telse if (pUuid->Type == BT_UUID_TYPE_32)\n"
            "\t{\n"
            "\t\tBtUuid_t full = *pUuid;\n"
            "\t\tfull.BaseIdx = 0;\n"
            "\t\tfull.Type = BT_UUID_TYPE_32;\n"
            f"\t\tBtUuidTo128(&full, l2pdu->Att.{m.group(1)}.Uuid.Uuid128);\n"
            "\t\tl2pdu->Hdr.Len += 16;\n"
            "\t}"
        ),
        block,
        count=1,
    )
    if n != 1:
        raise RuntimeError(f"{path}: failed to replace 32-bit branch in {fn}")
    save(path, text[:start] + block2 + text[end:])

# ---------------------------------------------------------------------------
# L2CAP never rejects a response or Identifier 0, including malformed lengths.
# ---------------------------------------------------------------------------
replace_once(
    "src/bluetooth/bt_l2cap.cpp",
    "\t\tif (cmdLen > (remain - cmdHdrLen))\n\t\t{\n\t\t\tBtL2CapAppendCmdReject(pOut, &outLen, outMax, pCmd->Id,\n\t\t\t\t\t\t\t\t\tBT_L2CAP_CMD_REJECT_REASON_NOT_UNDERSTOOD,\n\t\t\t\t\t\t\t\t\tnullptr, 0);\n\t\t\tbreak;\n\t\t}",
    "\t\tif (cmdLen > (remain - cmdHdrLen))\n"
    "\t\t{\n"
    "\t\t\tif (BtL2CapCodeIsResponse(pCmd->Code) == false && pCmd->Id != 0)\n"
    "\t\t\t{\n"
    "\t\t\t\tBtL2CapAppendCmdReject(pOut, &outLen, outMax, pCmd->Id,\n"
    "\t\t\t\t\t\t\t\t\tBT_L2CAP_CMD_REJECT_REASON_NOT_UNDERSTOOD,\n"
    "\t\t\t\t\t\t\t\t\tnullptr, 0);\n"
    "\t\t\t}\n"
    "\t\t\tbreak;\n"
    "\t\t}",
)

# ---------------------------------------------------------------------------
# Extended advertising input validation before signed arithmetic/memcpy.
# ---------------------------------------------------------------------------
replace_once(
    "src/bluetooth/bt_adv_hci.cpp",
    "\tif (g_BtAppData.State != BTAPP_STATE_ADVERTISING && g_BtAppData.State != BTAPP_STATE_IDLE)\n\t{",
    "\tif (AdvLen < 0 || SrLen < 0 ||\n"
    "\t\t(pAdvData == nullptr && AdvLen != 0) ||\n"
    "\t\t(pSrData == nullptr && SrLen != 0))\n"
    "\t{\n"
    "\t\treturn false;\n"
    "\t}\n\n"
    "\tif (g_BtAppData.State != BTAPP_STATE_ADVERTISING && g_BtAppData.State != BTAPP_STATE_IDLE)\n"
    "\t{",
)

# ---------------------------------------------------------------------------
# Native Nordic ports: reserve notifications before hvx, complete indications
# on HVC rather than the notification-only completion event, update both CCCD
# bits, and bound long-write parsing.
# ---------------------------------------------------------------------------

def patch_nordic(path: str, handle_name: str, old_pending_arg: str, has_null_check: bool) -> None:
    text = load(path)

    notify_old = (
        f"    uint32_t err_code = sd_ble_gatts_hvx({handle_name}, &params);\n\n"
        "    if (err_code == NRF_SUCCESS)\n"
        "    {\n"
        f"        BtGattTxPendingAdd({old_pending_arg}, pChar);\n"
        "        return true;\n"
        "    }\n\n"
        "    return false;"
    )
    notify_new = (
        f"    if (BtGattTxPendReserve({handle_name}, pChar, 1) == false)\n"
        "    {\n"
        "        return false;\n"
        "    }\n\n"
        f"    uint32_t err_code = sd_ble_gatts_hvx({handle_name}, &params);\n\n"
        "    if (err_code == NRF_SUCCESS)\n"
        "    {\n"
        "        return true;\n"
        "    }\n\n"
        f"    BtGattTxPendRelease({handle_name});\n"
        "    return false;"
    )
    if notify_old not in text:
        # nRF54 uses tabs rather than spaces; use regex fallback.
        pattern = (
            rf"\tuint32_t err_code = sd_ble_gatts_hvx\({handle_name}, &params\);\n\n"
            rf"\tif \(err_code == NRF_SUCCESS\)\n\t\{{\n\t\tBtGattTxPendingAdd\({old_pending_arg}, pChar\);\n\t\treturn true;\n\t\}}\n\n\treturn false;"
        )
        repl = notify_new.replace("    ", "\t")
        text, n = re.subn(pattern, repl, text, count=1)
        if n != 1:
            raise RuntimeError(f"{path}: notification block not found")
    else:
        text = text.replace(notify_old, notify_new, 1)

    # Indication: retain transaction timeout state, but callback ownership is
    # completed by HVC and never inserted into the HVN-only ring.
    pattern = (
        rf"(uint32_t err_code = sd_ble_gatts_hvx\({handle_name}, &params\);\n\n"
        r"\s*if \(err_code == NRF_SUCCESS\)\n\s*\{\n"
        rf"\s*BtDevice_t \*pConn = BtPeerFindByHdl\({handle_name}\);\n"
        r"\s*if \(pConn != nullptr\)\n\s*\{\n"
        r"\s*pConn->Conn.bIndCfmPending = true;\n"
        r"\s*pConn->Conn.IndCfmTime = BtGattMsTick\(\);\n"
        r"\s*\}\n)"
        rf"\s*BtGattTxPendingAdd\({handle_name}, pChar\);"
    )
    repl = r"\1            pConn->pIndChar = pChar;\n        }"
    # The replacement above also closes the pConn block; replace the complete
    # success body instead when indentation differs.
    text2, n = re.subn(pattern, repl, text, count=1)
    if n != 1:
        # More reliable complete-block form.
        pattern2 = (
            rf"(?P<indent>[ \t]*)uint32_t err_code = sd_ble_gatts_hvx\({handle_name}, &params\);\n\n"
            rf"(?P=indent)if \(err_code == NRF_SUCCESS\)\n(?P=indent)\{{\n"
            rf"(?P<body>.*?BtGattTxPendingAdd\({handle_name}, pChar\);\n)"
            rf"(?P=indent)\t?return true;\n(?P=indent)\}}\n\n(?P=indent)return false;"
        )
        m = re.search(pattern2, text, re.S)
        if not m:
            raise RuntimeError(f"{path}: indication block not found")
        ind = m.group("indent")
        replacement = (
            f"{ind}uint32_t err_code = sd_ble_gatts_hvx({handle_name}, &params);\n\n"
            f"{ind}if (err_code == NRF_SUCCESS)\n"
            f"{ind}{{\n"
            f"{ind}\tBtDevice_t *pConn = BtPeerFindByHdl({handle_name});\n"
            f"{ind}\tif (pConn != nullptr)\n"
            f"{ind}\t{{\n"
            f"{ind}\t\tpConn->Conn.bIndCfmPending = true;\n"
            f"{ind}\t\tpConn->Conn.IndCfmTime = BtGattMsTick();\n"
            f"{ind}\t\tpConn->pIndChar = pChar;\n"
            f"{ind}\t}}\n"
            f"{ind}\treturn true;\n"
            f"{ind}}}\n\n"
            f"{ind}return false;"
        )
        text = text[:m.start()] + replacement + text[m.end():]
    else:
        text = text2

    # Disconnect must clear both aggregate mirrors.
    text = text.replace(
        "\t\t\tpSrvc->pCharArray[i].bNotify = false;\n\t\t}",
        "\t\t\tpSrvc->pCharArray[i].bNotify = false;\n\t\t\tpSrvc->pCharArray[i].bIndic  = false;\n\t\t}",
        1,
    )

    # Replace CCCD notify-only handling with complete two-bit handling.
    cccd_pattern = r"if \(ble_srv_is_notification_enabled\(p_evt_write->data\)\).*?// Set notify callback\n\s*if \(pSrvc->pCharArray\[i\]\.SetNotifCB\)\n\s*\{\n\s*pSrvc->pCharArray\[i\]\.SetNotifCB\(&pSrvc->pCharArray\[i\], pSrvc->pCharArray\[i\]\.bNotify, pBleEvt->evt.gatts_evt.conn_handle\);\n\s*\}"
    if "ble_srv_is_notification_enabled" in text:
        cccd_repl = (
            "uint16_t cccd = (uint16_t)(p_evt_write->data[0] |\n"
            "\t\t\t\t\t\t\t\t\t\t ((uint16_t)p_evt_write->data[1] << 8));\n"
            "\t\t\t\t\t\t\tpSrvc->pCharArray[i].bNotify =\n"
            "\t\t\t\t\t\t\t\t(cccd & BT_DESC_CLIENT_CHAR_CONFIG_NOTIFICATION) != 0;\n"
            "\t\t\t\t\t\t\tpSrvc->pCharArray[i].bIndic =\n"
            "\t\t\t\t\t\t\t\t(cccd & BT_DESC_CLIENT_CHAR_CONFIG_INDICATION) != 0;\n"
            "\t\t\t\t\t\t\tif (pSrvc->pCharArray[i].SetNotifCB)\n"
            "\t\t\t\t\t\t\t{\n"
            "\t\t\t\t\t\t\t\tpSrvc->pCharArray[i].SetNotifCB(&pSrvc->pCharArray[i],\n"
            "\t\t\t\t\t\t\t\t\tpSrvc->pCharArray[i].bNotify,\n"
            "\t\t\t\t\t\t\t\t\tpBleEvt->evt.gatts_evt.conn_handle);\n"
            "\t\t\t\t\t\t\t}\n"
            "\t\t\t\t\t\t\tif (pSrvc->pCharArray[i].SetIndCB)\n"
            "\t\t\t\t\t\t\t{\n"
            "\t\t\t\t\t\t\t\tpSrvc->pCharArray[i].SetIndCB(&pSrvc->pCharArray[i],\n"
            "\t\t\t\t\t\t\t\t\tpSrvc->pCharArray[i].bIndic,\n"
            "\t\t\t\t\t\t\t\t\tpBleEvt->evt.gatts_evt.conn_handle);\n"
            "\t\t\t\t\t\t\t}"
        )
        text, n = re.subn(cccd_pattern, cccd_repl, text, count=1, flags=re.S)
        if n != 1:
            raise RuntimeError(f"{path}: nRF52 CCCD block not found")
    else:
        pattern = (
            r"if \(IsNotificationEnabled\(p_evt_write->data\)\).*?"
            r"if \(pSrvc->pCharArray\[i\]\.SetNotifCB\)\n\s*\{\n"
            r"\s*pSrvc->pCharArray\[i\]\.SetNotifCB\(&pSrvc->pCharArray\[i\], pSrvc->pCharArray\[i\]\.bNotify, pBleEvt->evt.gatts_evt.conn_handle\);\n\s*\}"
        )
        repl = (
            "uint16_t cccd = (uint16_t)(p_evt_write->data[0] |\n"
            "\t\t\t\t\t\t\t\t\t ((uint16_t)p_evt_write->data[1] << 8));\n"
            "\t\t\t\t\t\t\tpSrvc->pCharArray[i].bNotify =\n"
            "\t\t\t\t\t\t\t\t(cccd & BT_DESC_CLIENT_CHAR_CONFIG_NOTIFICATION) != 0;\n"
            "\t\t\t\t\t\t\tpSrvc->pCharArray[i].bIndic =\n"
            "\t\t\t\t\t\t\t\t(cccd & BT_DESC_CLIENT_CHAR_CONFIG_INDICATION) != 0;\n"
            "\t\t\t\t\t\t\tif (pSrvc->pCharArray[i].SetNotifCB)\n"
            "\t\t\t\t\t\t\t{\n"
            "\t\t\t\t\t\t\t\tpSrvc->pCharArray[i].SetNotifCB(&pSrvc->pCharArray[i],\n"
            "\t\t\t\t\t\t\t\t\tpSrvc->pCharArray[i].bNotify,\n"
            "\t\t\t\t\t\t\t\t\tpBleEvt->evt.gatts_evt.conn_handle);\n"
            "\t\t\t\t\t\t\t}\n"
            "\t\t\t\t\t\t\tif (pSrvc->pCharArray[i].SetIndCB)\n"
            "\t\t\t\t\t\t\t{\n"
            "\t\t\t\t\t\t\t\tpSrvc->pCharArray[i].SetIndCB(&pSrvc->pCharArray[i],\n"
            "\t\t\t\t\t\t\t\t\tpSrvc->pCharArray[i].bIndic,\n"
            "\t\t\t\t\t\t\t\t\tpBleEvt->evt.gatts_evt.conn_handle);\n"
            "\t\t\t\t\t\t\t}"
        )
        text, n = re.subn(pattern, repl, text, count=1, flags=re.S)
        if n != 1:
            raise RuntimeError(f"{path}: nRF54 CCCD block not found")

    # Bounded, iterative long-write record gathering.
    text, n = re.subn(
        r"(?:static )?void GatherLongWrBuff\(GATLWRHDR \*pHdr\)\n\{.*?\n\}",
        "static bool GatherLongWrBuff(GATLWRHDR *pHdr, uint8_t *pBase, uint16_t BuffSize)\n"
        "{\n"
        "\tif (pHdr == nullptr || pBase == nullptr || BuffSize < sizeof(GATLWRHDR))\n"
        "\t{\n"
        "\t\treturn false;\n"
        "\t}\n\n"
        "\tuint8_t *end = pBase + BuffSize;\n"
        "\tuint8_t *first = (uint8_t *)pHdr;\n"
        "\tif (first < pBase || first + sizeof(GATLWRHDR) > end ||\n"
        "\t\t(uint32_t)(end - first - sizeof(GATLWRHDR)) < pHdr->Len)\n"
        "\t{\n"
        "\t\treturn false;\n"
        "\t}\n\n"
        "\tuint32_t total = pHdr->Len;\n"
        "\tuint8_t *write = first + sizeof(GATLWRHDR) + pHdr->Len;\n"
        "\tuint8_t *scan = write;\n\n"
        "\twhile (scan + sizeof(GATLWRHDR) <= end)\n"
        "\t{\n"
        "\t\tGATLWRHDR next;\n"
        "\t\tmemcpy(&next, scan, sizeof(next));\n"
        "\t\tif (next.Handle != pHdr->Handle)\n"
        "\t\t{\n"
        "\t\t\tbreak;\n"
        "\t\t}\n"
        "\t\tif (next.Offset != (uint16_t)(pHdr->Offset + total) ||\n"
        "\t\t\ttotal + next.Len > UINT16_MAX ||\n"
        "\t\t\t(uint32_t)(end - scan - sizeof(GATLWRHDR)) < next.Len)\n"
        "\t\t{\n"
        "\t\t\treturn false;\n"
        "\t\t}\n\n"
        "\t\tmemmove(write, scan + sizeof(GATLWRHDR), next.Len);\n"
        "\t\twrite += next.Len;\n"
        "\t\ttotal += next.Len;\n"
        "\t\tscan += sizeof(GATLWRHDR) + next.Len;\n"
        "\t}\n\n"
        "\tpHdr->Len = (uint16_t)total;\n"
        "\treturn true;\n"
        "}",
        text,
        count=1,
        flags=re.S,
    )
    if n != 1:
        raise RuntimeError(f"{path}: long-write gather function not found")

    # Update call and require a valid callback.
    text = text.replace(
        "GatherLongWrBuff(hdr);\n",
        "if (GatherLongWrBuff(hdr, pLongWr, pLwConn->Conn.LongWrBuffSize) == false)\n"
        "\t\t\t\t\t\t\t{\n"
        "\t\t\t\t\t\t\t\tcontinue;\n"
        "\t\t\t\t\t\t\t}\n",
        1,
    )
    if "GatherLongWrBuff(hdr);" in text:
        raise RuntimeError(f"{path}: unpatched GatherLongWrBuff call")

    # nRF52 had an unconditional long-write callback.
    text = text.replace(
        "pSrvc->pCharArray[i].WrCB(&pSrvc->pCharArray[i], p, hdr->Offset, hdr->Len);",
        "if (pSrvc->pCharArray[i].WrCB != nullptr)\n"
        "\t\t\t\t\t\t\t\t{\n"
        "\t\t\t\t\t\t\t\t\tpSrvc->pCharArray[i].WrCB(&pSrvc->pCharArray[i], p, hdr->Offset, hdr->Len);\n"
        "\t\t\t\t\t\t\t\t}",
        1,
    )

    save(path, text)


patch_nordic("ARM/Nordic/nRF52/src/bt_gatt_nrf52.cpp", "hdl", "ConnHdl", False)
patch_nordic("ARM/Nordic/nRF54/src/bt_gatt_bm.cpp", "ch", "ConnHdl", True)

print("bt_conformance source patch applied")
