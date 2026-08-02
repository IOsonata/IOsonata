import base64
import zlib
from pathlib import Path

root = Path(__file__).resolve().parent
payload = "".join(
    (root / f"bt_fix_part_{index:02}.txt").read_text().strip()
    for index in range(13)
)
source = zlib.decompress(base64.b64decode(payload)).decode()

workflow_old = 'replace_once(\n    ".github/workflows/bluetooth-host-tests.yml",\n    \'\'\'      - "ARM/Nordic/nRF52/src/bt_gatt_nrf52.cpp"\n      - "ARM/Nordic/nRF54/src/bt_gatt_bm.cpp"\'\'\',\n    \'\'\'      - "ARM/**/src/bt_*.cpp"\'\'\',\n)\nreplace_once(\n    ".github/workflows/bluetooth-host-tests.yml",\n    \'\'\'      - "ARM/Nordic/nRF52/src/bt_gatt_nrf52.cpp"\n      - "ARM/Nordic/nRF54/src/bt_gatt_bm.cpp"\'\'\',\n    \'\'\'      - "ARM/**/src/bt_*.cpp"\'\'\',\n)'
workflow_new = 'path = ".github/workflows/bluetooth-host-tests.yml"\nold = \'\'\'      - "ARM/Nordic/nRF52/src/bt_gatt_nrf52.cpp"\n      - "ARM/Nordic/nRF54/src/bt_gatt_bm.cpp"\'\'\'\nnew = \'\'\'      - "ARM/**/src/bt_*.cpp"\'\'\'\ntext = read(path)\nif text.count(old) != 2:\n    raise RuntimeError(f"{path}: expected two target path blocks, found {text.count(old)}")\nwrite(path, text.replace(old, new))'

if source.count(workflow_old) != 1:
    raise RuntimeError(
        f"patch loader: expected workflow replacement block once, "
        f"found {source.count(workflow_old)}"
    )
source = source.replace(workflow_old, workflow_new)

marker = 'print("Bluetooth failure-path fixes applied")'
extras = r'''
replace_once(
    "src/bluetooth/bt_att.cpp",
    "__attribute__((weak)) bool BtGattCccdCanStore",
    """__attribute__((weak)) void BtGattCccdChanged(uint16_t ConnHdl,
                                             uint16_t CccdHdl,
                                             uint16_t OldValue)
{
    (void)ConnHdl;
    (void)CccdHdl;
    (void)OldValue;
}

__attribute__((weak)) bool BtGattCccdCanStore""",
)

replace_once(
    "src/bluetooth/bt_app.cpp",
    "\n\t\t.TxUntracked = 0,",
    "",
)

replace_once(
    "tests/bluetooth/host/bt_att_req_test.cpp",
    """bool BtGattCccdSet(uint16_t, uint16_t, uint16_t Value)
{
\t++g_CccdSetCount;
\tg_CccdSetVal = Value;
\treturn true;
}
void BtGattClientNotified""",
    """bool BtGattCccdSet(uint16_t, uint16_t, uint16_t Value)
{
\t++g_CccdSetCount;
\tg_CccdSetVal = Value;
\treturn true;
}
void BtGattCccdChanged(uint16_t, uint16_t CccdHdl, uint16_t)
{
\t++g_CccdSetCount;
\tfor (uint8_t i = 0; i < s_StubPeer.Conn.NbCccd; i++)
\t{
\t\tif (s_StubPeer.Conn.Cccd[i].Hdl == CccdHdl)
\t\t{
\t\t\tg_CccdSetVal = s_StubPeer.Conn.Cccd[i].Value;
\t\t\tbreak;
\t\t}
\t}
}
void BtGattClientNotified""",
)

replace_once(
    "tests/bluetooth/host/bt_att_adversarial_test.cpp",
    """\ts_CccdSetCount = 0;
\tstd::memset(rsp, 0, sizeof(rsp));
\treq[0] = BT_ATT_OPCODE_ATT_PREPARE_WRITE_REQ;""",
    """\ts_CccdSetResult = true;
\ts_CccdSetCount = 0;
\ts_Peer.Conn.NbCccd = BT_GATT_CCCD_STATE_MAX;
\tfor (uint8_t i = 0; i < BT_GATT_CCCD_STATE_MAX; i++)
\t{
\t\ts_Peer.Conn.Cccd[i].Hdl = (uint16_t)(0x3000 + i);
\t\ts_Peer.Conn.Cccd[i].Value =
\t\t\tBT_DESC_CLIENT_CHAR_CONFIG_NOTIFICATION;
\t}
\tstd::memset(rsp, 0, sizeof(rsp));
\treq[0] = BT_ATT_OPCODE_ATT_PREPARE_WRITE_REQ;""",
)

replace_once(
    "tests/bluetooth/host/bt_att_adversarial_test.cpp",
    """\tBT_CHECK(s_Test, rsp[4] == kInsufficientResources);
\tBT_CHECK(s_Test, s_CccdSetCount == 1);

\ts_PeerEnabled = false;
\ts_CccdSetResult = true;""",
    """\tBT_CHECK(s_Test, rsp[4] == kInsufficientResources);
\tBT_CHECK(s_Test, s_CccdSetCount == 0);
\tBT_CHECK(s_Test, s_Peer.Conn.NbCccd == BT_GATT_CCCD_STATE_MAX);

\ts_PeerEnabled = false;
\ts_CccdSetResult = true;""",
)

replace_once(
    "tests/bluetooth/host/bt_gatt_adversarial_test.cpp",
    """\tfor (int i = 0; i < 2; i++)
\t{
\t\tBtAttReqRsp_t request = {};
\t\tBtAttReqRsp_t response = {};
\t\trequest.OpCode = BT_ATT_OPCODE_ATT_PREPARE_WRITE_REQ;
\t\trequest.PrepareWriteReq.Hdl = chars[i].CccdHdl;
\t\trequest.PrepareWriteReq.Offset = 0;
\t\trequest.PrepareWriteReq.Data[0] = 1;
\t\trequest.PrepareWriteReq.Data[1] = 0;
\t\tuint32_t len = BtAttProcessReq(kConnHdl, &request, 7, &response);
\t\tBT_CHECK(s_Test, len == 7);
\t\tBT_CHECK(s_Test,
\t\t\t\t response.OpCode == BT_ATT_OPCODE_ATT_PREPARE_WRITE_RSP);
\t}""",
    """\tfor (int i = 0; i < 2; i++)
\t{
\t\tuint8_t request[7] = {};
\t\tuint8_t response[7] = {};
\t\trequest[0] = BT_ATT_OPCODE_ATT_PREPARE_WRITE_REQ;
\t\trequest[1] = (uint8_t)(chars[i].CccdHdl & 0xFF);
\t\trequest[2] = (uint8_t)(chars[i].CccdHdl >> 8);
\t\trequest[3] = 0;
\t\trequest[4] = 0;
\t\trequest[5] = 1;
\t\trequest[6] = 0;
\t\tuint32_t len = BtAttProcessReq(
\t\t\t\tkConnHdl,
\t\t\t\t(BtAttReqRsp_t *)request,
\t\t\t\tsizeof(request),
\t\t\t\t(BtAttReqRsp_t *)response);
\t\tBT_CHECK(s_Test, len == sizeof(response));
\t\tBT_CHECK(s_Test,
\t\t\t\t response[0] == BT_ATT_OPCODE_ATT_PREPARE_WRITE_RSP);
\t}""",
)

print("Bluetooth failure-path fixes applied")
'''

if source.count(marker) != 1:
    raise RuntimeError(
        f"patch loader: expected final marker once, found {source.count(marker)}"
    )
source = source.replace(marker, extras)
exec(compile(source, "<bt-fixes>", "exec"))
