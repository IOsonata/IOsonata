from pathlib import Path

path = Path("ARM/Nordic/nRF54/src/bt_app_bm.cpp")
text = path.read_text(encoding="utf-8")

old_bridge = '''// SoftDevice passkey octets are six ASCII digits, most significant first.
// Convert to the six digit integer used by the BtSmp interaction API.
static uint32_t BmPasskeyToVal(const uint8_t *pAscii)
'''

new_bridge = '''// Pairing configuration supplied through the common BtSmp API. The S145
// SoftDevice owns SMP, so this target stores the values until Peer Manager
// builds its ble_gap_sec_params_t.
static bool    s_bSmpAuthCfgValid;
static uint8_t s_SmpIoCaps = BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT;
static uint8_t s_SmpAuthReq =
\tBT_SMP_AUTHREQ_BONDING_FLAG_BONDING | BT_SMP_AUTHREQ_SC;

void BtSmpAuthConfig(uint8_t IoCaps, uint8_t AuthReq)
{
\tif (IoCaps > BT_SMP_IOCAPS_KEYBOARD_DISPLAY)
\t{
\t\treturn;
\t}

\ts_SmpIoCaps = IoCaps;
\ts_SmpAuthReq = (uint8_t)(AuthReq | BT_SMP_AUTHREQ_SC);
\ts_bSmpAuthCfgValid = true;
}

static void BmSmpAuthApply(ble_gap_sec_params_t *pSecParams)
{
\tif (!s_bSmpAuthCfgValid || pSecParams == nullptr)
\t{
\t\treturn;
\t}

\tswitch (s_SmpIoCaps)
\t{
\t\tcase BT_SMP_IOCAPS_DISPLAY_ONLY:
\t\t\tpSecParams->io_caps = BLE_GAP_IO_CAPS_DISPLAY_ONLY;
\t\t\tbreak;
\t\tcase BT_SMP_IOCAPS_DISPLAY_YESNO:
\t\t\tpSecParams->io_caps = BLE_GAP_IO_CAPS_DISPLAY_YESNO;
\t\t\tbreak;
\t\tcase BT_SMP_IOCAPS_KEYBOARD_ONLY:
\t\t\tpSecParams->io_caps = BLE_GAP_IO_CAPS_KEYBOARD_ONLY;
\t\t\tbreak;
\t\tcase BT_SMP_IOCAPS_KEYBOARD_DISPLAY:
\t\t\tpSecParams->io_caps = BLE_GAP_IO_CAPS_KEYBOARD_DISPLAY;
\t\t\tbreak;
\t\tcase BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT:
\t\tdefault:
\t\t\tpSecParams->io_caps = BLE_GAP_IO_CAPS_NONE;
\t\t\tbreak;
\t}

\tpSecParams->bond =
\t\t(s_SmpAuthReq & BT_SMP_AUTHREQ_BONDING_FLAG_MASK) !=
\t\tBT_SMP_AUTHREQ_BONDING_FLAG_NO_BONDING;
\tpSecParams->mitm = (s_SmpAuthReq & BT_SMP_AUTHREQ_MITM) != 0U;
\tpSecParams->lesc = 1;
\tpSecParams->keypress = (s_SmpAuthReq & BT_SMP_AUTHREQ_KEYPRESS) != 0U;
}

// SoftDevice passkey octets are six ASCII digits, most significant first.
// Convert to the six digit integer used by the BtSmp interaction API.
static uint32_t BmPasskeyToVal(const uint8_t *pAscii)
'''

if text.count(old_bridge) != 1:
    raise RuntimeError("unexpected SMP bridge insertion point")
text = text.replace(old_bridge, new_bridge, 1)

old_apply = '''\tif (SecKeyExchg & BTAPP_SECEXCHG_OOB)
\t{
\t\tsec_param.oob = 1;
\t}

\terr_code = pm_sec_params_set(&sec_param);
'''

new_apply = '''\tif (SecKeyExchg & BTAPP_SECEXCHG_OOB)
\t{
\t\tsec_param.oob = 1;
\t}

\t// BtSmpAuthConfig is the common application-facing API. On this target it
\t// overrides the association parameters before Peer Manager stores them.
\tBmSmpAuthApply(&sec_param);

\terr_code = pm_sec_params_set(&sec_param);
'''

if text.count(old_apply) != 1:
    raise RuntimeError("unexpected security parameter insertion point")
text = text.replace(old_apply, new_apply, 1)

required = (
    "void BtSmpAuthConfig(uint8_t IoCaps, uint8_t AuthReq)",
    "BmSmpAuthApply(&sec_param);",
    "BLE_GAP_IO_CAPS_DISPLAY_YESNO",
)
for item in required:
    if item not in text:
        raise RuntimeError(f"missing expected output: {item}")

path.write_text(text, encoding="utf-8")
