#!/usr/bin/env python3
from __future__ import annotations

import re
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def rep(text: str, old: str, new: str, label: str) -> str:
	count = text.count(old)
	if count != 1:
		raise RuntimeError(f"{label}: expected one match, found {count}")
	return text.replace(old, new, 1)


def sub(text: str, pattern: str, repl: str, label: str, flags: int = 0) -> str:
	text, count = re.subn(pattern, repl, text, count=1, flags=flags)
	if count != 1:
		raise RuntimeError(f"{label}: expected one match, found {count}")
	return text


def patch_sec() -> None:
	path = ROOT / "ARM/Nordic/nRF54/src/bt_sec_bm.cpp"
	text = path.read_text()

	text = rep(text,
		'#if defined(CONFIG_PM_LESC)\n#include <bm/bluetooth/peer_manager/nrf_ble_lesc.h>\n#endif',
		'#if defined(CONFIG_PM_LESC)\n#include "bt_lesc.h"\n#endif', "sec LESC include")
	text = rep(text, "return nrf_ble_lesc_public_key_get();",
		"return BtLescPubKeyGet();", "sec LESC public key")
	text = rep(text, "// ---- Keyset construction ----------------------------------------------------\n",
		"static void WriteBufRelease(uint16_t ConnHdl);\n\n"
		"// ---- Keyset construction ----------------------------------------------------\n",
		"sec release declaration")

	text = rep(text,
		"\tBtSecBmLink_t *pLink = LinkGet(ConnHdl);\n"
		"\tif (pLink == nullptr)\n\t{\n\t\treturn NRF_ERROR_INVALID_STATE;\n\t}\n",
		"\tBtSecBmLink_t *pLink = LinkGet(ConnHdl);\n"
		"\tif (pLink == nullptr)\n\t{\n"
		"\t\t(void)pdb_write_buf_release(tempPeerId, PM_PEER_DATA_ID_BONDING);\n"
		"\t\treturn NRF_ERROR_INVALID_STATE;\n\t}\n",
		"sec release missing link")
	text = rep(text,
		"\tr = im_ble_addr_get(ConnHdl, &peerData.bonding_data->peer_ble_id.id_addr_info);\n"
		"\tif (r != NRF_SUCCESS)\n\t{\n\t\treturn NRF_ERROR_INVALID_STATE;\n\t}\n",
		"\tr = im_ble_addr_get(ConnHdl, &peerData.bonding_data->peer_ble_id.id_addr_info);\n"
		"\tif (r != NRF_SUCCESS)\n\t{\n"
		"\t\t(void)pdb_write_buf_release(tempPeerId, PM_PEER_DATA_ID_BONDING);\n"
		"\t\treturn NRF_ERROR_INVALID_STATE;\n\t}\n",
		"sec release address failure")

	text = rep(text, "\tuint32_t             r = NRF_SUCCESS;\n",
		"\tuint32_t             r = NRF_SUCCESS;\n\tbool                 bWriteBufHeld = false;\n",
		"sec write buffer state")
	text = rep(text,
		"\t\t\t\t\treturn r;\n\t\t\t\t}\n\t\t\t}\n\t\t}\n\t}\n\n"
		"\t// Peripheral replies with its parameters;",
		"\t\t\t\t\treturn r;\n\t\t\t\t}\n"
		"\t\t\t\tbWriteBufHeld = true;\n\t\t\t}\n\t\t}\n\t}\n\n"
		"\t// Peripheral replies with its parameters;",
		"sec mark write buffer")
	text = rep(text,
		"\tr = sd_ble_gap_sec_params_reply(ConnHdl, secStatus, pReplyParams, &keyset);\n"
		"\tpm_conn_state_user_flag_set(ConnHdl, s_FlagReplyPendBusy, r == NRF_ERROR_BUSY);\n\n",
		"\tr = sd_ble_gap_sec_params_reply(ConnHdl, secStatus, pReplyParams, &keyset);\n"
		"\tpm_conn_state_user_flag_set(ConnHdl, s_FlagReplyPendBusy, r == NRF_ERROR_BUSY);\n\n"
		"\tif (bWriteBufHeld && r != NRF_SUCCESS && r != NRF_ERROR_BUSY)\n\t{\n"
		"\t\tWriteBufRelease(ConnHdl);\n\t}\n\n",
		"sec release terminal reply")
	text = rep(text,
		"\t\tif (peerId == PM_PEER_ID_INVALID)\n\t\t{\n"
		"\t\t\tUnexpectedErrorSend(connHdl, NRF_ERROR_NO_MEM);\n",
		"\t\tif (peerId == PM_PEER_ID_INVALID)\n\t\t{\n"
		"\t\t\tWriteBufRelease(connHdl);\n"
		"\t\t\tUnexpectedErrorSend(connHdl, NRF_ERROR_NO_MEM);\n",
		"sec release allocation failure")

	text = sub(text,
		r"#if defined\(CONFIG_PM_LESC\)\n\tuint32_t nrfErr = nrf_ble_lesc_init\(\);\n"
		r"\tif \(nrfErr != NRF_SUCCESS\)\n\t\{\n\t\treturn nrfErr;\n\t\}\n#endif",
		"#if defined(CONFIG_PM_LESC)\n\tif (!BtLescInit())\n\t{\n"
		"\t\treturn NRF_ERROR_INTERNAL;\n\t}\n#endif", "sec LESC init")
	text = rep(text, "nrf_ble_lesc_on_ble_evt(ble_evt);",
		"BtLescOnBleEvt(ble_evt);", "sec LESC event")
	text = text.replace("nrf_ble_lesc_request_handler", "BtLescRequestHandler")
	text = text.replace("nrf_ble_lesc replacement", "BtLesc module")
	if "nrf_ble_lesc_" in text:
		raise RuntimeError("sec: stale nrf_ble_lesc API")
	path.write_text(text)


def patch_app() -> None:
	path = ROOT / "ARM/Nordic/nRF54/src/bt_app_bm.cpp"
	text = path.read_text()

	text = rep(text, '#include "bm/bluetooth/peer_manager/peer_manager_handler.h"\n',
		'#include "bm/bluetooth/peer_manager/peer_manager_handler.h"\n'
		'#include <modules/conn_state.h>\n', "app conn state include")
	text = rep(text, '#include "bm/bluetooth/peer_manager/nrf_ble_lesc.h"\n', "",
		"app SDK LESC include")
	text = rep(text, "\tif (nrf_ble_lesc_own_oob_data_generate() != NRF_SUCCESS)\n",
		"\tif (!BtLescOobLocalGen())\n", "app OOB generate")
	text = rep(text, "\tble_gap_lesc_oob_data_t *p = nrf_ble_lesc_own_oob_data_get();\n",
		"\tble_gap_lesc_oob_data_t *p = BtLescOobLocalGet();\n", "app OOB get")
	text = rep(text,
		"\t//uint8_t role = ble_conn_state_role(p_ble_evt->evt.gap_evt.conn_handle);\n"
		"\tuint8_t role = g_BtAppData.AppDevice.Conn.Role;\n",
		"\tuint8_t role = pm_conn_state_role(p_ble_evt->evt.gap_evt.conn_handle);\n",
		"app connection role")
	text = rep(text,
		"\tif ((role == BLE_GAP_ROLE_CENTRAL) ||\n"
		"\t\t(g_BtAppData.AppDevice.Conn.Role & (BTAPP_ROLE_CENTRAL | BTAPP_ROLE_OBSERVER)))\n",
		"\tif ((role == BLE_GAP_ROLE_CENTRAL) ||\n"
		"\t\t(role == BLE_GAP_ROLE_INVALID &&\n"
		"\t\t (g_BtAppData.AppDevice.Conn.Role & (BTAPP_ROLE_CENTRAL | BTAPP_ROLE_OBSERVER))))\n",
		"app central routing")
	text = rep(text, "\tif (g_BtAppData.AppDevice.Conn.Role & BTAPP_ROLE_PERIPHERAL)\n",
		"\tif ((role == BLE_GAP_ROLE_PERIPH) ||\n"
		"\t\t(role == BLE_GAP_ROLE_INVALID &&\n"
		"\t\t (g_BtAppData.AppDevice.Conn.Role & BTAPP_ROLE_PERIPHERAL)))\n",
		"app peripheral routing")

	text = rep(text,
		"\terr = sd_ble_enable(&ramreq);\n\n\tif (ramreq > ramstart)\n\t{\n"
		"\t\tDEBUG_PRINTF(\"%x - Insufficient RAM allocated for the SoftDevice need %x\", err, ramreq);\n"
		"\t}\n\n\t(void)sdh_state_evt_observer_notify(NRF_SDH_STATE_EVT_BLE_ENABLED);\n",
		"\terr = sd_ble_enable(&ramreq);\n\n\tif (err != NRF_SUCCESS)\n\t{\n"
		"\t\tif (ramreq > ramstart)\n\t\t{\n"
		"\t\t\tDEBUG_PRINTF(\"%x - Insufficient RAM allocated for the SoftDevice need %x\",\n"
		"\t\t\t\t\t\t err, ramreq);\n\t\t}\n\t\telse\n\t\t{\n"
		"\t\t\tDEBUG_PRINTF(\"sd_ble_enable failed: 0x%x\\r\\n\", err);\n"
		"\t\t}\n\t\treturn false;\n\t}\n\n"
		"\t(void)sdh_state_evt_observer_notify(NRF_SDH_STATE_EVT_BLE_ENABLED);\n",
		"app sd_ble_enable")
	text = rep(text,
		"\tstatic CryptoDev_t s_LescEcdh;\n"
		"\tstatic uint8_t     s_LescEcdhMem[CRYPTO_MEMSIZE_ECDH];\t// ECDH per-instance key arena (fits HW or uECC)\n",
		"\tstatic CryptoDev_t s_LescEcdh;\n"
		"\talignas(uint32_t) static uint8_t s_LescEcdhMem[CRYPTO_MEMSIZE_ECDH];\n",
		"app ECDH alignment")
	text = sub(text,
		r"\t// When CONFIG_PM_LESC is defined, pm_init -> sm_init already called\n.*?"
		r"\tnrf_ble_lesc_peer_oob_data_handler_set\(BtAppOobPeerDataHandler\);\n#endif\n",
		"\t// Route staged peer OOB data into the LESC pairing.\n"
		"\tBtLescOobPeerHandlerSet(BtAppOobPeerDataHandler);\n",
		"app OOB registration", re.DOTALL)

	text = text.replace("nrf_ble_lesc_request_handler", "BtLescRequestHandler")
	text = text.replace("nrf_ble_lesc", "BtLesc")
	text = text.replace("\tBtSmpTimeoutCheck();\n", "")
	text = text.replace("SMP/GATT transaction timeout checks", "GATT transaction timeout checks")
	if "nrf_ble_lesc" in text or "BtSmpTimeoutCheck" in text:
		raise RuntimeError("app: stale LESC or native SMP call")
	path.write_text(text)


def patch_project() -> None:
	path = ROOT / "ARM/Nordic/nRF54/nRF54L15/lib/Eclipse/.project"
	text = path.read_text()
	for name in (
		"src/sdk-nrf-bm/lib/bluetooth/peer_manager/modules/id_manager.c",
		"src/bluetooth/bt_lesc_pm_shim.cpp",
		"src/bluetooth/bt_smp.cpp",
		"src/bluetooth/bt_smp_bond.cpp",
	):
		pattern = (r"\t\t<link>\n\t\t\t<name>" + re.escape(name) +
			r"</name>\n.*?\t\t</link>\n")
		text = sub(text, pattern, "", f"project remove {name}", re.DOTALL)
	app = ("\t\t<link>\n\t\t\t<name>src/bluetooth/bt_app_bm.cpp</name>\n"
		"\t\t\t<type>1</type>\n"
		"\t\t\t<locationURI>PARENT-4-PROJECT_LOC/nRF54/src/bt_app_bm.cpp</locationURI>\n"
		"\t\t</link>\n")
	identity = ("\t\t<link>\n\t\t\t<name>src/bluetooth/bt_id_bm.cpp</name>\n"
		"\t\t\t<type>1</type>\n"
		"\t\t\t<locationURI>PARENT-4-PROJECT_LOC/nRF54/src/bt_id_bm.cpp</locationURI>\n"
		"\t\t</link>\n")
	text = rep(text, app, app + identity, "project identity link")
	path.write_text(text)


def validate() -> None:
	project_path = ROOT / "ARM/Nordic/nRF54/nRF54L15/lib/Eclipse/.project"
	cproject_path = ROOT / "ARM/Nordic/nRF54/nRF54L15/lib/Eclipse/.cproject"
	ET.parse(project_path)
	ET.parse(cproject_path)
	project = project_path.read_text()
	cproject = cproject_path.read_text()
	sec = (ROOT / "ARM/Nordic/nRF54/src/bt_sec_bm.cpp").read_text()
	app = (ROOT / "ARM/Nordic/nRF54/src/bt_app_bm.cpp").read_text()
	identity = (ROOT / "ARM/Nordic/nRF54/src/bt_id_bm.cpp").read_text()

	for item in ("src/bluetooth/bt_id_bm.cpp",
		"PARENT-4-PROJECT_LOC/nRF54/src/bt_id_bm.cpp"):
		if project.count(item) != 1:
			raise RuntimeError(f"missing or duplicate project item: {item}")
	for item in ("src/sdk-nrf-bm/lib/bluetooth/peer_manager/modules/id_manager.c",
		"src/bluetooth/bt_lesc_pm_shim.cpp", "src/bluetooth/bt_smp.cpp",
		"src/bluetooth/bt_smp_bond.cpp"):
		if item in project or item in cproject:
			raise RuntimeError(f"stale build reference: {item}")
	if "nrf_ble_lesc" in sec or "nrf_ble_lesc" in app:
		raise RuntimeError("stale nrf_ble_lesc reference")
	if "BtSmpTimeoutCheck" in app:
		raise RuntimeError("native SMP timeout path remains")
	for symbol in ("im_ble_evt_handler", "im_allow_list_set", "im_privacy_get"):
		if symbol not in identity:
			raise RuntimeError(f"identity API missing: {symbol}")
	for text in (project, cproject):
		if re.search(r"(?:/Users/|/home/|[A-Za-z]:\\\\)", text):
			raise RuntimeError("absolute path found in Eclipse metadata")

	(ROOT / ".github/workflows/nrf54-stage1.yml").unlink()
	Path(__file__).unlink()


def main() -> None:
	patch_sec()
	patch_app()
	patch_project()
	validate()


if __name__ == "__main__":
	main()
