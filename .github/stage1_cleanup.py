#!/usr/bin/env python3
from __future__ import annotations

import re
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def rep(text: str, old: str, new: str, label: str, expected: int = 1) -> str:
	count = text.count(old)
	if count != expected:
		raise RuntimeError(f"{label}: expected {expected} matches, found {count}")
	return text.replace(old, new)


def patch_app() -> None:
	path = ROOT / "ARM/Nordic/nRF54/src/bt_app_bm.cpp"
	text = path.read_text()

	text = rep(text,
		"// Bare-metal fallback polling interval while connected. This guarantees generic\n"
		"// SMP/GATT transaction timeouts advance even when the peer goes fully silent.\n"
		"// RTOS ports should override BtAppEvtWait with a blocking primitive plus timer.\n",
		"// Bare-metal fallback polling interval while connected. This guarantees GATT\n"
		"// transaction timeouts advance even when the peer goes fully silent. RTOS\n"
		"// targets should override BtAppEvtWait with a blocking primitive plus timer.\n",
		"app timeout header")
	text = rep(text, "// On this port BtLesc owns the SC key pair, so the local OOB set comes\n",
		"// On this target BtLesc owns the SC key pair, so the local OOB set comes\n",
		"app OOB target wording")
	text = rep(text, "the nRF52\n// SoftDevice port.", "the nRF52\n// SoftDevice implementation.",
		"app nRF52 wording")
	text = rep(text, "the nRF52 port does", "the nRF52 implementation does",
		"app nRF52 implementation wording", expected=1)
	text = rep(text, "pm_init -> sm_init calls BtLesc_init", "pm_init -> sm_init calls BtLescInit",
		"app LESC name 1")
	text = rep(text, "BtLesc_init fails", "BtLescInit fails", "app LESC name 2")
	text = rep(text, "pm_peers_delete / BtSmpBondClearAll", "pm_peers_delete",
		"app native bond API comment")

	pattern = re.compile(
		r"// Millisecond clock for the generic SMP/GATT transaction timeouts, overriding\n"
		r"// the weak BtSmpMsTick/BtGattMsTick defaults\. The RTC/timer is owned by the\n"
		r"// SDK \+ SoftDevice; this reads the free-running GRTC3 count via the s_BtAppSdGrtc3\n"
		r"// handle the port enables as a SoftDevice prerequisite \(a read is\n"
		r"// non-destructive\)\. Declared in bt_smp\.h / bt_gatt\.h, so no linkage specifier is\n"
		r"// needed here\.\n"
		r"uint32_t BtSmpMsTick\(void\)\n"
		r"\{\n\treturn s_BtAppSdGrtc3\.mSecond\(\);\n\}\n\n"
		r"uint32_t BtGattMsTick\(void\)"
	)
	repl = (
		"// Millisecond clock for GATT transaction timeouts. The SDK and SoftDevice\n"
		"// own the timer; this reads the free-running GRTC3 count through the target\n"
		"// timer object.\n"
		"uint32_t BtGattMsTick(void)"
	)
	text, count = pattern.subn(repl, text, count=1)
	if count != 1:
		raise RuntimeError(f"app native SMP tick removal: expected 1 match, found {count}")

	text = rep(text,
		"\t\t// Drive the generic transaction timeouts (Core Vol 3 Part H 3.4, Part F\n"
		"\t\t// 3.3.3). Cheap no-ops when nothing is pending. NOTE: this loop wakes on\n"
		"\t\t// events, so a link that goes fully silent needs a periodic wake to also\n"
		"\t\t// call these - hook them into an existing SDK/SoftDevice periodic callback\n"
		"\t\t// (do not add a trigger to GRTC3, which the SoftDevice owns).\n"
		"\t\t\tBtGattIndicationTimeoutCheck();\n",
		"\t\t// Drive the GATT indication transaction timeout. The continuous trigger\n"
		"\t\t// configured after SoftDevice enable supplies the periodic wake when the\n"
		"\t\t// peer is otherwise silent.\n"
		"\t\tBtGattIndicationTimeoutCheck();\n",
		"app GATT timeout loop")
	text = rep(text, "// Port-level weak default for BtAppEvtWait.",
		"// Target-level weak default for BtAppEvtWait.", "app target wording")

	if "BtSmpMsTick" in text or "BtSmpTimeoutCheck" in text or "BtSmpBondClearAll" in text:
		raise RuntimeError("app native SMP remnant remains")
	if "BtLesc_init" in text:
		raise RuntimeError("app incorrect BtLescInit spelling remains")
	path.write_text(text)


def patch_sec() -> None:
	path = ROOT / "ARM/Nordic/nRF54/src/bt_sec_bm.cpp"
	text = path.read_text()
	text = text.replace("the nRF52\n\t\tport bt_sec_sd.cpp", "the nRF52\n\t\timplementation bt_sec_sd.cpp")
	text = text.replace("from the nRF52 port", "from the nRF52 implementation")
	text = text.replace("matching the nRF52 port", "matching the nRF52 implementation")
	if re.search(r"\bnRF52 port\b", text):
		raise RuntimeError("sec target still described as port")
	path.write_text(text)


def patch_cproject() -> None:
	path = ROOT / "ARM/Nordic/nRF54/nRF54L15/lib/Eclipse/.cproject"
	text = path.read_text()
	entry = "src/sdk-nrf-bm/lib/bluetooth/peer_manager/modules/nrf_ble_lesc.c|"
	count = text.count(entry)
	if count != 2:
		raise RuntimeError(f"cproject stale LESC exclusion: expected 2, found {count}")
	text = text.replace(entry, "")
	path.write_text(text)


def remove_stale_sources() -> None:
	for rel in (
		"ARM/Nordic/src/bt_lesc_pm_shim.cpp",
		"ARM/Nordic/nRF54/src/nrf_ble_lesc.c",
	):
		path = ROOT / rel
		if not path.exists():
			raise RuntimeError(f"stale source missing before removal: {rel}")
		path.unlink()


def validate() -> None:
	project_path = ROOT / "ARM/Nordic/nRF54/nRF54L15/lib/Eclipse/.project"
	cproject_path = ROOT / "ARM/Nordic/nRF54/nRF54L15/lib/Eclipse/.cproject"
	ET.parse(project_path)
	ET.parse(cproject_path)

	project = project_path.read_text()
	cproject = cproject_path.read_text()
	app = (ROOT / "ARM/Nordic/nRF54/src/bt_app_bm.cpp").read_text()
	sec = (ROOT / "ARM/Nordic/nRF54/src/bt_sec_bm.cpp").read_text()
	identity = (ROOT / "ARM/Nordic/nRF54/src/bt_id_bm.cpp").read_text()

	for stale in (
		"src/sdk-nrf-bm/lib/bluetooth/peer_manager/modules/id_manager.c",
		"src/sdk-nrf-bm/lib/bluetooth/peer_manager/modules/nrf_ble_lesc.c",
		"src/bluetooth/bt_lesc_pm_shim.cpp",
		"src/bluetooth/bt_smp.cpp",
		"src/bluetooth/bt_smp_bond.cpp",
	):
		if stale in project or stale in cproject:
			raise RuntimeError(f"stale Eclipse reference: {stale}")

	for path in (
		ROOT / "ARM/Nordic/src/bt_lesc_pm_shim.cpp",
		ROOT / "ARM/Nordic/nRF54/src/nrf_ble_lesc.c",
	):
		if path.exists():
			raise RuntimeError(f"stale source remains: {path}")

	if "nrf_ble_lesc" in app or "nrf_ble_lesc" in sec:
		raise RuntimeError("stale LESC API remains in active sources")
	if "BtSmpMsTick" in app or "BtSmpTimeoutCheck" in app:
		raise RuntimeError("native SMP runtime remains in app")
	for symbol in ("im_ble_evt_handler", "im_allow_list_set", "im_privacy_get"):
		if symbol not in identity:
			raise RuntimeError(f"identity implementation missing {symbol}")
	for text in (project, cproject):
		if re.search(r"(?:/Users/|/home/|[A-Za-z]:\\\\)", text):
			raise RuntimeError("absolute path found in Eclipse metadata")

	(ROOT / ".github/workflows/nrf54-stage1-cleanup.yml").unlink()
	Path(__file__).unlink()


def main() -> None:
	patch_app()
	patch_sec()
	patch_cproject()
	remove_stale_sources()
	validate()


if __name__ == "__main__":
	main()
