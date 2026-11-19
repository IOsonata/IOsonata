#!/usr/bin/env python3
"""Clean-build existing Nordic Eclipse CDT libraries and power/clock demos.

Each target receives its own temporary Eclipse workspace because the demo
projects have identical names across MCU families. The project files and
managed Debug/Release configurations are used unchanged. This does not
create Makefiles or substitute a command-line linker for Eclipse CDT.
"""
import argparse
from pathlib import Path
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]

TARGETS = {
	"nrf52832": "ARM/Nordic/nRF52/nRF52832",
	"nrf52840": "ARM/Nordic/nRF52/nRF52840",
	"nrf5340_app": "ARM/Nordic/nRF53/nRF5340_App",
	"nrf5340_net": "ARM/Nordic/nRF53/nRF5340_Net",
	"nrf54l15": "ARM/Nordic/nRF54/nRF54L15",
	"nrf9160": "ARM/Nordic/nRF91/nRF9160",
	"nrf91x1": "ARM/Nordic/nRF91/nRF91x1",
}
DEMOS = ("ClockDemo", "PowerOffDemo")
CONFIGS = ("Debug", "Release")


def project_name(path):
	pfile = path / ".project"
	cfile = path / ".cproject"
	if not pfile.is_file() or not cfile.is_file():
		return None
	root = ET.parse(pfile).getroot()
	name = root.findtext("name")
	if not name:
		raise ValueError("Missing Eclipse project name: " + str(pfile))
	return name


def project_configs(path):
	root = ET.parse(path / ".cproject").getroot()
	names = set()
	for config in root.iter("cconfiguration"):
		settings = config.find("storageModule")
		if settings is not None and settings.get("name"):
			names.add(settings.get("name"))
	return names


def find_ide(explicit):
	if explicit:
		ide = Path(explicit).expanduser().resolve()
		if not ide.is_file():
			raise FileNotFoundError("IDE executable not found: " + str(ide))
		return ide
	for app in ("IOcomposer", "Eclipse"):
		for launcher in ("iocomposerc", "iocomposer", "eclipsec", "eclipse"):
			path = Path("/Applications") / (app + ".app") / "Contents/MacOS" / launcher
			if path.is_file():
				return path
	raise FileNotFoundError("No Eclipse CDT launcher found; use --ide PATH")


def build(ide, workspace, logfile, args):
	cmd = [str(ide), "--launcher.suppressErrors", "-nosplash",
		"-application", "org.eclipse.cdt.managedbuilder.core.headlessbuild",
		"-data", str(workspace), "-no-indexer"] + args + ["-printErrorMarkers"]
	print("RUN:", " ".join(cmd), flush=True)
	with logfile.open("a", encoding="utf-8") as output:
		output.write("\nRUN: " + " ".join(cmd) + "\n")
		output.flush()
		result = subprocess.run(cmd, cwd=ROOT, stdout=output,
			stderr=subprocess.STDOUT, check=False)
	if result.returncode:
		print("FAIL:", logfile, "(exit", result.returncode, ")", flush=True)
	return result.returncode


def main():
	parser = argparse.ArgumentParser(description=__doc__)
	parser.add_argument("--ide", help="IOcomposer or Eclipse CDT executable")
	parser.add_argument("--target", action="append", choices=sorted(TARGETS),
		help="MCU target (repeatable); default: nrf52832, nrf52840")
	parser.add_argument("--logs", type=Path, default=ROOT / "tests/nordic/build_logs",
		help="directory for durable per-project logs")
	parser.add_argument("--library-only", action="store_true",
		help="build managed library only, not ClockDemo or PowerOffDemo")
	parser.add_argument("--check-only", action="store_true",
		help="check project and Debug/Release configuration discovery without launching IDE")
	args = parser.parse_args()
	ide = None
	if not args.check_only:
		try:
			ide = find_ide(args.ide)
		except (FileNotFoundError, ValueError) as error:
			parser.error(str(error))
	logdir = args.logs.expanduser().resolve()
	if not args.check_only:
		logdir.mkdir(parents=True, exist_ok=True)
	targets = args.target or ("nrf52832", "nrf52840")
	failures = 0
	skipped = 0
	passed = 0

	for target in targets:
		base = ROOT / TARGETS[target]
		paths = [base / "lib/ioc"]
		if not args.library_only:
			paths.extend(base / ("exemples/" + demo + "/ioc") for demo in DEMOS)
		projects = []
		for path in paths:
			name = project_name(path)
			if name is None:
				print("SKIP missing managed project:", path.relative_to(ROOT))
				skipped += 1
				continue
			configs = project_configs(path)
			for config in CONFIGS:
				if config not in configs:
					print("SKIP missing configuration:", target, name, config)
					skipped += 1
				else:
					projects.append((path, name, config))
		if not projects:
			print("FAIL no managed projects:", target)
			failures += 1
			continue
		if args.check_only:
			for path, name, config in projects:
				print("FOUND:", target, name, config, path.relative_to(ROOT))
			passed += len(projects)
			continue
		unique = list(dict.fromkeys(path for path, _, _ in projects))
		with tempfile.TemporaryDirectory(prefix="iosonata-nordic-cdt-") as temporary:
			ws = Path(temporary)
			# Import in one operation; names ClockDemo and PowerOffDemo
			# cannot conflict because every target has an isolated workspace.
			first = True
			for path, name, config in projects:
				logfile = logdir / (target + "_" + name + "_" + config + ".log")
				logfile.write_text("", encoding="utf-8")
				command = []
				if first:
					for project in unique:
						command.extend(["-import", str(project)])
					first = False
				command.extend(["-cleanBuild", name + "/" + config])
				status = build(ide, ws, logfile, command)
				if status:
					failures += 1
				else:
					passed += 1
				print(target, name, config, "PASS" if status == 0 else "FAIL")
	print("Nordic managed CDT", "project discovery:" if args.check_only else "builds:", passed, "PASS,",
		failures, "FAIL,", skipped, "SKIPPED")
	if not args.check_only:
		print("Logs:", logdir)
	return 1 if failures else 0


if __name__ == "__main__":
	sys.exit(main())
