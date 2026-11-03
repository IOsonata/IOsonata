#!/usr/bin/env python3
"""Validate literal linker-script references in Eclipse managed projects.

Run from the repository root:
    python3 tests/ldscript_reference_audit.py
    python3 tests/ldscript_reference_audit.py ARM/ST/STM32WBAxx

Relative -T paths are interpreted from ioc/<configuration>, where the
generated makefile runs, rather than from the .cproject file itself.
No compiler, SDK or Eclipse installation is required.
"""

import argparse
from pathlib import PureWindowsPath
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


def script_values(config):
    for option in config.iter("option"):
        identifier = option.get("superClass", "") + " " + option.get("id", "")
        if "linker.scriptfile" not in identifier:
            continue
        value = option.get("value")
        if value:
            yield value
        for item in option.iter("listOptionValue"):
            value = item.get("value")
            if value:
                yield value


def check_project(path):
    errors = []
    checked = 0
    skipped = 0
    try:
        xml = ET.parse(path)
    except ET.ParseError as exc:
        return 0, 0, [f"{path}: invalid XML: {exc}"]

    configs = xml.findall(".//cconfiguration")
    for config in configs:
        settings = config.find(".//configuration")
        name = settings.get("name") if settings is not None else None
        if not name:
            errors.append(f"{path}: configuration without build-directory name")
            continue

        workdir = path.parent / name
        for raw in script_values(config):
            reference = raw.strip().strip('"').strip("'")
            if not reference:
                continue
            if "$" in reference or "%" in reference:
                skipped += 1
                continue
            script = Path(reference)
            if script.is_absolute() or PureWindowsPath(reference).is_absolute():
                checked += 1
                errors.append(f"{path} [{name}]: nonportable absolute linker script: {reference}")
                continue
            if script.suffix.lower() not in (".ld", ".icf", ".sct"):
                skipped += 1
                continue
            checked += 1
            target = (workdir / script).resolve()
            if not target.is_file():
                errors.append(
                    f"{path} [{name}]: missing {reference} "
                    f"(resolved to {target})"
                )
    return checked, skipped, errors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "directory", nargs="?", default=".",
        help="Repository directory or MCU-family subtree to audit",
    )
    args = parser.parse_args()
    root = Path(args.directory).resolve()
    if not root.is_dir():
        parser.error(f"not a directory: {root}")

    projects = sorted(root.rglob(".cproject"))
    if not projects:
        print(f"No .cproject files under {root}", file=sys.stderr)
        return 2

    checked = 0
    skipped = 0
    problems = []
    for path in projects:
        count, ignored, errors = check_project(path)
        checked += count
        skipped += ignored
        problems.extend(errors)

    for problem in problems:
        print(f"FAIL: {problem}", file=sys.stderr)
    print(
        f"Projects: {len(projects)}; literal linker references: {checked}; "
        f"dynamic/external references skipped: {skipped}; "
        f"errors: {len(problems)}"
    )
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
