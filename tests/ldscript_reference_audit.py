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
import re
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



def check_iar_project(path):
    """Validate IlinkIcfFile values relative to the IAR project directory."""
    errors = []
    checked = 0
    skipped = 0
    try:
        xml = ET.parse(path)
    except ET.ParseError as exc:
        return 0, 0, [f"{path}: invalid XML: {exc}"]

    for option in xml.findall(".//option"):
        name = option.findtext("name", default="")
        if name not in ("IlinkIcfFile", "IlinkIcfFileSlave"):
            continue
        for state in option.findall("state"):
            reference = (state.text or "").strip().strip('"')
            if not reference:
                continue
            if "$" in reference or "%" in reference:
                skipped += 1
                continue
            if Path(reference).is_absolute() or PureWindowsPath(reference).is_absolute():
                checked += 1
                errors.append(f"{path}: nonportable absolute IAR linker script: {reference}")
                continue
            normalized = reference.replace(chr(92), "/")
            if Path(normalized).suffix.lower() != ".icf":
                skipped += 1
                continue
            checked += 1
            target = (path.parent / normalized).resolve()
            if not target.is_file():
                errors.append(f"{path}: missing IAR linker script {reference} (resolved to {target})")
    return checked, skipped, errors


def referenced_scripts(root):
    """Collect literal script paths from managed projects and IAR projects."""
    referenced = set()
    for project in root.rglob(".cproject"):
        try:
            xml = ET.parse(project)
        except ET.ParseError:
            continue
        for config in xml.findall(".//cconfiguration"):
            settings = config.find(".//configuration")
            name = settings.get("name") if settings is not None else None
            if not name:
                continue
            for raw in script_values(config):
                value = raw.strip().strip('"').strip("'")
                if "$" in value or "%" in value or Path(value).is_absolute():
                    continue
                if Path(value).suffix.lower() in (".ld", ".icf", ".sct"):
                    referenced.add((project.parent / name / value).resolve())
    for project in root.rglob("*.ewp"):
        try:
            xml = ET.parse(project)
        except ET.ParseError:
            continue
        for option in xml.findall(".//option"):
            if option.findtext("name", default="") not in ("IlinkIcfFile", "IlinkIcfFileSlave"):
                continue
            for state in option.findall("state"):
                value = (state.text or "").strip().strip('"')
                if not value or "$" in value or "%" in value:
                    continue
                if Path(value).is_absolute() or PureWindowsPath(value).is_absolute():
                    continue
                normalized = value.replace(chr(92), "/")
                if Path(normalized).suffix.lower() == ".icf":
                    referenced.add((project.parent / normalized).resolve())
    return referenced


def included_scripts(scripts):
    """Find literal GNU LD INCLUDE and IAR ICF include dependencies."""
    dependencies = set()
    pattern = re.compile(
        r'^\\s*(?:INCLUDE\\s+|include\\s+)(?:"([^"]+)"|<([^>]+)>|([^\\s;/]+))',
        re.MULTILINE,
    )
    for script in scripts:
        if script.suffix.lower() not in (".ld", ".icf"):
            continue
        try:
            content = script.read_text(encoding="utf-8")
        except (OSError, UnicodeError):
            continue
        for match in pattern.finditer(content):
            name = next(group for group in match.groups() if group is not None)
            if "$" in name or "%" in name:
                continue
            dependency = (script.parent / name.replace(chr(92), "/")).resolve()
            if dependency.is_file():
                dependencies.add(dependency)
    return dependencies


def report_unreferenced(root):
    """Report candidates only; external references may still exist."""
    referenced = referenced_scripts(root)
    scripts = sorted(
        path.resolve()
        for path in root.rglob("*")
        if path.is_file()
        and path.suffix.lower() in (".ld", ".icf", ".sct")
        and "ldscript" in path.parts
    )
    included = included_scripts(scripts)
    candidates = [path for path in scripts if path not in referenced and path not in included]
    for path in candidates:
        print(f"CANDIDATE (not directly selected by .cproject/.ewp): {path}")
    print(f"Included dependencies: {len(included)}; unreferenced candidates: {len(candidates)}; no files deleted")

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "directory", nargs="?", default=".",
        help="Repository directory or MCU-family subtree to audit",
    )
    parser.add_argument(
        "--report-unreferenced", action="store_true",
        help="Report linker scripts not directly selected by Eclipse or IAR projects; never delete",
    )
    args = parser.parse_args()
    root = Path(args.directory).resolve()
    if not root.is_dir():
        parser.error(f"not a directory: {root}")

    projects = sorted(root.rglob(".cproject"))
    iar_projects = sorted(root.rglob("*.ewp"))
    if not projects and not iar_projects:
        print(f"No Eclipse or IAR project files under {root}", file=sys.stderr)
        return 2

    checked = 0
    skipped = 0
    problems = []
    for path in projects:
        count, ignored, errors = check_project(path)
        checked += count
        skipped += ignored
        problems.extend(errors)

    for path in iar_projects:
        count, ignored, errors = check_iar_project(path)
        checked += count
        skipped += ignored
        problems.extend(errors)

    for problem in problems:
        print(f"FAIL: {problem}", file=sys.stderr)
    print(
        f"Eclipse projects: {len(projects)}; IAR projects: {len(iar_projects)}; "
        f"literal linker references: {checked}; "
        f"dynamic/external references skipped: {skipped}; "
        f"errors: {len(problems)}"
    )
    if args.report_unreferenced:
        report_unreferenced(root)
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
