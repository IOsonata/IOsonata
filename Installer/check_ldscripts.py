#!/usr/bin/env python3
"""Check IOsonata linker-script filenames and references.

Run from any directory:
    python3 Installer/check_ldscripts.py

No external Python packages required.
"""
import json
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
LINK_RE = re.compile(r'(?<![A-Za-z0-9_])([A-Za-z0-9_./\\-]+\.ld)(?![A-Za-z0-9_])')
INCLUDE_RE = re.compile(r'^\s*INCLUDE\s+["\']?([^"\'\s]+\.ld)', re.MULTILINE)
WIZARD_KEY = "linker_script"


def all_scripts():
    return sorted(ROOT.rglob("*.ld"))


def script_index(scripts):
    result = {}
    for script in scripts:
        result.setdefault(script.name, []).append(script)
    return result


def refs_from_json(node):
    if isinstance(node, dict):
        for key, value in node.items():
            if key == WIZARD_KEY and isinstance(value, str):
                yield value
            else:
                yield from refs_from_json(value)
    elif isinstance(node, list):
        for item in node:
            yield from refs_from_json(item)


def find_reference(filename, ref, scripts, index):
    # Do not silently accept ambiguous basenames: GNU ld's SEARCH_DIR
    # order can choose a different memory map.
    name = pathlib.PurePosixPath(ref).name
    matches = index.get(name, [])
    if not matches:
        return "missing"
    if len(matches) > 1:
        return "ambiguous"
    return None


def check():
    scripts = all_scripts()
    index = script_index(scripts)
    failures = []
    for name, matches in sorted(index.items()):
        if len(matches) > 1:
            failures.append("DUPLICATE: %s -> %s" % (
                name, ", ".join(str(p.relative_to(ROOT)) for p in matches)))

    for script in scripts:
        text = script.read_text(encoding="utf-8", errors="replace")
        for name in INCLUDE_RE.findall(text):
            issue = find_reference(script, name, scripts, index)
            if issue:
                failures.append("%s INCLUDE %s: %s" %
                    (script.relative_to(ROOT), name, issue))

    for wizard in ROOT.rglob("wizard.json"):
        try:
            data = json.loads(wizard.read_text(encoding="utf-8"))
        except (OSError, ValueError) as exc:
            failures.append("%s: %s" % (wizard.relative_to(ROOT), exc))
            continue
        for ref in refs_from_json(data):
            if not ref.endswith(".ld"):
                continue
            issue = find_reference(wizard, ref, scripts, index)
            if issue:
                failures.append("%s linker_script %s: %s" %
                    (wizard.relative_to(ROOT), ref, issue))

    for project in ROOT.rglob(".cproject"):
        text = project.read_text(encoding="utf-8", errors="replace")
        for name in sorted(set(LINK_RE.findall(text))):
            # For .cproject, check linker file references only; support
            # template-style names that deliberately resolve externally.
            issue = find_reference(project, name, scripts, index)
            if issue:
                failures.append("%s linker %s: %s" %
                    (project.relative_to(ROOT), name, issue))

    for issue in failures:
        print(issue)
    print("Linker check: %d scripts, %d findings" %
        (len(scripts), len(failures)))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(check())
