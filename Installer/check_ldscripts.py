#!/usr/bin/env python3
"""Check IOsonata linker-script filenames and references.

Run from any directory:
    python3 Installer/check_ldscripts.py

No external Python packages required.

Some projects use linker scripts from vendor packages in external/, which the
installer places next to IOsonata (<IOCOMPOSER_HOME>/IOsonata and
<IOCOMPOSER_HOME>/external): nrfx provides nrf_common.ld, and nRF5 SDK and
STM32Cube projects link the package's own script. Those references are listed
separately. When external/ is present, next to IOsonata or under
$IOCOMPOSER_HOME, they are checked like any other reference. Otherwise they are
listed as not checked and do not count as findings. A reference through an
Eclipse linked folder (${workspace_loc:/${ProjName}/<link>}) depends on a
workspace path variable and is listed without a check.
"""
import json
import os
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
LINK_RE = re.compile(r'(?<![A-Za-z0-9_])([A-Za-z0-9_./\\-]+\.ld)(?![A-Za-z0-9_])')
INCLUDE_RE = re.compile(r'^\s*INCLUDE\s+["\']?([^"\'\s]+\.ld)', re.MULTILINE)
ICF_RE = re.compile(r'([A-Za-z0-9_./\\-]+\.icf)', re.IGNORECASE)
WIZARD_KEY = "linker_script"
LINKED_FOLDER = "${workspace_loc:"

# Linker scripts that vendor packages in external/ provide for INCLUDE, with
# the folders under external/ that hold them (nrfx 3 and nrfx 4 layouts).
EXTERNAL_INCLUDES = {
    "nrf_common.ld": ("nrfx/mdk", "nrfx/bsp/stable/mdk", "nrfx/bsp/stable/mdk/common"),
}


def all_scripts(root=ROOT):
    return sorted(root.rglob("*.ld"))


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


def find_external_root(root=ROOT):
    """external/ of the installer layout, or None when it is not present."""
    bases = []
    home = os.environ.get("IOCOMPOSER_HOME")
    if home:
        bases.append(pathlib.Path(home))
    bases.append(root.parent)
    for base in bases:
        candidate = base / "external"
        if candidate.is_dir():
            return candidate
    return None


def external_subpath(ref):
    """The part of a reference below external/, or None."""
    text = ref.replace("\\", "/")
    if text.startswith("${external}/"):
        return text[len("${external}/"):]
    if text.startswith("external/"):
        return text[len("external/"):]
    pos = text.rfind("/external/")
    if pos >= 0:
        return text[pos + len("/external/"):]
    return None


def cproject_refs(text):
    """Linker file references in a .cproject, each with the text before it.

    The prefix is what precedes the path inside the same option value, such
    as ${workspace_loc:/${ProjName}/CubeRoot}, which LINK_RE does not match.
    """
    for match in LINK_RE.finditer(text):
        head = text[max(0, match.start() - 300):match.start()]
        cut = max(head.rfind('"'), head.rfind(';'), head.rfind('='))
        yield match.group(1), head[cut + 1:]


def external_kind(ref, prefix="", include=False):
    """How a reference not found in the repository resolves, or None.

    Returns ("include", name), ("path", subpath below external/) or
    ("linked", reference), or None when the reference does not point into a
    vendor package.
    """
    name = pathlib.PurePosixPath(ref.replace("\\", "/")).name
    if include:
        return ("include", name) if name in EXTERNAL_INCLUDES else None
    sub = external_subpath(prefix + ref)
    if sub:
        return ("path", sub)
    if LINKED_FOLDER in prefix:
        return ("linked", prefix + ref)
    return None


def check_external(kind, target, external):
    """'ok', 'missing', or the reason it cannot be checked."""
    if kind == "linked":
        return "not checked, Eclipse linked folder"
    if external is None:
        return "not checked, no external/ folder"
    if kind == "include":
        found = any((external / folder / target).is_file()
                    for folder in EXTERNAL_INCLUDES[target])
    else:
        found = (external / target).is_file()
    return "ok" if found else "missing"


def check(root=ROOT, external="auto"):
    if external == "auto":
        external = find_external_root(root)
    scripts = all_scripts(root)
    index = script_index(scripts)
    failures = []
    externals = []

    def note_external(source, label, ref, kind):
        status = check_external(kind[0], kind[1], external)
        line = "%s %s %s" % (source.relative_to(root), label, ref)
        if status == "missing":
            failures.append("%s: missing in %s" % (line, external))
        externals.append((line, status))

    for name, matches in sorted(index.items()):
        if len(matches) > 1:
            failures.append("DUPLICATE: %s -> %s" % (
                name, ", ".join(str(p.relative_to(root)) for p in matches)))

    for script in scripts:
        text = script.read_text(encoding="utf-8", errors="replace")
        for name in INCLUDE_RE.findall(text):
            issue = find_reference(script, name, scripts, index)
            kind = external_kind(name, include=True) if issue == "missing" else None
            if kind:
                note_external(script, "INCLUDE", name, kind)
            elif issue:
                failures.append("%s INCLUDE %s: %s" %
                    (script.relative_to(root), name, issue))

    for wizard in root.rglob("wizard.json"):
        try:
            data = json.loads(wizard.read_text(encoding="utf-8"))
        except (OSError, ValueError) as exc:
            failures.append("%s: %s" % (wizard.relative_to(root), exc))
            continue
        for ref in refs_from_json(data):
            if not ref.endswith(".ld"):
                continue
            issue = find_reference(wizard, ref, scripts, index)
            kind = external_kind(ref) if issue == "missing" else None
            if kind:
                note_external(wizard, "linker_script", ref, kind)
            elif issue:
                failures.append("%s linker_script %s: %s" %
                    (wizard.relative_to(root), ref, issue))

    for project in root.rglob(".cproject"):
        text = project.read_text(encoding="utf-8", errors="replace")
        for name, prefix in sorted(set(cproject_refs(text))):
            # For .cproject, check linker file references only; support
            # template-style names that deliberately resolve externally.
            issue = find_reference(project, name, scripts, index)
            kind = external_kind(name, prefix) if issue == "missing" else None
            if kind:
                note_external(project, "linker", prefix + name, kind)
            elif issue:
                failures.append("%s linker %s: %s" %
                    (project.relative_to(root), name, issue))

    # Validate repository IAR linker script references in project files.
    icf_index = script_index(root.rglob("*.icf"))
    for project in root.rglob("*.ewp"):
        content = project.read_text(encoding="utf-8", errors="replace")
        for ref in sorted(set(ICF_RE.findall(content))):
            name = pathlib.PurePosixPath(ref.replace("\\", "/")).name
            if name in icf_index:
                if len(icf_index[name]) > 1:
                    failures.append("%s IAR linker %s: ambiguous" %
                        (project.relative_to(root), ref))
            elif "/" in ref or "\\" in ref:
                failures.append("%s IAR linker %s: missing" %
                    (project.relative_to(root), ref))

    for issue in failures:
        print(issue)
    for line, status in externals:
        if status != "missing":
            print("external (%s): %s" % (status, line))
    print("Linker check: %d scripts, %d findings, %d external references%s" %
        (len(scripts), len(failures), len(externals),
         "" if not externals else
         (" (checked in %s)" % external if external else
          " (not checked, no external/ folder)")))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(check())
