#!/usr/bin/env python3
"""Build Nordic Power/Clock Eclipse managed projects in isolated workspaces.

Uses IOcomposer/Eclipse CDT headless builder and the checked-in .project and
.cproject metadata. This is not a replacement for the managed projects.
"""

import argparse
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[2]
TARGETS = ("nrf52832", "nrf52840")
PROJECT_ROOT = {
    "nrf52832": ROOT / "ARM/Nordic/nRF52/nRF52832",
    "nrf52840": ROOT / "ARM/Nordic/nRF52/nRF52840",
}
DEFAULT_ECLIPSE = Path("/Applications/IOcomposer.app/Contents/MacOS/eclipse")


def locate_eclipse(explicit):
    if explicit:
        candidate = Path(explicit).expanduser()
        if candidate.is_file():
            return candidate
        raise FileNotFoundError("Eclipse executable not found: " + str(candidate))

    configured = os.environ.get("IOCOMPOSER_ECLIPSE")
    if configured:
        return locate_eclipse(configured)

    if DEFAULT_ECLIPSE.is_file():
        return DEFAULT_ECLIPSE

    candidate = shutil.which("eclipse")
    if candidate:
        return Path(candidate)

    raise FileNotFoundError(
        "IOcomposer Eclipse not found; set --eclipse or IOCOMPOSER_ECLIPSE")


def project_name(project_dir):
    metadata = (project_dir / ".project").read_text(encoding="utf-8")
    match = re.search(r"<projectDescription>\s*<name>([^<]+)</name>", metadata)
    if not match:
        raise ValueError("Missing Eclipse project name in " + str(project_dir))
    return match.group(1)


def build(eclipse, workspace, directory, configuration):
    name = project_name(directory)
    args = [
        str(eclipse),
        "--launcher.suppressErrors",
        "-nosplash",
        "-application",
        "org.eclipse.cdt.managedbuilder.core.headlessbuild",
        "-data",
        str(workspace),
        "-no-indexer",
        "-import",
        str(directory),
        "-cleanBuild",
        name + "/" + configuration,
        "-printErrorMarkers",
    ]
    print("RUN:", " ".join(args), flush=True)
    result = subprocess.run(args, check=False)
    if result.returncode != 0:
        raise RuntimeError(
            "Managed build failed: " + name + "/" + configuration +
            ", return code " + str(result.returncode))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target", action="append", choices=TARGETS,
                        help="Build this Nordic target (repeatable)")
    parser.add_argument("--config", action="append",
                        choices=("Debug", "Release"),
                        help="Managed build configuration (default: Debug)")
    parser.add_argument("--eclipse", help="Path to IOcomposer Eclipse executable")
    args = parser.parse_args()

    targets = args.target or list(TARGETS)
    configurations = args.config or ["Debug"]
    eclipse = locate_eclipse(args.eclipse)

    for target in targets:
        root = PROJECT_ROOT[target]
        projects = [
            root / "lib/ioc",
            root / "exemples/ClockDemo/ioc",
            root / "exemples/PowerOffDemo/ioc",
        ]
        for directory in projects:
            if not (directory / ".project").is_file():
                raise FileNotFoundError("Missing managed project: " + str(directory))

        # ClockDemo has the same Eclipse project name on different targets.
        # Never import two targets into the same workspace.
        with tempfile.TemporaryDirectory(
                prefix="iosonata-nordic-cdt-" + target + "-") as work:
            workspace = Path(work)
            for configuration in configurations:
                for directory in projects:
                    build(eclipse, workspace, directory, configuration)
                print(target, configuration, "PASS", flush=True)

    print("All requested Nordic managed builds PASS", flush=True)


if __name__ == "__main__":
    try:
        main()
    except (FileNotFoundError, RuntimeError, ValueError) as exc:
        print("ERROR:", exc, file=sys.stderr)
        sys.exit(1)
