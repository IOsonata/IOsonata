#!/usr/bin/env bash
# IOcomposer website launcher. Installation is owned by IOsonata/Installer.
set -euo pipefail

ref="${IOSONATA_INSTALLER_REF:-main}"
url="https://raw.githubusercontent.com/IOsonata/IOsonata/$ref/Installer/install_iocdevtools_macos.sh"
stage=$(mktemp -d)
trap 'rm -rf "$stage"' EXIT
curl -fsSL "$url" -o "$stage/install.sh"

# Help/version work without a terminal. For piped installs, use the controlling
# terminal when one exists; otherwise preserve the caller's input stream.
noninteractive=0
for arg in "$@"; do
	case "$arg" in --help|--version) noninteractive=1 ;; esac
done
if [[ "$noninteractive" == "0" && ! -t 0 ]] && ( : </dev/tty ) 2>/dev/null; then
	bash "$stage/install.sh" "$@" </dev/tty
else
	bash "$stage/install.sh" "$@"
fi
