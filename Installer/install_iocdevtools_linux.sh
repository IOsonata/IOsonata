#!/usr/bin/env bash
set -euo pipefail

SCRIPT_NAME="install_iocdevtools_linux"
SCRIPT_VERSION="v1.0.87"

ROOT="$HOME/IOcomposer"
TOOLS="/opt/xPacks"
BIN="/usr/local/bin"
ECLIPSE_DIR="/opt/eclipse"

echo "=============================================="
echo "   IOcomposer MCU Dev Tools Installer (Linux) "
echo "   Script: $SCRIPT_NAME"
echo "   Version: $SCRIPT_VERSION"
echo "=============================================="
echo

# ---------------------------------------------------------
# CLI
# ---------------------------------------------------------
show_help() {
  cat <<EOF
Usage: ./$SCRIPT_NAME.sh [OPTION]

Options:
  --help           Show help and exit
  --version        Show version and exit
  --home <path>    Set custom SDK installation root (default: ~/IOcomposer)
  --force-update   Force reinstall
  --uninstall      Remove toolchains + Eclipse (keep repos/workspaces)
  (no option)      Install/update (skip if already installed)
EOF
}

MODE="install"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --help) show_help; exit 0 ;;
    --version) echo "$SCRIPT_NAME $SCRIPT_VERSION"; exit 0 ;;
    --force-update) MODE="force"; echo ">>> Force update mode enabled" ;;
    --uninstall) MODE="uninstall"; echo ">>> Uninstall mode enabled" ;;
    --home)
      shift
      if [[ -z "${1:-}" ]]; then
        echo "❌ Missing path after --home"; exit 1
      fi
      ROOT="$1"
      echo ">>> Using custom home folder: $ROOT"
      ;;
    *) echo "Unknown option: $1"; exit 1 ;;
  esac
  shift
done

EXT="$ROOT/external"
mkdir -p "$ROOT" "$EXT"
sudo mkdir -p "$TOOLS" "$BIN"

# ---------------------------------------------------------
# UNINSTALL
# ---------------------------------------------------------
if [[ "$MODE" == "uninstall" ]]; then
  echo "⚠️  Removing toolchains + Eclipse (repos and workspace kept)..."
  read -r -p "Proceed? (y/N) " c; [[ "$c" =~ ^[Yy]$ ]] || exit 0

  echo ">>> Checking xPacks in $TOOLS..."
  for dir in "$TOOLS"/*; do
    [ -d "$dir" ] || continue
    base=$(basename "$dir")
    echo "   Checking $base..."
    case "$base" in
      xpack-arm-none-eabi-*|xpack-riscv-none-elf-*|xpack-openocd-*)
        echo "   - Removing $dir"
        sudo rm -rf "$dir"
        ;;
      *) echo "   - Skipping $dir";;
    esac
  done

  echo ">>> Removing symlinks..."
  sudo rm -f "$BIN/arm-none-eabi-gcc" "$BIN/riscv-none-elf-gcc" "$BIN/openocd" || true

  echo ">>> Removing Eclipse installation..."
  sudo rm -rf "$ECLIPSE_DIR" || true

  echo ">>> Removing Eclipse user settings (~/.eclipse)..."
  rm -rf "$HOME/.eclipse" || true

  echo ">>> Repositories under $ROOT and workspace dirs were kept."
  echo ">>> Uninstall complete!"
  exit 0
fi

# ---------------------------------------------------------
# Arch detect
# ---------------------------------------------------------
ARCH=$(uname -m)
case "$ARCH" in
  aarch64|arm64)
    # Eclipse EPP uses "aarch64" for Linux ARM64.
    ECLIPSE_ARCH="aarch64"
    # xPack release assets typically use "linux-arm64" (sometimes "linux-aarch64").
    XPACK_PLATFORM_CANDIDATES=("linux-arm64" "linux-aarch64")
    ;;
  x86_64|amd64)
    # Eclipse EPP uses "x86_64" for Linux x86_64.
    ECLIPSE_ARCH="x86_64"
    # xPack release assets typically use "linux-x64" (sometimes "linux-x86_64").
    XPACK_PLATFORM_CANDIDATES=("linux-x64" "linux-x86_64")
    ;;
  *)
    echo "Unsupported architecture: $ARCH"
    exit 1
    ;;
esac

XPACK_PLATFORM_CSV=$(IFS=,; echo "${XPACK_PLATFORM_CANDIDATES[*]}")
echo ">>> Detected architecture: $ARCH -> Eclipse arch=$ECLIPSE_ARCH | xPack platforms=$XPACK_PLATFORM_CSV"

# ---------------------------------------------------------
# Helpers
# ---------------------------------------------------------
resolve_path() {
  python3 - "$1" <<'PY'
import os, sys
print(os.path.realpath(sys.argv[1]))
PY
}

java_hash() {
  python3 - "$1" <<'EOF'
import sys
s = sys.argv[1]; h = 0
for c in s: h = (31*h + ord(c)) & 0xFFFFFFFF
if h >= 2**31: h -= 2**32
print(h)
EOF
}

# Checks if the Content-Length is greater than 1MB (1,000,000 bytes)

# ---------------------------------------------------------
# Cleanup (DMG mount/temp)
# ---------------------------------------------------------
TMP_ECLIPSE_DMG=""
ECLIPSE_MNT=""

cleanup() {
  if [[ -n "${ECLIPSE_MNT:-}" ]] && mount | grep -q " ${ECLIPSE_MNT} "; then
    hdiutil detach "$ECLIPSE_MNT" -quiet || true
  fi
  if [[ -n "${TMP_ECLIPSE_DMG:-}" ]]; then
    rm -f "$TMP_ECLIPSE_DMG" || true
  fi
  if [[ -n "${ECLIPSE_MNT:-}" ]]; then
    rm -rf "$ECLIPSE_MNT" || true
  fi
}
trap cleanup EXIT

# ---------------------------------------------------------
# Install IOsonata Eclipse Plugin
# ---------------------------------------------------------
install_iosonata_plugin() {
  echo ">>> Installing IOsonata Eclipse Plugin..."

  local plugin_dir="$ROOT/IOsonata/Installer/eclipse_plugin"
  local dropins_dir="$ECLIPSE_DIR/dropins"

  if [[ ! -d "$plugin_dir" ]]; then
    echo "⚠️ Plugin directory not found at $plugin_dir"
    echo "   Skipping plugin installation."
    return 0
  fi

  local latest_plugin
  latest_plugin=$(ls -1 "$plugin_dir"/org.iosonata.embedcdt.templates.firmware_*.jar 2>/dev/null | sort -V | tail -n1)

  if [[ -z "$latest_plugin" ]]; then
    echo "⚠️ No IOsonata plugin jar file found in $plugin_dir"
    echo "   Skipping plugin installation."
    return 0
  fi

  echo "   → Found plugin: $(basename "$latest_plugin")"

  sudo mkdir -p "$dropins_dir"

  local old_plugins
  old_plugins=$(sudo find "$dropins_dir" -name "org.iosonata.embedcdt.templates.firmware_*.jar" 2>/dev/null || true)

  if [[ -n "$old_plugins" ]]; then
    echo "   → Removing old plugin versions..."
    echo "$old_plugins" | while read -r old_plugin; do
      if [[ -f "$old_plugin" ]]; then
        echo "     - Removing: $(basename "$old_plugin")"
        sudo rm -f "$old_plugin"
      fi
    done
  fi

  echo "   → Installing plugin to $dropins_dir"
  sudo cp "$latest_plugin" "$dropins_dir/"
  sudo chmod 644 "$dropins_dir/$(basename "$latest_plugin")"

  echo "✅ IOsonata Eclipse Plugin installed: $(basename "$latest_plugin")"
}

# ---------------------------------------------------------
# Build IOsonata Library for Selected MCU (headless, no-indexer)
# ---------------------------------------------------------

# ---------------------------------------------------------
# Install xPack toolchain
# ---------------------------------------------------------
install_xpack() {
  local repo=$1 tool=$2 name=$3

  echo ">>> Checking $name..." >&2

  local latest_json
  if ! latest_json=$(curl -fsSL "https://api.github.com/repos/xpack-dev-tools/${repo}/releases/latest"); then
    echo "❌ Failed to query GitHub API for ${repo}. This may be a network issue or GitHub API rate limiting." >&2
    exit 1
  fi

  local latest_tag latest_url
  read -r latest_tag latest_url < <(
    printf '%s' "$latest_json" | python3 -c '
import json, sys

platforms = [p.strip() for p in (sys.argv[1] or "").split(",") if p.strip()]
j = json.load(sys.stdin)

tag = j.get("tag_name", "") or ""
url = ""

# Pick an actual archive (ignore .sha/.sig/.txt) for one of the platform tokens.
assets = (j.get("assets") or [])
preferred_exts = (".tar.gz", ".tgz", ".tar.xz", ".txz", ".zip")

def is_archive(u: str) -> bool:
  return u.endswith(preferred_exts)

# First pass: platform match + archive extension.
for a in assets:
  u = a.get("browser_download_url", "") or ""
  if any(p in u for p in platforms) and is_archive(u):
    url = u
    break

# Second pass: platform match (anything), as a last resort.
if not url:
  for a in assets:
    u = a.get("browser_download_url", "") or ""
    if any(p in u for p in platforms):
      url = u
      break

print(tag, url)
' "$XPACK_PLATFORM_CSV"
  )

  if [[ -z "${latest_tag:-}" || -z "${latest_url:-}" ]]; then
    echo "❌ Could not resolve latest release tag or download URL for $name (repo=$repo, platforms=$XPACK_PLATFORM_CSV)." >&2
    exit 1
  fi

  local latest_norm latest_base
  latest_norm=$(echo "$latest_tag" | sed 's/^v//')
  latest_base=$(echo "$latest_norm" | cut -d- -f1)

  local installed_ver=""
  if command -v "$tool" >/dev/null 2>&1; then
    installed_ver=$($tool --version 2>&1 | head -n1 | grep -oE '[0-9]+\.[0-9]+\.[0-9]+' | head -n1 || true)
  fi

  echo ">>> Installed: ${installed_ver:-none} | Latest: $latest_base" >&2

  if [[ "$MODE" != "force" && -n "$installed_ver" && "$installed_ver" == "$latest_base" ]]; then
    echo "✅ $name already up-to-date ($installed_ver)" >&2
    local bin_path real_bin instdir
    bin_path=$(command -v "$tool")
    real_bin=$(resolve_path "$bin_path")
    instdir=$(dirname "$(dirname "$real_bin")")
    echo "$instdir"
    return 0
  fi

  echo "⬇️ Installing $name $latest_norm..." >&2
  local suffix tmpfile
  case "$latest_url" in
    *.tar.gz|*.tgz) suffix=".tar.gz" ;;
    *.tar.xz|*.txz) suffix=".tar.xz" ;;
    *.zip)          suffix=".zip" ;;
    *)              suffix="" ;;
  esac

  tmpfile=$(mktemp ${suffix:+--suffix="$suffix"})

  if ! curl -fL "$latest_url" -o "$tmpfile"; then
    echo "❌ Failed to download $name from: $latest_url" >&2
    rm -f "$tmpfile" || true
    exit 1
  fi

  # Extract (format-aware)
  if [[ "$latest_url" == *.zip ]]; then
    sudo unzip -q "$tmpfile" -d "$TOOLS"
  elif [[ "$latest_url" == *.tar.gz || "$latest_url" == *.tgz ]]; then
    sudo tar -xzf "$tmpfile" -C "$TOOLS"
  elif [[ "$latest_url" == *.tar.xz || "$latest_url" == *.txz ]]; then
    sudo tar -xJf "$tmpfile" -C "$TOOLS"
  else
    # Last-resort: attempt gzip first, then xz.
    if ! sudo tar -xzf "$tmpfile" -C "$TOOLS" 2>/dev/null; then
      sudo tar -xJf "$tmpfile" -C "$TOOLS"
    fi
  fi

  rm -f "$tmpfile" || true

  local origdir
  origdir=$(ls -d "$TOOLS"/${repo}-* "$TOOLS"/xpack-${repo/-xpack/}-* 2>/dev/null | grep "$latest_norm" | head -n1 || true)
  if [[ -z "$origdir" ]]; then
    echo "❌ Could not find extracted folder for $name ($latest_norm) under $TOOLS" >&2
    exit 1
  fi

  local basever prefix targetdir
  basever=$(echo "$latest_norm" | cut -d- -f1)

  case "$tool" in
    arm-none-eabi-gcc)   prefix="xpack-arm-none-eabi-gcc" ;;
    riscv-none-elf-gcc)  prefix="xpack-riscv-none-elf-gcc" ;;
    openocd)             prefix="xpack-openocd" ;;
    *)                   prefix="xpack-$tool" ;;
  esac

  targetdir="$TOOLS/$prefix-$basever"

  if [[ "$origdir" != "$targetdir" ]]; then
    sudo rm -rf "$targetdir" || true
    sudo mv "$origdir" "$targetdir"
  fi

  sudo ln -sf "$targetdir/bin/$tool" "$BIN/$tool"

  echo "✅ $name installed at $targetdir" >&2
  echo "$targetdir"
}

ARM_DIR=""
RISCV_DIR=""
OPENOCD_DIR=""

# ---------------------------------------------------------
# Toolchains
# ---------------------------------------------------------
echo
ARM_DIR=$(install_xpack "arm-none-eabi-gcc-xpack" "arm-none-eabi-gcc" "ARM GCC")
RISCV_DIR=$(install_xpack "riscv-none-elf-gcc-xpack" "riscv-none-elf-gcc" "RISC-V GCC")
OPENOCD_DIR=$(install_xpack "openocd-xpack" "openocd" "OpenOCD")

echo "✅ Toolchains installed:"
echo "   ARM:    $ARM_DIR"
echo "   RISC-V: $RISCV_DIR"
echo "   OpenOCD:$OPENOCD_DIR"

# ---------------------------------------------------------
# Install Eclipse Embedded CDT
# ---------------------------------------------------------
echo

if [[ "$MODE" != "force" && -d "$ECLIPSE_DIR" ]]; then
  echo "✅ Eclipse already installed at $ECLIPSE_DIR (skipping; use --force-update to reinstall)."
else
  echo "💻 Installing Eclipse Embedded CDT IDE..."
  MIRROR="https://ftp2.osuosl.org/pub/eclipse/technology/epp/downloads/release"

  # Prefer GA release trains from the official EPP release list to avoid selecting
  # future/milestone directories that may appear on mirrors.
  RELEASES=$(curl -fsSL "https://www.eclipse.org/downloads/packages/release" \
    | grep -oE '20[0-9]{2}-[0-9]{2} R' \
    | awk '{print $1}' \
    | head -n 8 \
    | tr '\n' ' ' \
    || true)

  if [[ -z "${RELEASES// }" ]]; then
    # Fallback: enumerate mirror directories (less reliable).
    RELEASES=$(curl -fsSL "$MIRROR/" \
      | grep -oE 'href="20[0-9]{2}-[0-9]{2}/"' \
      | cut -d'"' -f2 \
      | sed 's|/||g' \
      | sort -r \
      | uniq \
      || true)
  fi

  if [[ -z "${RELEASES:-}" ]]; then
    echo "❌ Failed to enumerate Eclipse releases (official list + mirror fallback failed)." >&2
    exit 1
  fi

  ECLIPSE_URL=""
  for release in $RELEASES; do
    URL_EMBEDCDT="$MIRROR/$release/R/eclipse-embedcdt-$release-R-linux-gtk-$ECLIPSE_ARCH.tar.gz"
    URL_EMBEDCPP="$MIRROR/$release/R/eclipse-embedcpp-$release-R-linux-gtk-$ECLIPSE_ARCH.tar.gz"

    if curl --head --silent --fail "$URL_EMBEDCDT" >/dev/null 2>&1; then
      ECLIPSE_URL="$URL_EMBEDCDT"
      break
    elif curl --head --silent --fail "$URL_EMBEDCPP" >/dev/null 2>&1; then
      ECLIPSE_URL="$URL_EMBEDCPP"
      break
    fi
  done

  if [[ -z "$ECLIPSE_URL" ]]; then
    echo "❌ Could not find a valid Eclipse Embedded CDT download URL for any release." >&2
    exit 1
  fi

  echo "⬇️ Downloading Eclipse: $ECLIPSE_URL"

  TMP_ECLIPSE_TAR=$(mktemp)
  if ! curl -fL "$ECLIPSE_URL" -o "$TMP_ECLIPSE_TAR"; then
    echo "❌ Eclipse download failed." >&2
    exit 1
  fi

  echo "📦 Extracting Eclipse..."
  sudo rm -rf "$ECLIPSE_DIR"
  sudo mkdir -p "$ECLIPSE_DIR"
  sudo tar -xzf "$TMP_ECLIPSE_TAR" -C /opt
  # tar creates /opt/eclipse, which is what we want

  rm -f "$TMP_ECLIPSE_TAR" || true

  echo "✅ Eclipse installed at $ECLIPSE_DIR"
fi

# ---------------------------------------------------------
# Seed preferences (user) — no sudo under ~/.eclipse
# ---------------------------------------------------------
seed_eclipse_prefs() {
  echo
  echo ">>> Seeding Eclipse preferences in ~/.eclipse..."

  local base="$HOME/.eclipse"
  mkdir -p "$base"

  # If a previous run created ~/.eclipse as root, fix it (cross-distro).
  local base_uid user_group
  base_uid=$(stat -c '%u' "$base" 2>/dev/null || stat -f '%u' "$base" 2>/dev/null || echo "")
  user_group=$(id -gn "$USER" 2>/dev/null || echo "$USER")
  if [[ "${base_uid:-}" == "0" ]]; then
    echo "⚠️  $base is owned by root (likely from a previous installer run). Fixing ownership..."
    sudo chown -R "$USER":"$user_group" "$base" || true
  fi

  local instance_cfg
  instance_cfg=$(ls -d "$base"/org.eclipse.platform_*/configuration 2>/dev/null | sort -r | head -n1 || true)

  if [[ -z "${instance_cfg:-}" ]]; then
    echo "⚠️  Eclipse user configuration directory not found under $base."
    echo "   Run Eclipse once (it will create ~/.eclipse/org.eclipse.platform_*/configuration), then re-run this installer to seed prefs."
    echo "   Continuing without seeding preferences."
    return 0
  fi

  mkdir -p "$instance_cfg/.settings"

  if [[ ! -w "$instance_cfg/.settings" ]]; then
    echo "⚠️  $instance_cfg/.settings is not writable. Fixing ownership..."
    sudo chown -R "$USER":"$user_group" "$instance_cfg" || true
  fi

  local ARM_HASH RISCV_HASH
  ARM_HASH=$(java_hash "$ARM_DIR/bin")
  RISCV_HASH=$(java_hash "$RISCV_DIR/bin")

  cat > "$instance_cfg/.settings/org.eclipse.cdt.core.prefs" <<EOF
eclipse.preferences.version=1
environment/buildEnvironmentInclude=true
org.eclipse.cdt.core.parser.taskTags=TODO,FIXME,XXX
EOF

  cat > "$instance_cfg/.settings/org.eclipse.embedcdt.core.prefs" <<EOF
eclipse.preferences.version=1
buildtools.path.$ARM_HASH=$ARM_DIR/bin
buildtools.path.$RISCV_HASH=$RISCV_DIR/bin
buildtools.path.strict=true
EOF

  cat > "$instance_cfg/.settings/org.eclipse.embedcdt.managedbuild.core.prefs" <<EOF
eclipse.preferences.version=1
toolchain.path.$ARM_HASH=$ARM_DIR/bin
toolchain.path.$RISCV_HASH=$RISCV_DIR/bin
toolchain.path.strict=true
EOF

  cat > "$instance_cfg/.settings/org.eclipse.embedcdt.debug.gdbjtag.openocd.core.prefs" <<EOF
install.folder=$OPENOCD_DIR/bin
install.folder.strict=true
EOF

  echo "✅ Eclipse preferences seeded in:"
  echo "   $instance_cfg/.settings"
}

seed_eclipse_prefs

# ---------------------------------------------------------
# Set global iosonata_loc system property in eclipse.ini
# ---------------------------------------------------------
echo
echo ">>> Setting iosonata_loc system property in Eclipse installation..."

ECLIPSE_INI="$ECLIPSE_DIR/eclipse.ini"
IOSONATA_LOC_PROP="-Diosonata_loc=$ROOT"

# Keep a backup for recovery/debug
sudo cp -f "$ECLIPSE_INI" "$ECLIPSE_INI.bak" 2>/dev/null || true

# Use python to avoid GNU/BSD sed differences.
sudo python3 - "$ECLIPSE_INI" "$IOSONATA_LOC_PROP" <<'PY'
import sys
ini = sys.argv[1]
prop = sys.argv[2].strip()

with open(ini, "r", encoding="utf-8", errors="replace") as f:
    lines = f.read().splitlines()

# Remove any existing values (idempotent).
lines = [ln for ln in lines if not ln.startswith("-Diosonata_loc=") and not ln.startswith("-Diosonata.home=")]

out = []
inserted = False
for ln in lines:
    out.append(ln)
    if (ln.strip() == "-vmargs") and (not inserted):
        out.append(prop)
        inserted = True

if not inserted:
    out.append(prop)

with open(ini, "w", encoding="utf-8") as f:
    f.write("\n".join(out) + "\n")
PY

echo "✅ iosonata_loc set in eclipse.ini:"
grep "^-Diosonata_loc=" "$ECLIPSE_INI" || true

# ---------------------------------------------------------
# Clone repos
# ---------------------------------------------------------
if [[ -d "$ROOT/IOsonata" ]]; then
  if [[ "$MODE" == "force" ]]; then
    rm -rf "$ROOT/IOsonata"
    git clone --depth=1 https://github.com/IOsonata/IOsonata.git "$ROOT/IOsonata"
  else
    (cd "$ROOT/IOsonata" && git pull --ff-only)
  fi
else
  git clone --depth=1 https://github.com/IOsonata/IOsonata.git "$ROOT/IOsonata"
fi

cd "$EXT"
repos=(
  "https://github.com/NordicSemiconductor/nrfx.git"
  "https://github.com/nrfconnect/sdk-nrf-bm.git"
  "https://github.com/nrfconnect/sdk-nrfxlib.git"
  "https://github.com/IOsonata/nRF5_SDK.git"
  "https://github.com/IOsonata/nRF5_SDK_Mesh.git"
  "https://github.com/boschsensortec/Bosch-BSEC2-Library.git"
  "https://github.com/xioTechnologies/Fusion.git"
  "https://github.com/dlaidig/vqf.git"
  "https://github.com/lvgl/lvgl.git"
  "https://github.com/lwip-tcpip/lwip.git"
  "https://github.com/hathach/tinyusb.git"
)
for repo in "${repos[@]}"; do
  name=$(basename "$repo" .git)
  if [[ "$name" == "Bosch-BSEC2-Library" ]]; then name="BSEC"; fi
  if [[ -d "$name" ]]; then
    if [[ "$MODE" == "force" ]]; then
      rm -rf "$name"
      git clone --depth=1 "$repo" "$name"
    else
      (cd "$name" && git pull --ff-only)
    fi
  else
    git clone --depth=1 "$repo" "$name"
  fi
done

echo "📦 Cloning FreeRTOS-Kernel..."
if [[ -d "FreeRTOS-Kernel" ]]; then
  if [[ "$MODE" == "force" ]]; then
    rm -rf "FreeRTOS-Kernel"
    git clone --depth=1 https://github.com/FreeRTOS/FreeRTOS-Kernel.git FreeRTOS-Kernel
  else
    (cd FreeRTOS-Kernel && git pull --ff-only)
  fi
else
  git clone --depth=1 https://github.com/FreeRTOS/FreeRTOS-Kernel.git FreeRTOS-Kernel
  echo "✅ FreeRTOS-Kernel cloned"
fi

# ---------------------------------------------------------
# Install IOsonata Eclipse Plugin
# ---------------------------------------------------------
echo
install_iosonata_plugin

# ---------------------------------------------------------
# Generate makefile_path.mk
# ---------------------------------------------------------
echo
echo "📝 Generating makefile_path.mk for Makefile-based builds..."

MAKEFILE_PATH_MK="$ROOT/IOsonata/makefile_path.mk"

cat > "$MAKEFILE_PATH_MK" <<EOF
# makefile_path.mk
# Auto-generated by install_iocdevtools_linux.sh $SCRIPT_VERSION
# This file contains all path macros required to compile IOsonata projects using Makefiles
# Include this file in your project Makefile: include \$(IOSONATA_ROOT)/makefile_path.mk

# ============================================
# Toolchain Paths
# ============================================
ARM_GCC_ROOT = $ARM_DIR
ARM_GCC_BIN = $ARM_DIR/bin
ARM_GCC = \$(ARM_GCC_BIN)/arm-none-eabi-gcc
ARM_GPP = \$(ARM_GCC_BIN)/arm-none-eabi-g++
ARM_AS = \$(ARM_GCC_BIN)/arm-none-eabi-as
ARM_LD = \$(ARM_GCC_BIN)/arm-none-eabi-ld
ARM_AR = \$(ARM_GCC_BIN)/arm-none-eabi-ar
ARM_OBJCOPY = \$(ARM_GCC_BIN)/arm-none-eabi-objcopy
ARM_OBJDUMP = \$(ARM_GCC_BIN)/arm-none-eabi-objdump
ARM_SIZE = \$(ARM_GCC_BIN)/arm-none-eabi-size
ARM_GDB = \$(ARM_GCC_BIN)/arm-none-eabi-gdb

RISCV_GCC_ROOT = $RISCV_DIR
RISCV_GCC_BIN = $RISCV_DIR/bin
RISCV_GCC = \$(RISCV_GCC_BIN)/riscv-none-elf-gcc
RISCV_GPP = \$(RISCV_GCC_BIN)/riscv-none-elf-g++
RISCV_AS = \$(RISCV_GCC_BIN)/riscv-none-elf-as
RISCV_LD = \$(RISCV_GCC_BIN)/riscv-none-elf-ld
RISCV_AR = \$(RISCV_GCC_BIN)/riscv-none-elf-ar
RISCV_OBJCOPY = \$(RISCV_GCC_BIN)/riscv-none-elf-objcopy
RISCV_OBJDUMP = \$(RISCV_GCC_BIN)/riscv-none-elf-objdump
RISCV_SIZE = \$(RISCV_GCC_BIN)/riscv-none-elf-size
RISCV_GDB = \$(RISCV_GCC_BIN)/riscv-none-elf-gdb

OPENOCD_ROOT = $OPENOCD_DIR
OPENOCD = $OPENOCD_DIR/bin/openocd

# ============================================
# IOsonata Paths
# ============================================
ifndef IOCOMPOSER_HOME
\$(error IOCOMPOSER_HOME is not set. Please set it to your IOcomposer root directory)
endif

IOSONATA_ROOT = \$(IOCOMPOSER_HOME)/IOsonata
IOSONATA_INCLUDE = \$(IOSONATA_ROOT)/include
IOSONATA_SRC = \$(IOSONATA_ROOT)/src

ARM_ROOT = \$(IOSONATA_ROOT)/ARM
ARM_CMSIS = \$(ARM_ROOT)/CMSIS
ARM_CMSIS_INCLUDE = \$(ARM_CMSIS)/Include
ARM_INCLUDE = \$(ARM_ROOT)/include
ARM_SRC = \$(ARM_ROOT)/src
ARM_LDSCRIPT = \$(ARM_ROOT)/ldscript

RISCV_ROOT = \$(IOSONATA_ROOT)/RISCV
RISCV_INCLUDE = \$(RISCV_ROOT)/include
RISCV_SRC = \$(RISCV_ROOT)/src
RISCV_LDSCRIPT = \$(RISCV_ROOT)/ldscript

EXTERNAL_ROOT = \$(IOCOMPOSER_HOME)/external
NRFX_ROOT = \$(EXTERNAL_ROOT)/nrfx
SDK_NRF_BM_ROOT = \$(EXTERNAL_ROOT)/sdk-nrf-bm
SDK_NRFXLIB_ROOT = \$(EXTERNAL_ROOT)/sdk-nrfxlib
NRF5_SDK_ROOT = \$(EXTERNAL_ROOT)/nRF5_SDK
NRF5_SDK_MESH_ROOT = \$(EXTERNAL_ROOT)/nRF5_SDK_Mesh
BSEC_ROOT = \$(EXTERNAL_ROOT)/BSEC
FUSION_ROOT = \$(EXTERNAL_ROOT)/Fusion
LVGL_ROOT = \$(EXTERNAL_ROOT)/lvgl
LWIP_ROOT = \$(EXTERNAL_ROOT)/lwip
FREERTOS_KERNEL_ROOT = \$(EXTERNAL_ROOT)/FreeRTOS-Kernel
TINYUSB_ROOT = \$(EXTERNAL_ROOT)/tinyusb
EOF

echo "✅ makefile_path.mk created at: $MAKEFILE_PATH_MK"

# ---------------------------------------------------------
# Build IOsonata Library
# ---------------------------------------------------------

# =========================================================
# Build IOsonata Libraries (using standalone script)
# =========================================================
BUILD_SCRIPT="$ROOT/IOsonata/Installer/build_iosonata_lib_linux.sh"

if [[ -f "$BUILD_SCRIPT" ]]; then
  echo ""
  echo "========================================================="
  echo "  IOsonata Library Build"
  echo "========================================================="
  echo ""
  "$BUILD_SCRIPT" --home "$ROOT" || true
else
  echo ""
  echo "ℹ️  To build IOsonata libraries:"
  echo "   ./build_iosonata_lib_linux.sh --home $ROOT"
  echo ""
fi

# ---------------------------------------------------------
# Summary
# ---------------------------------------------------------
echo
echo "=============================================="
echo " IOcomposer MCU Dev Tools Installation Summary"
echo "=============================================="

ECLIPSE_VER="Not installed"
if [[ -f "$ECLIPSE_DIR/.eclipseproduct" ]]; then
  ECLIPSE_VER=$(grep -m1 "^-Declipse.buildId=" "$ECLIPSE_DIR/eclipse.ini" 2>/dev/null | cut -d'=' -f2 || echo "Unknown")
fi

ARM_VER="Not found"
if [[ -n "${ARM_DIR:-}" && -x "$ARM_DIR/bin/arm-none-eabi-gcc" ]]; then
  ARM_VER=$("$ARM_DIR/bin/arm-none-eabi-gcc" --version | head -n1 | grep -oE '[0-9]+\.[0-9]+\.[0-9]+' | head -n1 || echo "Unknown")
fi

RISCV_VER="Not found"
if [[ -n "${RISCV_DIR:-}" && -x "$RISCV_DIR/bin/riscv-none-elf-gcc" ]]; then
  RISCV_VER=$("$RISCV_DIR/bin/riscv-none-elf-gcc" --version | head -n1 | grep -oE '[0-9]+\.[0-9]+\.[0-9]+' | head -n1 || echo "Unknown")
fi

OPENOCD_VER="Not found"
if [[ -n "${OPENOCD_DIR:-}" && -x "$OPENOCD_DIR/bin/openocd" ]]; then
  OPENOCD_VER=$("$OPENOCD_DIR/bin/openocd" --version 2>&1 | head -n1 | grep -oE '[0-9]+\.[0-9]+\.[0-9]+' | head -n1 || echo "Unknown")
fi

printf "%-25s %s\n" "Eclipse Embedded CDT:" "$ECLIPSE_VER"
printf "%-25s %s\n" "ARM GCC:" "$ARM_VER"
printf "%-25s %s\n" "RISC-V GCC:" "$RISCV_VER"
printf "%-25s %s\n" "OpenOCD:" "$OPENOCD_VER"
printf "%-25s %s\n" "iosonata_loc:" "$ROOT"

echo "=============================================="
echo " Installation complete!"
echo "=============================================="
echo
echo "✅ iosonata_loc is configured in Eclipse installation"
echo
echo "Usage in .cproject files:"
echo "  \${system_property:iosonata_loc}/IOsonata/include"
echo "  \${system_property:iosonata_loc}/IOsonata/ARM/include"
echo "  \${system_property:iosonata_loc}/IOsonata/ARM/CMSIS/Include"
echo
echo "If prefs seeding was skipped, run Eclipse once, then re-run this script to seed Embedded CDT toolchain paths."
echo
