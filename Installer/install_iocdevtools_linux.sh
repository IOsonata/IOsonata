#!/usr/bin/env bash
set -euo pipefail

SCRIPT_NAME="install_iocdevtools_linux"
SCRIPT_VERSION="v1.0.89"

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
    ECLIPSE_ARCH="aarch64"
    XPACK_PLATFORM_CANDIDATES=("linux-arm64" "linux-aarch64")
    ;;
  x86_64|amd64)
    ECLIPSE_ARCH="x86_64"
    XPACK_PLATFORM_CANDIDATES=("linux-x64" "linux-x86_64")
    ;;
  *)
    echo "Unsupported architecture: $ARCH"
    exit 1
    ;;
esac
XPACK_PLATFORM_CSV=$(IFS=,; echo "${XPACK_PLATFORM_CANDIDATES[*]}")
echo ">>> Detected architecture: $ARCH -> Eclipse arch=$ECLIPSE_ARCH"

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

# ---------------------------------------------------------
# Cleanup
# ---------------------------------------------------------
TMP_ECLIPSE_DMG=""
cleanup() {
  if [[ -n "${TMP_ECLIPSE_DMG:-}" ]]; then rm -f "$TMP_ECLIPSE_DMG" || true; fi
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
    echo "⚠️ Plugin directory not found at $plugin_dir (Skipping)"
    return 0
  fi

  local latest_plugin
  latest_plugin=$(ls -1 "$plugin_dir"/org.iosonata.embedcdt.templates.firmware_*.jar 2>/dev/null | sort -V | tail -n1)

  if [[ -z "$latest_plugin" ]]; then
    echo "⚠️ No IOsonata plugin jar file found (Skipping)"
    return 0
  fi

  sudo mkdir -p "$dropins_dir"
  sudo find "$dropins_dir" -name "org.iosonata.embedcdt.templates.firmware_*.jar" -delete
  sudo cp "$latest_plugin" "$dropins_dir/"
  sudo chmod 644 "$dropins_dir/$(basename "$latest_plugin")"
  echo "✅ Installed: $(basename "$latest_plugin")"
}

# ---------------------------------------------------------
# Install xPack toolchain
# ---------------------------------------------------------
install_xpack() {
  local repo=$1 tool=$2 name=$3
  echo ">>> Checking $name..." >&2

  local latest_json
  if ! latest_json=$(curl -fsSL "https://api.github.com/repos/xpack-dev-tools/${repo}/releases/latest"); then
    echo "❌ GitHub API check failed for $name" >&2
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
assets = (j.get("assets") or [])
preferred_exts = (".tar.gz", ".tgz", ".tar.xz", ".txz", ".zip")
for a in assets:
  u = a.get("browser_download_url", "") or ""
  if any(p in u for p in platforms) and u.endswith(preferred_exts):
    url = u
    break
if not url:
  for a in assets:
    u = a.get("browser_download_url", "") or ""
    if any(p in u for p in platforms):
      url = u
      break
print(tag, url)
' "$XPACK_PLATFORM_CSV"
  )

  if [[ -z "$latest_url" ]]; then echo "❌ No URL found for $name"; exit 1; fi
  local latest_norm=$(echo "$latest_tag" | sed 's/^v//')
  local basever=$(echo "$latest_norm" | cut -d- -f1)

  local installed_ver=""
  if command -v "$tool" >/dev/null 2>&1; then
    installed_ver=$($tool --version 2>&1 | head -n1 | grep -oE '[0-9]+\.[0-9]+\.[0-9]+' | head -n1 || true)
  fi

  if [[ "$MODE" != "force" && "$installed_ver" == "$basever" ]]; then
    echo "✅ $name up-to-date ($installed_ver)" >&2
    local bin_path=$(command -v "$tool")
    local real_bin=$(resolve_path "$bin_path")
    echo "$(dirname "$(dirname "$real_bin")")"
    return 0
  fi

  echo "⬇️ Installing $name $latest_norm..." >&2
  local tmpfile=$(mktemp)
  curl -fL "$latest_url" -o "$tmpfile"
  
  if [[ "$latest_url" == *.zip ]]; then sudo unzip -q "$tmpfile" -d "$TOOLS"
  elif [[ "$latest_url" == *.tar.xz || "$latest_url" == *.txz ]]; then sudo tar -xJf "$tmpfile" -C "$TOOLS"
  else sudo tar -xzf "$tmpfile" -C "$TOOLS"; fi
  rm -f "$tmpfile"

  local origdir=$(ls -d "$TOOLS"/${repo}-* "$TOOLS"/xpack-${repo/-xpack/}-* 2>/dev/null | grep "$latest_norm" | head -n1)
  local prefix="xpack-$tool"
  [[ "$tool" == "arm-none-eabi-gcc" ]] && prefix="xpack-arm-none-eabi-gcc"
  [[ "$tool" == "riscv-none-elf-gcc" ]] && prefix="xpack-riscv-none-elf-gcc"
  [[ "$tool" == "openocd" ]] && prefix="xpack-openocd"

  local targetdir="$TOOLS/$prefix-$basever"
  if [[ "$origdir" != "$targetdir" ]]; then sudo rm -rf "$targetdir"; sudo mv "$origdir" "$targetdir"; fi
  sudo ln -sf "$targetdir/bin/$tool" "$BIN/$tool"

  echo "✅ Installed at $targetdir" >&2
  echo "$targetdir"
}

echo
ARM_DIR=$(install_xpack "arm-none-eabi-gcc-xpack" "arm-none-eabi-gcc" "ARM GCC")
RISCV_DIR=$(install_xpack "riscv-none-elf-gcc-xpack" "riscv-none-elf-gcc" "RISC-V GCC")
OPENOCD_DIR=$(install_xpack "openocd-xpack" "openocd" "OpenOCD")

# ---------------------------------------------------------
# Install Eclipse
# ---------------------------------------------------------
echo
if [[ "$MODE" != "force" && -d "$ECLIPSE_DIR" ]]; then
  echo "✅ Eclipse already installed at $ECLIPSE_DIR"
else
  echo "💻 Installing Eclipse Embedded CDT..."
  MIRROR="https://ftp2.osuosl.org/pub/eclipse/technology/epp/downloads/release"
  CURRENT_YEAR=$(date +"%Y")
  YEARS=(); for i in {0..3}; do YEARS+=($((CURRENT_YEAR - i))); done
  MONTHS=("12" "09" "06" "03")
  ECLIPSE_URL=""
  
  echo "🔍 Probing for latest release..."
  for y in "${YEARS[@]}"; do
    if [[ -n "$ECLIPSE_URL" ]]; then break; fi
    for m in "${MONTHS[@]}"; do
      REL="${y}-${m}"
      U1="$MIRROR/$REL/R/eclipse-embedcdt-$REL-R-linux-gtk-$ECLIPSE_ARCH.tar.gz"
      U2="$MIRROR/$REL/R/eclipse-embedcpp-$REL-R-linux-gtk-$ECLIPSE_ARCH.tar.gz"
      if curl --head --silent --fail "$U1" >/dev/null 2>&1; then ECLIPSE_URL="$U1"; break; fi
      if curl --head --silent --fail "$U2" >/dev/null 2>&1; then ECLIPSE_URL="$U2"; break; fi
    done
  done

  if [[ -z "$ECLIPSE_URL" ]]; then echo "❌ No Eclipse URL found."; exit 1; fi
  echo "⬇️ Downloading: $ECLIPSE_URL"
  TMP=$(mktemp)
  curl -fL "$ECLIPSE_URL" -o "$TMP"
  sudo rm -rf "$ECLIPSE_DIR"; sudo mkdir -p "$ECLIPSE_DIR"
  sudo tar -xzf "$TMP" -C /opt
  rm -f "$TMP"
  echo "✅ Eclipse installed at $ECLIPSE_DIR"
fi

# ---------------------------------------------------------
# Seed preferences (Parity with macOS)
# ---------------------------------------------------------
seed_eclipse_prefs() {
  echo; echo ">>> Seeding Eclipse preferences..."
  local base="$HOME/.eclipse"
  mkdir -p "$base"
  
  # Ownership fix
  local base_uid=$(stat -c '%u' "$base" 2>/dev/null || stat -f '%u' "$base" 2>/dev/null || echo "")
  if [[ "$base_uid" == "0" ]]; then sudo chown -R "$USER":$(id -gn) "$base"; fi

  local instance_cfg=$(ls -d "$base"/org.eclipse.platform_*/configuration 2>/dev/null | sort -r | head -n1)
  if [[ -z "$instance_cfg" ]]; then
    echo "⚠️  Eclipse config dir not found (Run Eclipse once to generate it)."
    return 0
  fi
  mkdir -p "$instance_cfg/.settings"

  # Parity: Hash the PATH, not the name
  local ARM_HASH=$(java_hash "$ARM_DIR/bin")
  local RISCV_HASH=$(java_hash "$RISCV_DIR/bin")

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

  echo "✅ Preferences seeded in $instance_cfg/.settings"
}

seed_eclipse_prefs

# ---------------------------------------------------------
# eclipse.ini (iosonata_loc)
# ---------------------------------------------------------
echo; echo ">>> Configuring eclipse.ini..."
ECLIPSE_INI="$ECLIPSE_DIR/eclipse.ini"
IOSONATA_LOC_PROP="-Diosonata_loc=$ROOT"
sudo cp -f "$ECLIPSE_INI" "$ECLIPSE_INI.bak"

sudo python3 - "$ECLIPSE_INI" "$IOSONATA_LOC_PROP" <<'PY'
import sys
ini = sys.argv[1]; prop = sys.argv[2].strip()
with open(ini, "r") as f: lines = f.read().splitlines()
lines = [ln for ln in lines if not ln.startswith("-Diosonata_loc=") and not ln.startswith("-Diosonata.home=")]
out = []; inserted = False
for ln in lines:
    out.append(ln)
    if ln.strip() == "-vmargs" and not inserted:
        out.append(prop); inserted = True
if not inserted: out.append(prop)
with open(ini, "w") as f: f.write("\n".join(out) + "\n")
PY
echo "✅ iosonata_loc set."

# ---------------------------------------------------------
# Clone repos
# ---------------------------------------------------------
if [[ ! -d "$ROOT/IOsonata" ]]; then
  git clone --depth=1 https://github.com/IOsonata/IOsonata.git "$ROOT/IOsonata"
else
  (cd "$ROOT/IOsonata" && git pull --ff-only) || true
fi
if [[ -d "$ROOT/IOsonata/Installer" ]]; then chmod +x "$ROOT/IOsonata/Installer/"*.sh; fi

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
  [[ "$name" == "Bosch-BSEC2-Library" ]] && name="BSEC"
  if [[ ! -d "$name" ]]; then git clone --depth=1 "$repo" "$name"; else (cd "$name" && git pull --ff-only) || true; fi
done

if [[ ! -d "FreeRTOS-Kernel" ]]; then
  git clone --depth=1 https://github.com/FreeRTOS/FreeRTOS-Kernel.git FreeRTOS-Kernel
else
  (cd FreeRTOS-Kernel && git pull --ff-only) || true
fi

# ---------------------------------------------------------
# Install Plugin (after cloning)
# ---------------------------------------------------------
install_iosonata_plugin

# ---------------------------------------------------------
# Generate makefile_path.mk
# ---------------------------------------------------------
echo; echo "📝 Generating makefile_path.mk..."
MAKEFILE_PATH_MK="$ROOT/IOsonata/makefile_path.mk"
cat > "$MAKEFILE_PATH_MK" <<EOF
# makefile_path.mk
# Generated by install_iocdevtools_linux.sh $SCRIPT_VERSION
ifndef IOCOMPOSER_HOME
\$(error IOCOMPOSER_HOME is not set)
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
EOF
echo "✅ makefile_path.mk created."

# ---------------------------------------------------------
# Build
# ---------------------------------------------------------
BUILD_SCRIPT="$ROOT/IOsonata/Installer/build_iosonata_lib_linux.sh"
if [[ -f "$BUILD_SCRIPT" ]]; then
  echo; echo ">>> Building IOsonata Libraries..."
  chmod +x "$BUILD_SCRIPT"
  "$BUILD_SCRIPT" --home "$ROOT" || true
else
  echo; echo "ℹ️  Build script not found (will be available after full sync)."
fi

echo; echo "=============================================="
echo " Installation complete!"
echo "=============================================="