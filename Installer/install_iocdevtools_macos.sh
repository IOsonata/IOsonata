#!/usr/bin/env bash
set -euo pipefail

SCRIPT_NAME="install_iocdevtools_macos"
SCRIPT_VERSION="v1.0.78"

ROOT="$HOME/IOcomposer"
TOOLS="/opt/xPacks"
BIN="/usr/local/bin"
ECLIPSE_APP="/Applications/Eclipse.app"

echo "=============================================="
echo "   IOcomposer MCU Dev Tools Installer (macOS) "
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
  sudo rm -rf "$ECLIPSE_APP" || true

  echo ">>> Removing Eclipse user settings (~/.eclipse)..."
  rm -rf "$HOME/.eclipse" || true

  echo ">>> Removing IDAP tools..."
  rm -rf "$ROOT/IDAP" || true

  # Prompt for IOsonata and external SDK removal
  echo
  read -r -p "Also remove IOsonata and external SDK folders? (y/N) " ans2
  if [[ "$ans2" =~ ^[Yy]$ ]]; then
    if [[ -d "$ROOT/IOsonata" ]]; then
      rm -rf "$ROOT/IOsonata"
      echo "   ✅ IOsonata removed."
    fi
    if [[ -d "$ROOT/external" ]]; then
      rm -rf "$ROOT/external"
      echo "   ✅ External SDK removed."
    fi
    if [[ -d "$ROOT/.iocomposer" ]]; then
      rm -rf "$ROOT/.iocomposer"
      echo "   ✅ .iocomposer removed."
    fi
  else
    echo ">>> Repositories under $ROOT and workspace dirs were kept."
  fi


  # ---------------------------------------------------------
  # Remove $ROOT if it is truly empty (no files/dirs).
  # Uses Bash globbing (more reliable than 'find' across environments)
  # and prints the first entry if not empty for easy debugging.
  # ---------------------------------------------------------
  if [[ -d "$ROOT" ]]; then
    # Include dotfiles in the check; treat missing matches as empty.
    shopt -s nullglob dotglob
    entries=( "$ROOT"/* )
    shopt -u nullglob dotglob

    if (( ${#entries[@]} == 0 )); then
      # Avoid "directory busy" if caller happens to be inside $ROOT.
      if ( cd "$HOME" && rmdir "$ROOT" 2>/dev/null ); then
        echo "   ✅ Removed empty root folder: $ROOT"
      else
        echo "   ⚠️  Root folder is empty but could not be removed: $ROOT"
      fi
    else
      echo "   ℹ️  Keeping $ROOT (not empty). First entry: ${entries[0]}"
    fi
  fi

  echo ">>> Uninstall complete!"
  exit 0
fi

mkdir -p "$ROOT" "$EXT"
sudo mkdir -p "$TOOLS" "$BIN"

# ---------------------------------------------------------
# Arch detect
# ---------------------------------------------------------
ARCH=$(uname -m)
case "$ARCH" in
  arm64) PLATFORM="darwin-arm64"; ECLIPSE_ARCH="aarch64" ;;
  x86_64|amd64) PLATFORM="darwin-x64"; ECLIPSE_ARCH="x86_64" ;;
  *) echo "Unsupported architecture: $ARCH"; exit 1 ;;
esac
echo ">>> Detected architecture: $ARCH -> Eclipse arch=$ECLIPSE_ARCH"

# ---------------------------------------------------------
# Helpers
# ---------------------------------------------------------
resolve_path() {
  local target=$1
  if command -v greadlink >/dev/null 2>&1; then greadlink -f "$target"
  elif command -v realpath >/dev/null 2>&1; then realpath "$target"
  else /usr/bin/stat -f "%Y" "$target"
  fi
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

# Helper function to validate a download URL
# Checks if the Content-Length is greater than 1MB (1,000,000 bytes)
is_valid_dmg_url() {
  local url="$1"
  # --- DEBUG ECHO REMOVED ---
  # echo "   Checking URL: $(basename "$url")"

  local headers
  headers=$(curl -s -L -I --max-time 10 "$url")

  local length_line
  length_line=$(echo "$headers" | grep -i '^Content-Length:' | tail -n 1)

  if [[ -z "$length_line" ]]; then
    # --- DEBUG ECHO REMOVED ---
    # echo "   -> Failed: No Content-Length header found."
    return 1
  fi

  local length
  length=$(echo "$length_line" | awk '{print $2}' | tr -d '\r')

  if ! [[ "$length" =~ ^[0-9]+$ ]]; then
    # --- DEBUG ECHO REMOVED ---
    # echo "   -> Failed: Content-Length is not a number ($length)."
    return 1
  fi

  if (( length > 1000000 )); then
    # --- DEBUG ECHO REMOVED ---
    # echo "   -> Success: Found file, size ${length} bytes."
    return 0 # Success
  else
    # --- DEBUG ECHO REMOVED ---
    # echo "   -> Failed: File size ($length bytes) is too small. Likely a 404 page."
    return 1 # Failure
  fi
}

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
  local dropins_dir="$ECLIPSE_APP/Contents/Eclipse/dropins"
  
  # Check if plugin directory exists
  if [[ ! -d "$plugin_dir" ]]; then
    echo "⚠️ Plugin directory not found at $plugin_dir"
    echo "   Skipping plugin installation."
    return
  fi
  
  # Find the latest plugin jar file
  local latest_plugin
  latest_plugin=$(ls -1 "$plugin_dir"/org.iosonata.embedcdt.templates.wizard_*.jar 2>/dev/null | sort -V | tail -n1)
  
  if [[ -z "$latest_plugin" ]]; then
    echo "⚠️ No IOsonata plugin jar file found in $plugin_dir"
    echo "   Skipping plugin installation."
    return
  fi
  
  echo "   → Found plugin: $(basename "$latest_plugin")"
  
  # Create dropins directory if it doesn't exist
  sudo mkdir -p "$dropins_dir"
  
  # Remove old versions of the plugin
  local old_plugins
  old_plugins=$(sudo find "$dropins_dir" -name "org.iosonata.embedcdt.templates.wizard_*.jar" 2>/dev/null || true)
  
  if [[ -n "$old_plugins" ]]; then
    echo "   → Removing old plugin versions..."
    echo "$old_plugins" | while read -r old_plugin; do
      if [[ -f "$old_plugin" ]]; then
        echo "     - Removing: $(basename "$old_plugin")"
        sudo rm -f "$old_plugin"
      fi
    done
  fi
  
  # Copy the latest plugin to dropins
  echo "   → Installing plugin to $dropins_dir"
  sudo cp "$latest_plugin" "$dropins_dir/"
  
  # Set proper permissions
  sudo chmod 644 "$dropins_dir/$(basename "$latest_plugin")"
  
  echo "✅ IOsonata Eclipse Plugin installed: $(basename "$latest_plugin")"
}


# ---------------------------------------------------------
# Install xPack toolchain
# ---------------------------------------------------------
install_xpack() {
  local repo=$1 tool=$2 name=$3

  echo ">>> Checking $name..." >&2
  local latest_json=$(curl -s https://api.github.com/repos/xpack-dev-tools/${repo}/releases/latest)
  local latest_tag=$(echo "$latest_json" | grep '"tag_name"' | cut -d '"' -f 4)
  local latest_norm=$(echo "$latest_tag" | sed 's/^v//')
  local latest_base=$(echo "$latest_norm" | cut -d- -f1)
  local latest_url=$(echo "$latest_json" | grep "browser_download_url" | grep "$PLATFORM" | cut -d '"' -f 4 | head -n1)

  local installed_ver=""
  if command -v "$tool" >/dev/null; then
  #  installed_ver=$($tool --version | head -n1 | grep -oE '[0-9]+\.[0-9]+\.[0-9]+')
    installed_ver=$($tool --version 2>&1 | head -n1 | grep -oE '[0-9]+\.[0-9]+\.[0-9]+')
  fi

  echo ">>> Installed: ${installed_ver:-none} | Latest: $latest_base" >&2

  if [[ "$MODE" != "force" && -n "$installed_ver" && "$installed_ver" == "$latest_base" ]]; then
    echo "✅ $name already up-to-date ($installed_ver)" >&2
    bin_path=$(command -v "$tool")
    real_bin=$(resolve_path "$bin_path")
    instdir=$(dirname "$(dirname "$real_bin")")
    echo "$instdir"
    return 0
  fi

  echo "⬇️ Installing $name $latest_norm..." >&2
  curl -L "$latest_url" | sudo tar -xJ -C "$TOOLS"

  origdir=$(ls -d "$TOOLS"/${repo}-* "$TOOLS"/xpack-${repo/-xpack/}-* 2>/dev/null | grep "$latest_norm" | head -n1 || true)
  if [[ -z "$origdir" ]]; then
    echo "❌ Could not find extracted folder for $name ($latest_norm)" >&2
    exit 1
  fi

  basever=$(echo "$latest_norm" | cut -d- -f1)
  targetdir="$TOOLS/$(echo $(basename "$origdir") | cut -d- -f1-4)-$basever"
  if [[ "$origdir" != "$targetdir" ]]; then
    sudo rm -rf "$targetdir" || true
    sudo mv "$origdir" "$targetdir"
  fi

  # Create only generic alias
  sudo ln -sf "$targetdir/bin/$tool" "$BIN/$tool"

  echo "✅ $name installed at $targetdir" >&2
  echo "$targetdir"
}

ARM_DIR=$(install_xpack "arm-none-eabi-gcc-xpack" "arm-none-eabi-gcc" "ARM GCC")
RISCV_DIR=$(install_xpack "riscv-none-elf-gcc-xpack" "riscv-none-elf-gcc" "RISC-V GCC")
OPENOCD_DIR=$(install_xpack "openocd-xpack" "openocd" "OpenOCD")

if [[ -n "$RISCV_DIR" && ! -f "$RISCV_DIR/bin/riscv-none-elf-gcc" && -f "$RISCV_DIR/bin/riscv64-unknown-elf-gcc" ]]; then
  sudo ln -sf "$RISCV_DIR/bin/riscv64-unknown-elf-gcc" "$RISCV_DIR/bin/riscv-none-elf-gcc"
  sudo ln -sf "$RISCV_DIR/bin/riscv-none-elf-gcc" "$BIN/riscv-none-elf-gcc"
fi

# ---------------------------------------------------------
# Install IDAP tools
# ---------------------------------------------------------
IDAP_DIR="$ROOT/IDAP"
mkdir -p "$IDAP_DIR"

IDAP_PROG="$IDAP_DIR/IDAPnRFProg"
if [[ ! -f "$IDAP_PROG" || "$MODE" == "force" ]]; then
  echo
  echo ">>> Downloading IDAPnRFProg tool..."
  IDAP_URL="https://sourceforge.net/projects/idaplinkfirmware/files/OSX/IDAPnRFProg_OSX_2_1_240807.zip/download"
  IDAP_TMP=$(mktemp)
  if curl -fL "$IDAP_URL" -o "$IDAP_TMP"; then
    unzip -o -j "$IDAP_TMP" -d "$IDAP_DIR" 2>/dev/null || true
    chmod +x "$IDAP_DIR/IDAPnRFProg" 2>/dev/null || true
    rm -f "$IDAP_TMP"
    echo "✅ IDAPnRFProg installed at: $IDAP_PROG"
  else
    echo "⚠️  Failed to download IDAPnRFProg. You can download manually from:"
    echo "   https://sourceforge.net/projects/idaplinkfirmware/files/OSX/"
    rm -f "$IDAP_TMP"
  fi
else
  echo "✅ IDAPnRFProg already installed (skipping)"
fi

# ---------------------------------------------------------
# Install Eclipse Embedded CDT
# ---------------------------------------------------------
echo
echo "💻 Installing Eclipse Embedded CDT IDE..."
MIRROR="https://ftp2.osuosl.org/pub/eclipse/technology/epp/downloads/release"

RELEASES=$(curl -fsSL "$MIRROR/" | grep -oE 'href="20[0-9]{2}-[0-9]{2}/"' | cut -d'"' -f2 | sed 's|/||g' | sort -r | uniq || true)
if [[ -z "${RELEASES:-}" ]]; then
  echo "❌ Failed to enumerate Eclipse releases from mirror: $MIRROR" >&2
  exit 1
fi

ECLIPSE_URL=""
for release in $RELEASES; do
  URL_EMBEDCDT="$MIRROR/$release/R/eclipse-embedcdt-$release-R-macosx-cocoa-$ECLIPSE_ARCH.dmg"
  URL_EMBEDCPP="$MIRROR/$release/R/eclipse-embedcpp-$release-R-macosx-cocoa-$ECLIPSE_ARCH.dmg"

  if is_valid_dmg_url "$URL_EMBEDCDT"; then
    ECLIPSE_URL="$URL_EMBEDCDT"
    break
  elif is_valid_dmg_url "$URL_EMBEDCPP"; then
    ECLIPSE_URL="$URL_EMBEDCPP"
    break
  fi
done

if [[ -z "$ECLIPSE_URL" ]]; then
  echo "❌ Could not find a valid Eclipse Embedded CDT download URL for any release." >&2
  exit 1
fi

echo "⬇️ Downloading Eclipse: $ECLIPSE_URL"

TMP_ECLIPSE_DMG=$(mktemp)
if ! curl -fL "$ECLIPSE_URL" -o "$TMP_ECLIPSE_DMG"; then
  echo "❌ Eclipse download failed." >&2
  exit 1
fi

ECLIPSE_MNT=$(mktemp -d /tmp/eclipse.XXXX)

if ! hdiutil attach "$TMP_ECLIPSE_DMG" -mountpoint "$ECLIPSE_MNT" -nobrowse -quiet; then
  echo "❌ Failed to mount Eclipse DMG." >&2
  exit 1
fi

sudo rm -rf "$ECLIPSE_APP"
sudo ditto "$ECLIPSE_MNT/Eclipse.app" "$ECLIPSE_APP"

hdiutil detach "$ECLIPSE_MNT" -quiet || true
rm -f "$TMP_ECLIPSE_DMG" || true
rm -rf "$ECLIPSE_MNT" || true

TMP_ECLIPSE_DMG=""
ECLIPSE_MNT=""

echo "✅ Eclipse installed at $ECLIPSE_APP"

# ---------------------------------------------------------
# Seed Eclipse prefs
# ---------------------------------------------------------

ECLIPSE_SETTINGS="$ECLIPSE_APP/Contents/Eclipse/configuration/.settings"
sudo mkdir -p "$ECLIPSE_SETTINGS"

ARM_HASH=$(java_hash "xPack GNU Arm Embedded GCC")
if (( ARM_HASH < 0 )); then ARM_HASH=$((ARM_HASH + 4294967296)); fi

RISCV_HASH=$(java_hash "xPack GNU RISC-V Embedded GCC")
if (( RISCV_HASH < 0 )); then RISCV_HASH=$((RISCV_HASH + 4294967296)); fi
RISCV_HASH=$((RISCV_HASH + 1))

sudo sh -c "cat > '$ECLIPSE_SETTINGS/org.eclipse.embedcdt.core.prefs' <<EOF
eclipse.preferences.version=1
xpack.arm.toolchain.path=$ARM_DIR/bin
xpack.riscv.toolchain.path=$RISCV_DIR/bin
xpack.openocd.path=$OPENOCD_DIR/bin
xpack.strict=true
EOF"

sudo sh -c "cat > '$ECLIPSE_SETTINGS/org.eclipse.embedcdt.managedbuild.cross.arm.core.prefs' <<EOF
toolchain.path.$ARM_HASH=$ARM_DIR/bin
toolchain.path.1287942917=$ARM_DIR/bin
toolchain.path.strict=true
EOF"

sudo sh -c "cat > '$ECLIPSE_SETTINGS/org.eclipse.embedcdt.managedbuild.cross.riscv.core.prefs' <<EOF
toolchain.path.$RISCV_HASH=$RISCV_DIR/bin
toolchain.path.strict=true
EOF"

sudo sh -c "cat > '$ECLIPSE_SETTINGS/org.eclipse.embedcdt.debug.gdbjtag.openocd.core.prefs' <<EOF
install.folder=$OPENOCD_DIR/bin
install.folder.strict=true
EOF"

sudo sh -c "cat > '$ECLIPSE_SETTINGS/org.eclipse.core.runtime.prefs' <<EOF
eclipse.preferences.version=1
environment/project/IOCOMPOSER_HOME/value=$ROOT
environment/project/ARM_GCC_HOME/value=$ARM_DIR/bin
environment/project/RISCV_GCC_HOME/value=$RISCV_DIR/bin
environment/project/OPENOCD_HOME/value=$OPENOCD_DIR/bin
environment/project/NRFX_HOME/value=$EXT/nrfx
environment/project/NRFXLIB_HOME/value=$EXT/sdk-nrfxlib
environment/project/NRF5_SDK_HOME/value=$EXT/nRF5_SDK
environment/project/NRF5_SDK_MESH_HOME/value=$EXT/nRF5_SDK_Mesh
environment/project/BSEC_HOME/value=$EXT/BSEC
EOF"

echo "✅ Eclipse preferences seeded (ARM, RISC-V, OpenOCD, macros)."

# ---------------------------------------------------------
# Configure iosonata_loc and iocomposer_home as Java System Properties
# ---------------------------------------------------------
echo
echo ">>> Configuring IOsonata and IOcomposer system properties in eclipse.ini..."

ECLIPSE_INI="$ECLIPSE_APP/Contents/Eclipse/eclipse.ini"

# Remove old properties if they exist
sudo sed -i.bak '/^-Diosonata\.home=/d' "$ECLIPSE_INI"
sudo sed -i '' '/^-Diosonata_loc=/d' "$ECLIPSE_INI"
sudo sed -i '' '/^-Diocomposer_home=/d' "$ECLIPSE_INI"

# Find the -vmargs line and insert after it
# If no -vmargs, create it first
if grep -q "^-vmargs" "$ECLIPSE_INI"; then
    # Insert after -vmargs line
    sudo sed -i '' '/^-vmargs$/a\
-Diosonata_loc='"$ROOT"'
' "$ECLIPSE_INI"
    sudo sed -i '' '/^-Diosonata_loc=/a\
-Diocomposer_home='"$ROOT"'
' "$ECLIPSE_INI"
else
    # No -vmargs section, add it at the end with our properties
    echo "-vmargs" | sudo tee -a "$ECLIPSE_INI" > /dev/null
    echo "-Diosonata_loc=$ROOT" | sudo tee -a "$ECLIPSE_INI" > /dev/null
    echo "-Diocomposer_home=$ROOT" | sudo tee -a "$ECLIPSE_INI" > /dev/null
fi

echo "✅ System properties configured in eclipse.ini:"
echo "   iosonata_loc=$ROOT"
echo "   iocomposer_home=$ROOT"
echo

# Step 1: Ensure Eclipse has initialized its instance folder
if [[ -d "$ECLIPSE_APP" ]]; then
  echo "⏳ Initializing Eclipse to create instance configuration..."
  "$ECLIPSE_APP/Contents/MacOS/eclipse" -nosplash -initialize || true
fi

# Step 2: Find the newest instance configuration folder under ~/.eclipse
INSTANCE_CFG=""
if command -v python3 >/dev/null 2>&1; then
  INSTANCE_CFG=$(python3 - <<'PY'
import os, sys
base = os.path.expanduser("~/.eclipse")
cands = []
if os.path.isdir(base):
  for root, dirs, _ in os.walk(base):
    for d in dirs:
      if d == "configuration":
        p = os.path.join(root, d)
        try:
          cands.append((os.path.getmtime(p), p))
        except OSError:
          pass
if not cands:
  sys.exit(1)
cands.sort(reverse=True)
print(cands[0][1])
PY
) || INSTANCE_CFG=""
fi

if [[ -z "$INSTANCE_CFG" ]]; then
  echo "⚠️ Could not find Eclipse instance configuration folder under ~/.eclipse."
  echo "   Eclipse may not have been started yet."
else
  echo "📂 Found Eclipse settings: $INSTANCE_CFG"
  mkdir -p "$INSTANCE_CFG/.settings"

  cat > "$INSTANCE_CFG/.settings/org.eclipse.embedcdt.core.prefs" <<EOF
eclipse.preferences.version=1
xpack.arm.toolchain.path=$ARM_DIR/bin
xpack.riscv.toolchain.path=$RISCV_DIR/bin
xpack.openocd.path=$OPENOCD_DIR/bin
xpack.strict=true
EOF

  cat > "$INSTANCE_CFG/.settings/org.eclipse.embedcdt.managedbuild.cross.arm.core.prefs" <<EOF
eclipse.preferences.version=1
toolchain.path.$ARM_HASH=$ARM_DIR/bin
toolchain.path.1287942917=$ARM_DIR/bin
toolchain.path.strict=true
EOF

  cat > "$INSTANCE_CFG/.settings/org.eclipse.embedcdt.managedbuild.cross.riscv.core.prefs" <<EOF
eclipse.preferences.version=1
toolchain.path.$RISCV_HASH=$RISCV_DIR/bin
toolchain.path.strict=true
EOF

  cat > "$INSTANCE_CFG/.settings/org.eclipse.embedcdt.debug.gdbjtag.openocd.core.prefs" <<EOF
eclipse.preferences.version=1
install.folder=$OPENOCD_DIR/bin
install.folder.strict=true
EOF

  cat > "$INSTANCE_CFG/.settings/org.eclipse.core.runtime.prefs" <<EOF
eclipse.preferences.version=1
environment/project/IOCOMPOSER_HOME/value=$ROOT
environment/project/ARM_GCC_HOME/value=$ARM_DIR/bin
environment/project/RISCV_GCC_HOME/value=$RISCV_DIR/bin
environment/project/OPENOCD_HOME/value=$OPENOCD_DIR/bin
environment/project/NRFX_HOME/value=$EXT/nrfx
environment/project/NRFXLIB_HOME/value=$EXT/sdk-nrfxlib
environment/project/NRF5_SDK_HOME/value=$EXT/nRF5_SDK
environment/project/NRF5_SDK_MESH_HOME/value=$EXT/nRF5_SDK_Mesh
environment/project/BSEC_HOME/value=$EXT/BSEC
EOF
fi

# ---------------------------------------------------------
# Sync IOsonata + external SDKs (single source of truth)
# ---------------------------------------------------------
echo
echo "📦 Syncing IOsonata + external SDKs..."

CLONE_URL="https://raw.githubusercontent.com/IOsonata/IOsonata/refs/heads/master/Installer/clone_iosonata_sdk_macos.sh"
TMP_CLONE=$(mktemp /tmp/clone_iosonata_sdk_macos.XXXXXX.sh)
CLONE_LOG=$(mktemp /tmp/clone_iosonata_sdk_macos.XXXXXX.log)

if ! curl -fsSL "$CLONE_URL" -o "$TMP_CLONE"; then
  echo "❌ Failed to download clone script:"
  echo "   $CLONE_URL"
  exit 1
fi

chmod +x "$TMP_CLONE"

# Prefer --no-build if supported by the clone script (newer versions). If not supported,
# retry without it. We capture output so we can detect whether the builder ran.
CLONE_ARGS=(--home "$ROOT" --no-build)
if [[ "$MODE" == "force" ]]; then
  CLONE_ARGS+=(--mode force)
fi

run_clone() {
  # shellcheck disable=SC2068
  zsh "$TMP_CLONE" "$@" 2>&1 | tee -a "$CLONE_LOG"
}

if run_clone "${CLONE_ARGS[@]}"; then
  :
else
  echo "⚠️  Clone script does not support --no-build yet. Retrying without it..."
  CLONE_ARGS_NO_BUILD=()
  for a in "${CLONE_ARGS[@]}"; do
    [[ "$a" == "--no-build" ]] && continue
    CLONE_ARGS_NO_BUILD+=("$a")
  done
  run_clone "${CLONE_ARGS_NO_BUILD[@]}"
fi

rm -f "$TMP_CLONE" || true

# Detect whether the clone script already invoked the IOsonata library builder.
CLONE_INVOKED_BUILD=0
if grep -qE "IOsonata Library (Auto-)?Build|IOsonata Library Builder|Available IOsonata library projects|Select project to build" "$CLONE_LOG"; then
  CLONE_INVOKED_BUILD=1
fi

rm -f "$CLONE_LOG" || true

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
# Auto-generated by install_iocdevtools_macos.sh $SCRIPT_VERSION
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
# IOCOMPOSER_HOME must be set to your IOcomposer root directory
ifndef IOCOMPOSER_HOME
\$(error IOCOMPOSER_HOME is not set. Please set it to your IOcomposer root directory)
endif

IOSONATA_ROOT = \$(IOCOMPOSER_HOME)/IOsonata
IOSONATA_INCLUDE = \$(IOSONATA_ROOT)/include
IOSONATA_SRC = \$(IOSONATA_ROOT)/src

# ============================================
# ARM-specific Paths
# ============================================
ARM_ROOT = \$(IOSONATA_ROOT)/ARM
ARM_CMSIS = \$(ARM_ROOT)/CMSIS
ARM_CMSIS_INCLUDE = \$(ARM_CMSIS)/Include
ARM_INCLUDE = \$(ARM_ROOT)/include
ARM_SRC = \$(ARM_ROOT)/src
ARM_LDSCRIPT = \$(ARM_ROOT)/ldscript

# Vendor-specific paths
ARM_NORDIC = \$(ARM_ROOT)/Nordic
ARM_NXP = \$(ARM_ROOT)/NXP
ARM_ST = \$(ARM_ROOT)/ST
ARM_MICROCHIP = \$(ARM_ROOT)/Microchip
ARM_RENESAS = \$(ARM_ROOT)/Renesas

# ============================================
# RISC-V-specific Paths
# ============================================
RISCV_ROOT = \$(IOSONATA_ROOT)/RISCV
RISCV_INCLUDE = \$(RISCV_ROOT)/include
RISCV_SRC = \$(RISCV_ROOT)/src
RISCV_LDSCRIPT = \$(RISCV_ROOT)/ldscript

# Vendor-specific paths
RISCV_ESPRESSIF = \$(RISCV_ROOT)/Espressif
RISCV_NORDIC = \$(RISCV_ROOT)/Nordic
RISCV_RENESAS = \$(RISCV_ROOT)/Renesas

# ============================================
# External Libraries
# ============================================
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

# ============================================
# Additional IOsonata Modules
# ============================================
FATFS_ROOT = \$(IOSONATA_ROOT)/fatfs
LITTLEFS_ROOT = \$(IOSONATA_ROOT)/littlefs
MICRO_ECC_ROOT = \$(IOSONATA_ROOT)/micro-ecc

# ============================================
# Common Include Paths (for -I flags)
# ============================================
IOSONATA_INCLUDES = -I\$(IOSONATA_INCLUDE) \\
                    -I\$(IOSONATA_INCLUDE)/bluetooth \\
                    -I\$(IOSONATA_INCLUDE)/audio \\
                    -I\$(IOSONATA_INCLUDE)/converters \\
                    -I\$(IOSONATA_INCLUDE)/coredev \\
                    -I\$(IOSONATA_INCLUDE)/display \\
                    -I\$(IOSONATA_INCLUDE)/imu \\
                    -I\$(IOSONATA_INCLUDE)/miscdev \\
                    -I\$(IOSONATA_INCLUDE)/pwrmgnt \\
                    -I\$(IOSONATA_INCLUDE)/sensors \\
                    -I\$(IOSONATA_INCLUDE)/storage \\
                    -I\$(IOSONATA_INCLUDE)/sys \\
                    -I\$(IOSONATA_INCLUDE)/usb

ARM_INCLUDES = -I\$(ARM_INCLUDE) \\
               -I\$(ARM_CMSIS_INCLUDE)

RISCV_INCLUDES = -I\$(RISCV_INCLUDE)

# ============================================
# Environment Variables (optional)
# ============================================
export ARM_GCC_HOME := $ARM_DIR/bin
export RISCV_GCC_HOME := $RISCV_DIR/bin
export OPENOCD_HOME := $OPENOCD_DIR/bin
export NRFX_HOME := \$(NRFX_ROOT)
export NRFXLIB_HOME := \$(SDK_NRFXLIB_ROOT)
export NRF5_SDK_HOME := \$(NRF5_SDK_ROOT)
export NRF5_SDK_MESH_HOME := \$(NRF5_SDK_MESH_ROOT)
export BSEC_HOME := \$(BSEC_ROOT)
EOF

echo "✅ makefile_path.mk created at: $MAKEFILE_PATH_MK"
echo "   Projects can include it with: include \$(IOSONATA_ROOT)/makefile_path.mk"

# ---------------------------------------------------------
# IOsonata Library Build (run exactly once)
# ---------------------------------------------------------
# If the clone script already invoked the builder, do NOT run it again.
# If it did NOT invoke the builder (e.g., clone supports --no-build), run it here.
if [[ "${CLONE_INVOKED_BUILD:-0}" -eq 0 ]]; then
  BUILD_SCRIPT="$ROOT/IOsonata/Installer/build_iosonata_lib_macos.sh"
  if [[ -f "$BUILD_SCRIPT" ]]; then
    echo ""
    echo "========================================================="
    echo "  IOsonata Library Build"
    echo "========================================================="
    echo ""
    bash "$BUILD_SCRIPT" --home "$ROOT" || true
  else
    echo ""
    echo "ℹ️  IOsonata library builder not found:"
    echo "   $BUILD_SCRIPT"
    echo "   (Skipping library build)"
    echo ""
  fi
else
  echo ""
  echo "ℹ️  IOsonata libraries were already handled by the clone step (skipping rebuild)."
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
if [[ -f "$ECLIPSE_APP/Contents/Info.plist" ]]; then
  ECLIPSE_VER=$(/usr/libexec/PlistBuddy -c "Print :CFBundleShortVersionString" "$ECLIPSE_APP/Contents/Info.plist" 2>/dev/null || echo "Unknown")
fi

if [[ -n "${ARM_DIR:-}" && -x "$ARM_DIR/bin/arm-none-eabi-gcc" ]]; then
  ARM_VER=$("$ARM_DIR/bin/arm-none-eabi-gcc" --version | head -n1 | grep -oE '[0-9]+\.[0-9]+\.[0-9]+' | head -n1)
else
  ARM_VER="Not found"
fi

if [[ -n "${RISCV_DIR:-}" && -x "$RISCV_DIR/bin/riscv-none-elf-gcc" ]]; then
  RISCV_VER=$("$RISCV_DIR/bin/riscv-none-elf-gcc" --version | head -n1 | grep -oE '[0-9]+\.[0-9]+\.[0-9]+' | head -n1)
else
  RISCV_VER="Not found"
fi

if [[ -n "${OPENOCD_DIR:-}" && -x "$OPENOCD_DIR/bin/openocd" ]]; then
#  OPENOCD_VER=$("$OPENOCD_DIR/bin/openocd" --version | head -n1 | grep -oE '[0-9]+\.[0-9]+\.[0-9]+' | head -n1)
  OPENOCD_VER=$("$OPENOCD_DIR/bin/openocd" --version 2>&1 | head -n1 | grep -oE '[0-9]+\.[0-9]+\.[0-9]+' | head -n1)
else
  OPENOCD_VER="Not found"
fi

printf "%-25s %s\n" "Eclipse Embedded CDT:" "$ECLIPSE_VER"
printf "%-25s %s\n" "ARM GCC:" "$ARM_VER"
printf "%-25s %s\n" "RISC-V GCC:" "$RISCV_VER"
printf "%-25s %s\n" "OpenOCD:" "$OPENOCD_VER"
echo ""
printf "%-25s %s\n" "iocomposer_home:" "$ROOT"
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
echo "The variable is available in ALL workspaces."
echo "No macOS reboot required!"
echo "=============================================="
echo

