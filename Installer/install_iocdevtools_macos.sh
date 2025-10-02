#!/usr/bin/env bash
set -euo pipefail

SCRIPT_NAME="install_iocdevtools_macos"
SCRIPT_VERSION="v1.0.46"

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
mkdir -p "$ROOT" "$EXT"; sudo mkdir -p "$TOOLS" "$BIN"

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

  echo ">>> Repositories under $ROOT and workspace dirs were kept."
  echo ">>> Uninstall complete!"
  exit 0
fi

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
    installed_ver=$($tool --version | head -n1 | grep -oE '[0-9]+\.[0-9]+\.[0-9]+')
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
# Install Eclipse Embedded CDT
# ---------------------------------------------------------
echo
echo "💻 Installing Eclipse Embedded CDT IDE..."
MIRROR="https://ftp2.osuosl.org/pub/eclipse/technology/epp/downloads/release"
LATEST=$(curl -s "$MIRROR/" | grep -oE '20[0-9]{2}-[0-9]{2}' | sort -r | head -1)
[ -z "$LATEST" ] && { echo "❌ Could not detect Eclipse release"; exit 1; }

ARCH="x86_64"; if [ "$(uname -m)" = "arm64" ]; then ARCH="aarch64"; fi
ECLIPSE_URL="$MIRROR/$LATEST/R/eclipse-embedcdt-$LATEST-R-macosx-cocoa-$ARCH.dmg"
if ! curl --head --silent --fail "$ECLIPSE_URL" >/dev/null; then
  ECLIPSE_URL="$MIRROR/$LATEST/R/eclipse-embedcpp-$LATEST-R-macosx-cocoa-$ARCH.dmg"
fi

echo "⬇️ Downloading Eclipse: $ECLIPSE_URL"
tmpdmg=$(mktemp)
curl -L "$ECLIPSE_URL" -o "$tmpdmg"
MNT=$(mktemp -d /tmp/eclipse.XXXX)
hdiutil attach "$tmpdmg" -mountpoint "$MNT" -nobrowse -quiet
rm -rf "$ECLIPSE_APP"
cp -R "$MNT/Eclipse.app" "$ECLIPSE_APP"
hdiutil detach "$MNT" -quiet
rm -rf "$tmpdmg" "$MNT"
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
# Clone repos
# ---------------------------------------------------------
if [[ -d "$ROOT/IOsonata" ]]; then
  if [[ "$MODE" == "force" ]]; then rm -rf "$ROOT/IOsonata"; git clone --depth=1 https://github.com/IOsonata/IOsonata.git "$ROOT/IOsonata"
  else (cd "$ROOT/IOsonata" && git pull); fi
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
  "https://github.com/BoschSensortec/BSEC-Arduino-library.git"
)
for repo in "${repos[@]}"; do
  name=$(basename "$repo" .git)
  if [[ "$name" == "BSEC-Arduino-library" ]]; then name="BSEC"; fi
  if [[ -d "$name" ]]; then
    if [[ "$MODE" == "force" ]]; then rm -rf "$name"; git clone --depth=1 "$repo" "$name"
    else (cd "$name" && git pull); fi
  else
    git clone --depth=1 "$repo" "$name"
  fi
done

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
  OPENOCD_VER=$("$OPENOCD_DIR/bin/openocd" --version | head -n1 | grep -oE '[0-9]+\.[0-9]+\.[0-9]+' | head -n1)
else
  OPENOCD_VER="Not found"
fi

printf "%-25s %s\n" "Eclipse Embedded CDT:" "$ECLIPSE_VER"
printf "%-25s %s\n" "ARM GCC:" "$ARM_VER"
printf "%-25s %s\n" "RISC-V GCC:" "$RISCV_VER"
printf "%-25s %s\n" "OpenOCD:" "$OPENOCD_VER"

echo "=============================================="
echo " Installation complete!"
echo "=============================================="
