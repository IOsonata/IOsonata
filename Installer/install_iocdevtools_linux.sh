#!/usr/bin/env bash
set -euo pipefail

SCRIPT_NAME="install_iocdevtools_linux"
SCRIPT_VERSION="v1.0.94"

ROOT="$HOME/IOcomposer"
TOOLS="/opt/xPacks"
BIN="/usr/local/bin"
ECLIPSE_DIR="$HOME/eclipse"

echo "=============================================="
echo "   IOcomposer MCU Dev Tools Installer (Linux) "
echo "   Script: $SCRIPT_NAME"
echo "   Version: $SCRIPT_VERSION"
echo "=============================================="
echo

# ---------------------------------------------------------
# DEPENDENCY CHECK AND INSTALL
# ---------------------------------------------------------
check_and_install_dependencies() {
  local missing_dependencies=()
  
  command -v git &>/dev/null || missing_dependencies+=("git")
  command -v make &>/dev/null || missing_dependencies+=("make")
  
  if [[ ${#missing_dependencies[@]} -eq 0 ]]; then
    echo "✓ All required dependencies are installed"
    return 0
  fi
  
  echo "⚠️  Missing dependencies: ${missing_dependencies[*]}"
  
  if command -v apt-get &>/dev/null; then
    echo ">>> Installing via apt-get..."
    sudo apt-get update && sudo apt-get install -y "${missing_dependencies[@]}"
  elif command -v dnf &>/dev/null; then
    sudo dnf install -y "${missing_dependencies[@]}"
  elif command -v yum &>/dev/null; then
    sudo yum install -y "${missing_dependencies[@]}"
  elif command -v pacman &>/dev/null; then
    sudo pacman -S --noconfirm "${missing_dependencies[@]}"
  elif command -v zypper &>/dev/null; then
    sudo zypper install -y "${missing_dependencies[@]}"
  else
    echo "❌ Could not detect package manager. Please install: ${missing_dependencies[*]}"
    exit 1
  fi
  
  echo "✓ Dependencies installed successfully"
}

check_and_install_dependencies

# ---------------------------------------------------------
# CLI
# ---------------------------------------------------------
show_help() {
  cat <<EOF
Usage: ./$SCRIPT_NAME.sh [OPTION]

Options:
  --help              Show help and exit
  --version           Show version and exit
  --home <path>       Set custom SDK installation root (default: ~/IOcomposer)
  --force-update      Force reinstall
  --uninstall         Remove toolchains + Eclipse (keep repos/workspaces)
  (no option)         Install/update (skip if already installed) <-- Sudo permissions no longer required
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
    --eclipse)
      shift
      if [[ -z "${1:-}" ]]; then
        echo "❌ Missing path after --eclipse"; exit 1
      fi
      ECLIPSE_DIR="$1"
      echo ">>> Using custom Eclipse folder: $ECLIPSE_DIR"
      ;;
    *) echo "Unknown option: $1"; exit 1 ;;
  esac
  shift
done

EXT="$ROOT/external"

# Create directories
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

# Checks if the Content-Length is greater than 1MB (1,000,000 bytes)
is_valid_archive_url() {
  local url="$1"
  local headers
  headers=$(curl -fsSLI --max-time 10 "$url" 2>/dev/null || true)

  local length_line
  length_line=$(echo "$headers" | grep -i '^Content-Length:' | tail -n 1)

  if [[ -z "$length_line" ]]; then
    return 1
  fi

  local length
  length=$(echo "$length_line" | awk '{print $2}' | tr -d '\r')

  if ! [[ "$length" =~ ^[0-9]+$ ]]; then
    return 1
  fi

  if (( length > 1000000 )); then
    return 0
  else
    return 1
  fi
}

# ---------------------------------------------------------
# Cleanup
# ---------------------------------------------------------
TMP_ECLIPSE_ARCHIVE=""
cleanup() {
  if [[ -n "${TMP_ECLIPSE_ARCHIVE:-}" ]]; then rm -f "$TMP_ECLIPSE_ARCHIVE" || true; fi
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
  old_plugins=$(find "$dropins_dir" -name "org.iosonata.embedcdt.templates.firmware_*.jar" 2>/dev/null || true)

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
assets = (j.get("assets") or [])
preferred_exts = (".tar.gz", ".tgz", ".tar.xz", ".txz", ".zip")
for a in assets:
  u = a.get("browser_download_url", "") or ""
  if any(p in u for p in platforms) and any(u.endswith(ext) for ext in preferred_exts):
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

  if [[ -z "${latest_tag:-}" || -z "${latest_url:-}" ]]; then
    echo "❌ Could not resolve latest release tag or download URL for $name (repo=$repo)." >&2
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

  local tmpfile
  tmpfile=$(mktemp)
  if ! curl -fL "$latest_url" -o "$tmpfile"; then
    echo "❌ Failed to download $name from: $latest_url" >&2
    rm -f "$tmpfile" || true
    exit 1
  fi

  if [[ "$latest_url" == *.zip ]]; then
    sudo unzip -q "$tmpfile" -d "$TOOLS"
  elif [[ "$latest_url" == *.tar.xz || "$latest_url" == *.txz ]]; then
    sudo tar -xJf "$tmpfile" -C "$TOOLS"
  else
    sudo tar -xzf "$tmpfile" -C "$TOOLS"
  fi
  rm -f "$tmpfile"

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

  RELEASES=$(curl -fsSL "$MIRROR/" | grep -oE 'href="20[0-9]{2}-[0-9]{2}/"' | cut -d'"' -f2 | sed 's|/||g' | sort -r | uniq || true)
  if [[ -z "${RELEASES:-}" ]]; then
    echo "❌ Failed to enumerate Eclipse releases from mirror: $MIRROR" >&2
    exit 1
  fi

  ECLIPSE_URL=""
  for release in $RELEASES; do
    URL_EMBEDCDT="$MIRROR/$release/R/eclipse-embedcdt-$release-R-linux-gtk-$ECLIPSE_ARCH.tar.gz"
    URL_EMBEDCPP="$MIRROR/$release/R/eclipse-embedcpp-$release-R-linux-gtk-$ECLIPSE_ARCH.tar.gz"

    if is_valid_archive_url "$URL_EMBEDCDT"; then
      ECLIPSE_URL="$URL_EMBEDCDT"
      break
    elif is_valid_archive_url "$URL_EMBEDCPP"; then
      ECLIPSE_URL="$URL_EMBEDCPP"
      break
    fi
  done

  if [[ -z "$ECLIPSE_URL" ]]; then
    echo "❌ Could not find a valid Eclipse Embedded CDT download URL for any release." >&2
    exit 1
  fi

  echo "⬇️ Downloading Eclipse: $ECLIPSE_URL"

  TMP_ECLIPSE_ARCHIVE=$(mktemp)
  if ! curl -fL "$ECLIPSE_URL" -o "$TMP_ECLIPSE_ARCHIVE"; then
    echo "❌ Eclipse download failed." >&2
    exit 1
  fi

  # Extract to temp dir first, then move (safer approach)
  TMP_EXTRACT=$(mktemp -d)
  tar -xzf "$TMP_ECLIPSE_ARCHIVE" -C "$TMP_EXTRACT"

  sudo rm -rf "$ECLIPSE_DIR"
  sudo mv "$TMP_EXTRACT/eclipse" "$ECLIPSE_DIR"
  rm -rf "$TMP_EXTRACT"
  rm -f "$TMP_ECLIPSE_ARCHIVE"
  TMP_ECLIPSE_ARCHIVE=""

  echo "✅ Eclipse installed at $ECLIPSE_DIR"
fi

# ---------------------------------------------------------
# Seed preferences to Eclipse INSTALLATION directory
# (Works immediately without needing to run Eclipse first)
# ---------------------------------------------------------
seed_eclipse_install_prefs() {
  echo
  echo ">>> Seeding Eclipse MCU preferences in installation directory..."

  local install_cfg="$ECLIPSE_DIR/configuration/.settings"
  sudo mkdir -p "$install_cfg"

  # Hash calculation - use toolchain NAME strings like macOS (not paths)
  local ARM_HASH RISCV_HASH
  ARM_HASH=$(java_hash "xPack GNU Arm Embedded GCC")
  if (( ARM_HASH < 0 )); then ARM_HASH=$((ARM_HASH + 4294967296)); fi
  
  RISCV_HASH=$(java_hash "xPack GNU RISC-V Embedded GCC")
  if (( RISCV_HASH < 0 )); then RISCV_HASH=$((RISCV_HASH + 4294967296)); fi
  RISCV_HASH=$((RISCV_HASH + 1))  # +1 matches macOS behavior

  # 1. org.eclipse.core.runtime.prefs (with environment variables like macOS)
  sudo tee "$install_cfg/org.eclipse.core.runtime.prefs" > /dev/null <<EOF
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

  # 2. org.eclipse.cdt.core.prefs
  sudo tee "$install_cfg/org.eclipse.cdt.core.prefs" > /dev/null <<EOF
eclipse.preferences.version=1
environment/buildEnvironmentInclude=true
org.eclipse.cdt.core.parser.taskTags=TODO,FIXME,XXX
EOF

  # 3. org.eclipse.embedcdt.core.prefs (xPack paths - matching macOS format)
  sudo tee "$install_cfg/org.eclipse.embedcdt.core.prefs" > /dev/null <<EOF
eclipse.preferences.version=1
xpack.arm.toolchain.path=$ARM_DIR/bin
xpack.riscv.toolchain.path=$RISCV_DIR/bin
xpack.openocd.path=$OPENOCD_DIR/bin
xpack.strict=true
EOF

  # 4. org.eclipse.embedcdt.managedbuild.cross.arm.core.prefs
  sudo tee "$install_cfg/org.eclipse.embedcdt.managedbuild.cross.arm.core.prefs" > /dev/null <<EOF
toolchain.path.$ARM_HASH=$ARM_DIR/bin
toolchain.path.1287942917=$ARM_DIR/bin
toolchain.path.strict=true
EOF

  # 5. org.eclipse.embedcdt.managedbuild.cross.riscv.core.prefs
  sudo tee "$install_cfg/org.eclipse.embedcdt.managedbuild.cross.riscv.core.prefs" > /dev/null <<EOF
toolchain.path.$RISCV_HASH=$RISCV_DIR/bin
toolchain.path.strict=true
EOF

  # 6. org.eclipse.embedcdt.debug.gdbjtag.openocd.core.prefs
  sudo tee "$install_cfg/org.eclipse.embedcdt.debug.gdbjtag.openocd.core.prefs" > /dev/null <<EOF
install.folder=$OPENOCD_DIR/bin
install.folder.strict=true
EOF

  echo "✅ Eclipse MCU preferences seeded in:"
  echo "   $install_cfg"
}

seed_eclipse_install_prefs

# ---------------------------------------------------------
# Seed preferences to user directory (~/.eclipse)
# (Only works if Eclipse has been run at least once)
# ---------------------------------------------------------
seed_eclipse_user_prefs() {
  echo
  echo ">>> Checking for user Eclipse preferences in ~/.eclipse..."

  local base="$HOME/.eclipse"
  mkdir -p "$base"

  # If a previous run created ~/.eclipse as root, fix it.
  local base_uid
  base_uid=$(stat -c '%u' "$base" 2>/dev/null || stat -f '%u' "$base" 2>/dev/null || echo "")
  if [[ "$base_uid" == "0" ]]; then
    echo "⚠️  $base is owned by root (likely from a previous installer run). Fixing ownership..."
    sudo chown -R "$USER":$(id -gn) "$base" || true
  fi

  # Initialize Eclipse to create user configuration directory (like macOS)
  echo "⏳ Initializing Eclipse to create instance configuration..."
  if [[ -x "$ECLIPSE_DIR/eclipse" ]]; then
    "$ECLIPSE_DIR/eclipse" -nosplash -initialize 2>/dev/null || true
  fi

  local instance_cfg
  instance_cfg=$(ls -d "$base"/org.eclipse.platform_*/configuration 2>/dev/null | sort -r | head -n1 || true)

  if [[ -z "${instance_cfg:-}" ]]; then
    echo "   User config directory not found (Eclipse not yet run). Skipping user prefs."
    echo "   Installation-level prefs are already set above."
    return 0
  fi

  echo "📂 Found Eclipse settings: $instance_cfg"

  mkdir -p "$instance_cfg/.settings"

  if [[ ! -w "$instance_cfg/.settings" ]]; then
    echo "⚠️  $instance_cfg/.settings is not writable. Fixing ownership..."
    sudo chown -R "$USER":$(id -gn) "$instance_cfg" || true
  fi

  # Check for Java (informational, not fatal on Linux)
  local JRE=""
  if command -v java >/dev/null 2>&1; then
    JRE=$(dirname "$(dirname "$(readlink -f "$(command -v java)")")" 2>/dev/null || true)
  fi
  if [[ -z "$JRE" ]]; then
    echo "⚠️  No JDK found in PATH. Eclipse requires Java 17+. Please install a JDK."
  fi

  # Hash calculation - use toolchain NAME strings like macOS (not paths)
  local ARM_HASH RISCV_HASH
  ARM_HASH=$(java_hash "xPack GNU Arm Embedded GCC")
  if (( ARM_HASH < 0 )); then ARM_HASH=$((ARM_HASH + 4294967296)); fi
  
  RISCV_HASH=$(java_hash "xPack GNU RISC-V Embedded GCC")
  if (( RISCV_HASH < 0 )); then RISCV_HASH=$((RISCV_HASH + 4294967296)); fi
  RISCV_HASH=$((RISCV_HASH + 1))  # +1 matches macOS behavior

  # Only 3 files for user directory (same as macOS)

  # 1. org.eclipse.embedcdt.managedbuild.cross.arm.core.prefs
  cat > "$instance_cfg/.settings/org.eclipse.embedcdt.managedbuild.cross.arm.core.prefs" <<EOF
toolchain.path.$ARM_HASH=$ARM_DIR/bin
toolchain.path.1287942917=$ARM_DIR/bin
toolchain.path.strict=true
EOF

  # 2. org.eclipse.embedcdt.managedbuild.cross.riscv.core.prefs
  cat > "$instance_cfg/.settings/org.eclipse.embedcdt.managedbuild.cross.riscv.core.prefs" <<EOF
toolchain.path.$RISCV_HASH=$RISCV_DIR/bin
toolchain.path.strict=true
EOF

  # 3. org.eclipse.embedcdt.debug.gdbjtag.openocd.core.prefs
  cat > "$instance_cfg/.settings/org.eclipse.embedcdt.debug.gdbjtag.openocd.core.prefs" <<EOF
install.folder=$OPENOCD_DIR/bin
install.folder.strict=true
EOF

  echo "✅ Eclipse user preferences seeded in:"
  echo "   $instance_cfg/.settings"
}

seed_eclipse_user_prefs

# ---------------------------------------------------------
# Set global iosonata_loc and iocomposer_home system property in eclipse.ini
# ---------------------------------------------------------
echo
echo ">>> Setting iosonata_loc system property in Eclipse installation..."

ECLIPSE_INI="$ECLIPSE_DIR/eclipse.ini"
IOSONATA_LOC_PROP="-Diosonata_loc=$ROOT"
IOCOMPOSER_HOME_PROP="-Diocomposer_home=$ROOT"

# Backup before modifying
sudo cp -f "$ECLIPSE_INI" "$ECLIPSE_INI.bak"

#sudo python3 - "$ECLIPSE_INI" "$IOSONATA_LOC_PROP" <<'PY'
#import sys
#ini = sys.argv[1]; prop = sys.argv[2].strip()
#with open(ini, "r") as f: lines = f.read().splitlines()
#lines = [ln for ln in lines if not ln.startswith("-Diosonata_loc=") and not ln.startswith("-Diosonata.home=")]
#out = []; inserted = False
#for ln in lines:
#    out.append(ln)
#    if ln.strip() == "-vmargs" and not inserted:
#        out.append(prop); inserted = True
#if not inserted: out.append(prop)
#with open(ini, "w") as f: f.write("\n".join(out) + "\n")
#PY

# Remove old properties if they exist (GNU sed syntax for Linux)
sudo sed -i '/^-Diosonata\.home=/d' "$ECLIPSE_INI"
sudo sed -i '/^-Diosonata_loc=/d' "$ECLIPSE_INI"
sudo sed -i '/^-Diocomposer_home=/d' "$ECLIPSE_INI"

# Find the -vmargs line and insert after it
# If no -vmargs, create it first
if grep -q "^-vmargs" "$ECLIPSE_INI"; then
    # Insert after -vmargs line (GNU sed syntax)
    sudo sed -i "/^-vmargs$/a -Diosonata_loc=$ROOT" "$ECLIPSE_INI"
    sudo sed -i "/^-Diosonata_loc=/a -Diocomposer_home=$ROOT" "$ECLIPSE_INI"
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

#echo "✅ iosonata_loc set in eclipse.ini:"
#grep "^-Diosonata_loc=" "$ECLIPSE_INI" || true


# ---------------------------------------------------------
# Clone repos
# ---------------------------------------------------------

# Helper function for updating shallow clones
update_shallow_repo() {
  local dir="$1"
  local name=$(basename "$dir")
  echo "   Updating $name..."
  
  pushd "$dir" > /dev/null
  
  local branch
  branch=$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo "master")
  
  # For shallow clones, fetch + reset is more reliable than pull
  if git fetch --depth=1 origin "$branch" 2>/dev/null; then
    git reset --hard "origin/$branch" 2>/dev/null || git pull --ff-only 2>/dev/null || true
  else
    # Fallback to regular pull
    git pull --ff-only 2>/dev/null || git pull 2>/dev/null || true
  fi
  
  popd > /dev/null
}

if [[ -d "$ROOT/IOsonata" ]]; then
  if [[ "$MODE" == "force" ]]; then
    rm -rf "$ROOT/IOsonata"
    echo "   Cloning IOsonata..."
    git clone --depth=1 https://github.com/IOsonata/IOsonata.git "$ROOT/IOsonata"
  else
    update_shallow_repo "$ROOT/IOsonata"
  fi
else
  echo "   Cloning IOsonata..."
  git clone --depth=1 https://github.com/IOsonata/IOsonata.git "$ROOT/IOsonata"
fi

if [[ -d "$ROOT/IOsonata/Installer" ]]; then
  chmod +x "$ROOT/IOsonata/Installer/"*.sh 2>/dev/null || true
fi

echo ">>> Fixing Eclipse project paths for Linux compatibility..."

# Fix nrfx MDK path (nrfx repo restructured: mdk moved to bsp/stable/mdk)
find "$ROOT/IOsonata" -name ".cproject" -exec sed -i 's|nrfx/mdk|nrfx/bsp/stable/mdk|g' {} \; 2>/dev/null || true

# Fix Fusion case sensitivity (Linux filesystems are case-sensitive)
find "$ROOT/IOsonata" -name ".cproject" -exec sed -i 's|external/fusion|external/Fusion|g' {} \; 2>/dev/null || true

echo "✅ Eclipse project paths fixed for Linux"

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
      echo "   Cloning $name..."
      git clone --depth=1 "$repo" "$name"
    else
      update_shallow_repo "$name"
    fi
  else
    echo "   Cloning $name..."
    git clone --depth=1 "$repo" "$name"
  fi
done

echo "📦 Cloning FreeRTOS-Kernel..."
if [[ -d "FreeRTOS-Kernel" ]]; then
  if [[ "$MODE" == "force" ]]; then
    rm -rf "FreeRTOS-Kernel"
    git clone --depth=1 https://github.com/FreeRTOS/FreeRTOS-Kernel.git FreeRTOS-Kernel
  else
    update_shallow_repo "FreeRTOS-Kernel"
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
VQF_ROOT = \$(EXTERNAL_ROOT)/vqf
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
  chmod +x "$BUILD_SCRIPT"
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
  ECLIPSE_VER=$(grep '^version=' "$ECLIPSE_DIR/.eclipseproduct" 2>/dev/null | cut -d= -f2 || echo "Unknown")
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

echo ""
echo "┌──────────────────────────┬────────────────┬──────────────────────────────────┐"
echo "│ Component                │ Version        │ Location                         │"
echo "├──────────────────────────┼────────────────┼──────────────────────────────────┤"
printf "│ %-24s │ %-14s │ %-32s │\n" "Eclipse Embedded CDT" "$ECLIPSE_VER" "$ECLIPSE_DIR"
printf "│ %-24s │ %-14s │ %-32s │\n" "ARM GCC" "$ARM_VER" "${ARM_DIR:-N/A}"
printf "│ %-24s │ %-14s │ %-32s │\n" "RISC-V GCC" "$RISCV_VER" "${RISCV_DIR:-N/A}"
printf "│ %-24s │ %-14s │ %-32s │\n" "OpenOCD" "$OPENOCD_VER" "${OPENOCD_DIR:-N/A}"
echo "├──────────────────────────┴────────────────┴──────────────────────────────────┤"
printf "│ %-24s   %-49s │\n" "IOcomposer Home:" "$ROOT"
printf "│ %-24s   %-49s │\n" "xPacks Directory:" "$TOOLS"
printf "│ %-24s   %-49s │\n" "Symlinks Directory:" "$BIN"
echo "└─────────────────────────────────────────────────────────────────────────────┘"
echo ""
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
