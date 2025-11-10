#!/usr/bin/env bash
set -euo pipefail

SCRIPT_NAME="install_iocdevtools_linux"
SCRIPT_VERSION="v1.0.8"

ROOT="$HOME/IOcomposer"
TOOLS="/opt/xPacks"
BIN="/usr/local/bin"
ECLIPSE_DIR="/opt/eclipse"

echo "=============================================="
echo "  IOcomposer Dev Tools Installer (Linux) "
echo "  Script: $SCRIPT_NAME"
echo "  Version: $SCRIPT_VERSION"
echo "=============================================="
echo

# ---------------------------------------------------------
# CLI
# ---------------------------------------------------------
show_help() {
  cat <<EOF
Usage: $SCRIPT_NAME [OPTION]

Options:
  --help           Show help and exit
  --version        Show version and exit
  --home <path>    Set custom IOcomposer root (default: ~/IOcomposer)
  --force-update   Force reinstallation
  --uninstall      Remove toolchains + Eclipse (keep repos/workspaces)
EOF
}

MODE="install"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --help) show_help; exit 0 ;;
    --version) echo "$SCRIPT_NAME $SCRIPT_VERSION"; exit 0 ;;
    --home) ROOT="$2"; shift 2 ;;
    --force-update) MODE="force"; shift ;;
    --uninstall) MODE="uninstall"; shift ;;
    *) echo "Unknown option: $1"; show_help; exit 1 ;;
  esac
done

# EXT must follow ROOT
EXT="$ROOT/external"

# ---------------------------------------------------------
# Prerequisites
# ---------------------------------------------------------
install_prerequisites() {
  echo ">>> Installing prerequisites..."
  if command -v apt &>/dev/null; then
    sudo apt update
    sudo apt install -y wget curl tar unzip openjdk-17-jdk build-essential git
  elif command -v dnf &>/dev/null; then
    sudo dnf install -y wget curl tar unzip java-17-openjdk gcc make git
  elif command -v yum &>/dev/null; then
    sudo yum install -y wget curl tar unzip java-17-openjdk gcc make git
  elif command -v pacman &>/dev/null; then
    sudo pacman -Syu --noconfirm wget curl tar unzip jdk17-openjdk base-devel git
  else
    echo "Unsupported package manager. Please install wget, curl, tar, unzip, Java 17+, gcc, make, git manually."
  fi
}

# ---------------------------------------------------------
# Java String.hashCode in Bash
# ---------------------------------------------------------
java_hash() {
  local s="$1"
  local h=0
  local c
  for (( i=0; i<${#s}; i++ )); do
    c=$(printf "%d" "'${s:$i:1}")
    h=$(( (h * 31 + c) & 0xFFFFFFFF ))
  done
  if (( h > 2147483647 )); then
    h=$((h - 4294967296))
  fi
  echo $h
}

# ---------------------------------------------------------
# Install xPack Toolchain (arch aware)
# ---------------------------------------------------------
install_xpack() {
  local repo="$1"
  local label="$2"

  echo ">>> Installing $label ..."
  sudo mkdir -p "$TOOLS"
  sudo chown -R "$USER":"$USER" "$TOOLS"

  # Detect architecture
  local arch
  case "$(uname -m)" in
    x86_64) arch="linux-x64" ;;
    aarch64|arm64) arch="linux-arm64" ;;
    *) echo "!!! WARNING: unknown arch $(uname -m), defaulting to linux-x64"
       arch="linux-x64" ;;
  esac

  local latest_json
  latest_json=$(curl -s "https://api.github.com/repos/xpack-dev-tools/${repo}/releases/latest")

  local url
  url=$(echo "$latest_json" | grep "browser_download_url" | grep "${arch}.tar.gz" | cut -d '"' -f 4 | head -n1)

  if [[ -z "$url" ]]; then
    echo "!!! WARNING: no ${arch} release found for $repo, skipping."
    return
  fi

  local archive
  archive=$(basename "$url")
  local dest="$TOOLS/${archive%-linux-x64.tar.gz}"

  if [[ ! -d "$dest" || "$MODE" = "force" ]]; then
    echo "    → Downloading $url ..."
    wget -q "$url" -O "/tmp/$archive"
    tar -xf "/tmp/$archive" -C "$TOOLS"
  fi

  local binpath
  binpath=$(find "$TOOLS" -maxdepth 2 -type d -name "bin" | grep "${repo%-xpack}" | head -n1 || true)
  if [[ -n "$binpath" ]]; then
    for exe in "$binpath"/*; do
      sudo ln -sf "$exe" "$BIN/$(basename "$exe")"
    done
  fi

   eval "${3}=${dest}"
}

# ---------------------------------------------------------
# Install Eclipse Embedded CDT (latest release train, arch aware)
# ---------------------------------------------------------
install_eclipse_embedded() {
  echo ">>> Installing Eclipse Embedded CDT package ..."
  local MIRROR="https://ftp2.osuosl.org/pub/eclipse/technology/epp/downloads/release"

  # --- START: Modified Section ---
  # Get the top 10 most recent release names
  echo ">>> Finding available Eclipse releases..."
  local RELEASES
  RELEASES=$(curl -s "$MIRROR/" | grep -Eo '20[0-9]{2}-[0-9]{2}' | sort -r | uniq | head -n10)

  if [[ -z "$RELEASES" ]]; then
    echo "!!! ERROR: Could not determine any Eclipse release trains."
    exit 1
  fi

  # Detect architecture
  local arch
  case "$(uname -m)" in
    x86_64) arch="x86_64" ;;
    aarch64|arm64) arch="aarch64" ;;
    *) echo "!!! WARNING: unknown arch $(uname -m), defaulting to x86_64"; arch="x86_64" ;;
  esac

  local url=""
  local release_train=""

  # Loop through releases, newest first, until we find a valid package
  # This logic mirrors the working Windows/macOS scripts
  for release in $RELEASES; do
    echo ">>> Checking release: $release..."

    # --- KEY CHANGE: Use the filenames from the original script & working macOS script ---
    local URL_EMBEDCDT="$MIRROR/$release/R/eclipse-embedcdt-$release-R-linux-gtk-$arch.tar.gz"
    local URL_EMBEDCPP="$MIRROR/$release/R/eclipse-embedcpp-$release-R-linux-gtk-$arch.tar.gz"

    # --- PRIORITY 1: Check for 'eclipse-embedcdt' (no hyphen) ---
    echo "   - Checking for 'eclipse-embedcdt' package..."
    # Use 'curl -s --head --fail' to check if the URL exists (returns 0) without downloading
    if curl -s --head --fail "$URL_EMBEDCDT" > /dev/null; then
        echo "[OK] Found 'eclipse-embedcdt' package for $release."
        url="$URL_EMBEDCDT"
        release_train="$release"
        break # Found it, exit loop
    else
        echo "   - 'eclipse-embedcdt' not found. Checking for 'eclipse-embedcpp'..."
        # --- PRIORITY 2: Check for 'eclipse-embedcpp' ---
        if curl -s --head --fail "$URL_EMBEDCPP" > /dev/null; then
            echo "[OK] Found 'eclipse-embedcpp' package for $release."
            url="$URL_EMBEDCPP"
            release_train="$release"
            break # Found it, exit loop
        else
            echo "   - No valid 'embed' packages found for $release. Trying next..."
        fi
    fi
  done

  # Check if we ever found a URL after looping
  if [[ -z "$url" ]]; then
    echo "!!! ERROR: Could not find 'eclipse-embedcdt' or 'eclipse-embedcpp' in any of the top 10 recent Eclipse releases."
    echo "[INFO] Please check the mirror '$MIRROR' manually."
    exit 1
  fi
  # --- END: Modified Section ---


  local archive
  archive=$(basename "$url")

  if [[ ! -d "$ECLIPSE_DIR" || "$MODE" = "force" ]]; then
    echo "    → Downloading $url ..."
    wget -q "$url" -O "/tmp/$archive"

    if [[ "$MODE" = "force" && -d "$ECLIPSE_DIR" ]]; then
      sudo rm -rf "$ECLIPSE_DIR"
    fi

    sudo tar -xf "/tmp/$archive" -C /opt
  fi

  sudo ln -sf "$ECLIPSE_DIR/eclipse" "$BIN/eclipse"
}

# ---------------------------------------------------------
# Configure Eclipse Preferences (with hash-based keys)
# ---------------------------------------------------------
configure_eclipse_prefs() {
  echo ">>> Writing Eclipse Embedded CDT default preferences ..."

  local prefs_root="$ECLIPSE_DIR/configuration/.settings"
  sudo mkdir -p "$prefs_root"

  # Compute hashes
  ARM_HASH=$(java_hash "xPack GNU Arm Embedded GCC")
  (( ARM_HASH < 0 )) && ARM_HASH=$((ARM_HASH + 4294967296))

  RISCV_HASH=$(java_hash "xPack GNU RISC-V Embedded GCC")
  (( RISCV_HASH < 0 )) && RISCV_HASH=$((RISCV_HASH + 4294967296))
  RISCV_HASH=$((RISCV_HASH + 1))

  # ARM toolchain
  sudo tee "$prefs_root/org.eclipse.embedcdt.managedbuild.cross.arm.core.prefs" >/dev/null <<EOF
eclipse.preferences.version=1
toolchain.path.$ARM_HASH=$ARM_DIR/bin
EOF

  # RISC-V toolchain
  sudo tee "$prefs_root/org.eclipse.embedcdt.managedbuild.cross.riscv.core.prefs" >/dev/null <<EOF
eclipse.preferences.version=1
toolchain.path.$RISCV_HASH=$RISCV_DIR/bin
EOF

  # OpenOCD
  sudo tee "$prefs_root/org.eclipse.embedcdt.debug.gdbjtag.openocd.core.prefs" >/dev/null <<EOF
eclipse.preferences.version=1
install.folder=$OPENOCD_DIR/bin
EOF

  # Environment macros
  sudo tee "$prefs_root/org.eclipse.core.runtime.prefs" >/dev/null <<EOF
# Repo paths
environment/project/io.github.embedded.tools/environment/IOSONATA/value=$ROOT/IOsonata
environment/project/io.github.embedded.tools/environment/NRFX/value=$EXT/nrfx
environment/project/io.github.embedded.tools/environment/SDK_NRF_BM/value=$EXT/sdk-nrf-bm
environment/project/io.github.embedded.tools/environment/SDK_NRFXLIB/value=$EXT/sdk-nrfxlib
environment/project/io.github.embedded.tools/environment/NRF5_SDK/value=$EXT/nRF5_SDK
environment/project/io.github.embedded.tools/environment/NRF5_SDK_MESH/value=$EXT/nRF5_SDK_Mesh
environment/project/io.github.embedded.tools/environment/BSEC/value=$EXT/BSEC

# Toolchains + global paths
environment/project/io.github.embedded.tools/environment/ARM_GCC/value=$ARM_DIR
environment/project/io.github.embedded.tools/environment/RISCV_GCC/value=$RISCV_DIR
environment/project/io.github.embedded.tools/environment/OPENOCD/value=$OPENOCD_DIR
environment/project/io.github.embedded.tools/environment/TOOLS/value=$TOOLS
environment/project/io.github.embedded.tools/environment/WORKSPACE/value=$ROOT
environment/project/io.github.embedded.tools/environment/EXT_LIBS/value=$EXT
EOF

echo "Eclipse preferences seeded (ARM, RISC-V, OpenOCD, macros)."

# Step 1: Ensure Eclipse has initialized its instance folder
if [ -d "$ECLIPSE_DIR" ]; then
  echo "Initializing Eclipse to create instance configuration..."
  "$ECLIPSE_DIR/eclipse" -nosplash -initialize || true
fi

# Step 2: Find instance settings folder dynamically
INSTANCE_CFG=$(find "$HOME/.eclipse" -type d -path "*/configuration" | head -n 1)

if [ -z "$INSTANCE_CFG" ]; then
  echo "Could not find Eclipse instance configuration folder."
  echo "Eclipse may not have been started yet."
else
  echo "Found Eclipse settings: $INSTANCE_CFG"

  mkdir -p "$INSTANCE_CFG/.settings"
  
  tee > "$INSTANCE_CFG/.settings/org.eclipse.embedcdt.managedbuild.cross.arm.core.prefs" <<EOF
toolchain.path.$ARM_HASH=$ARM_DIR/bin
toolchain.path.strict=true
EOF

  tee > "$INSTANCE_CFG/.settings/org.eclipse.embedcdt.managedbuild.cross.riscv.core.prefs" <<EOF
toolchain.path.$RISCV_HASH=$RISCV_DIR/bin
toolchain.path.strict=true
EOF

  tee > "$INSTANCE_CFG/.settings/org.eclipse.embedcdt.debug.gdbjtag.openocd.core.prefs" <<EOF
install.folder=$OPENOCD_DIR/bin
install.folder.strict=true
EOF

fi

}

# ---------------------------------------------------------
# Clone repositories
# ---------------------------------------------------------
clone_repos() {
  echo ">>> Cloning/updating IOcomposer repositories ..."
  mkdir -p "$ROOT"

  # IOsonata
  if [[ -d "$ROOT/IOsonata" ]]; then
    if [[ "$MODE" == "force" ]]; then rm -rf "$ROOT/IOsonata"; git clone --depth=1 https://github.com/IOsonata/IOsonata.git "$ROOT/IOsonata"
    else (cd "$ROOT/IOsonata" && git pull); fi
  else
    git clone --depth=1 https://github.com/IOsonata/IOsonata.git "$ROOT/IOsonata"
  fi

  # External repos
  mkdir -p "$EXT"
  cd "$EXT"
  repos=(
    "https://github.com/NordicSemiconductor/nrfx.git"
    "https://github.com/nrfconnect/sdk-nrf-bm.git"
    "https://github.com/nrfconnect/sdk-nrfxlib.git"
    "https://github.com/IOsonata/nRF5_SDK.git"
    "https://github.com/IOsonata/nRF5_SDK_Mesh.git"
    "https://github.com/boschsensortec/Bosch-BSEC2-Library.git"
    "https://github.com/xioTechnologies/Fusion.git"
    "https://github.com/lvgl/lvgl.git"
  )
  for repo in "${repos[@]}"; do
    name=$(basename "$repo" .git)
    [[ "$name" == "Bosch-BSEC2-Library" ]] && name="BSEC"
    if [[ -d "$name" ]]; then
      if [[ "$MODE" == "force" ]]; then rm -rf "$name"; git clone --depth=1 "$repo" "$name"
      else (cd "$name" && git pull); fi
    else
      git clone --depth=1 "$repo" "$name"
    fi
  done
}

# ---------------------------------------------------------
# Display installation report
# ---------------------------------------------------------
display_report() {
  echo
  echo "=============================================="
  echo "  IOcomposer Dev Tools Installation Report"
  echo "=============================================="
  echo "Script version     : $SCRIPT_VERSION"
  echo "Install mode       : $MODE"
  echo
  echo "Root directory     : $ROOT"
  echo "External repos dir : $EXT"
  echo
  echo "Eclipse IDE        : $ECLIPSE_DIR"
  if command -v eclipse &>/dev/null; then
    echo "  Version          : $(eclipse -version 2>&1 | head -n1)"
    echo "  Symlink          : $BIN/eclipse"
  else
    echo "  Not installed"
  fi
  echo
  echo "Toolchains (xPack):"
  if command -v arm-none-eabi-gcc &>/dev/null; then
    echo "  ARM GCC          : ${ARM_DIR:-unknown} ($(arm-none-eabi-gcc --version | head -n1))"
  else
    echo "  ARM GCC          : not installed"
  fi
  if command -v riscv-none-elf-gcc &>/dev/null; then
    echo "  RISC-V GCC       : ${RISCV_DIR:-unknown} ($(riscv-none-elf-gcc --version | head -n1))"
  else
    echo "  RISC-V GCC       : not installed"
  fi
  if command -v openocd &>/dev/null; then
    echo "  OpenOCD          : ${OPENOCD_DIR:-unknown} ($(openocd --version | head -n1))"
  else
    echo "  OpenOCD          : not installed"
  fi
  echo "  Symlinks         : /usr/local/bin/*"
  echo
  echo "Repositories cloned:"
  echo "  IOsonata         : $ROOT/IOsonata"
  echo "  nrfx             : $EXT/nrfx"
  echo "  sdk-nrf-bm       : $EXT/sdk-nrf-bm"
  echo "  sdk-nrfxlib      : $EXT/sdk-nrfxlib"
  echo "  nRF5_SDK         : $EXT/nRF5_SDK"
  echo "  nRF5_SDK_Mesh    : $EXT/nRF5_SDK_Mesh"
  echo "  BSEC             : $EXT/BSEC"
  echo
  echo "=============================================="
  echo ">>> Installation complete. You can start Eclipse by running: eclipse"
  echo "=============================================="
}

# ---------------------------------------------------------
# Uninstall
# ---------------------------------------------------------
uninstall_all() {
  echo ">>> Uninstalling IOcomposer Dev Tools ..."
  sudo rm -rf "$TOOLS"
  sudo rm -rf "$ECLIPSE_DIR"
  sudo rm -f "$BIN/arm-none-eabi-gcc" "$BIN/riscv-none-elf-gcc" "$BIN/openocd" "$BIN/eclipse"
  echo ">>> Uninstall completed."
}

# ---------------------------------------------------------
# Main
# ---------------------------------------------------------
case "$MODE" in
  install|force)
    install_prerequisites
    install_xpack "arm-none-eabi-gcc-xpack" "GNU Arm Embedded GCC" ret_value
    ARM_DIR=$ret_value
    install_xpack "riscv-none-elf-gcc-xpack" "GNU RISC-V Embedded GCC" ret_value
    RISCV_DIR=$ret_value
    install_xpack "openocd-xpack" "OpenOCD" ret_value
    OPENOCD_DIR=$ret_value

    install_eclipse_embedded
    configure_eclipse_prefs
    clone_repos
    display_report
    ;;
  uninstall)
    uninstall_all
    ;;
esac
