#!/usr/bin/env bash
set -euo pipefail

SCRIPT_NAME="install_iocdevtools_linux"
SCRIPT_VERSION="v1.0.12"

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

# ---------------------------------------------------------
# Configure eclipse.ini with System Properties
# ---------------------------------------------------------
echo
echo ">>> Configuring IOsonata and IOcomposer system properties in eclipse.ini..."

ECLIPSE_INI="$ECLIPSE_DIR/eclipse.ini"

# Remove old properties if they exist
sudo sed -i.bak '/^-Diosonata\.home=/d' "$ECLIPSE_INI" 2>/dev/null || true
sudo sed -i '' '/^-Diosonata_loc=/d' "$ECLIPSE_INI" 2>/dev/null || true
sudo sed -i '' '/^-Diocomposer_home=/d' "$ECLIPSE_INI" 2>/dev/null || true

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
    "https://github.com/lwip-tcpip/lwip.git"
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
  
  # Clone FreeRTOS-Kernel
  echo ">>> Cloning FreeRTOS-Kernel..."
  if [[ -d "FreeRTOS-Kernel" ]]; then
    if [[ "$MODE" == "force" ]]; then
      rm -rf "FreeRTOS-Kernel"
      git clone --depth=1 https://github.com/FreeRTOS/FreeRTOS-Kernel.git FreeRTOS-Kernel
    else
      (cd FreeRTOS-Kernel && git pull)
    fi
  else
    git clone --depth=1 https://github.com/FreeRTOS/FreeRTOS-Kernel.git FreeRTOS-Kernel
    echo "✅ FreeRTOS-Kernel cloned"
  fi
}

# ---------------------------------------------------------
# Generate makefile_path.mk
# ---------------------------------------------------------
generate_makefile_paths() {
  echo
  echo ">>> Generating makefile_path.mk for Makefile-based builds..."
  
  local MAKEFILE_PATH_MK="$ROOT/IOsonata/makefile_path.mk"
  
  cat > "$MAKEFILE_PATH_MK" <<'EOF'
# makefile_path.mk
# Auto-generated by install_iocdevtools_linux.sh
# This file contains all path macros required to compile IOsonata projects using Makefiles
# Include this file in your project Makefile: include $(IOSONATA_ROOT)/makefile_path.mk

# ============================================
# IOsonata Paths
# ============================================
# IOCOMPOSER_HOME must be set to your IOcomposer root directory
ifndef IOCOMPOSER_HOME
$(error IOCOMPOSER_HOME is not set. Please set it to your IOcomposer root directory)
endif

IOSONATA_ROOT = $(IOCOMPOSER_HOME)/IOsonata
IOSONATA_INCLUDE = $(IOSONATA_ROOT)/include
IOSONATA_SRC = $(IOSONATA_ROOT)/src

# ============================================
# ARM-specific Paths
# ============================================
ARM_ROOT = $(IOSONATA_ROOT)/ARM
ARM_CMSIS = $(ARM_ROOT)/CMSIS
ARM_CMSIS_INCLUDE = $(ARM_CMSIS)/Include
ARM_INCLUDE = $(ARM_ROOT)/include
ARM_SRC = $(ARM_ROOT)/src
ARM_LDSCRIPT = $(ARM_ROOT)/ldscript

# Vendor-specific paths
ARM_NORDIC = $(ARM_ROOT)/Nordic
ARM_NXP = $(ARM_ROOT)/NXP
ARM_ST = $(ARM_ROOT)/ST
ARM_MICROCHIP = $(ARM_ROOT)/Microchip
ARM_RENESAS = $(ARM_ROOT)/Renesas

# ============================================
# RISC-V-specific Paths
# ============================================
RISCV_ROOT = $(IOSONATA_ROOT)/RISCV
RISCV_INCLUDE = $(RISCV_ROOT)/include
RISCV_SRC = $(RISCV_ROOT)/src
RISCV_LDSCRIPT = $(RISCV_ROOT)/ldscript

# Vendor-specific paths
RISCV_ESPRESSIF = $(RISCV_ROOT)/Espressif
RISCV_NORDIC = $(RISCV_ROOT)/Nordic
RISCV_RENESAS = $(RISCV_ROOT)/Renesas

# ============================================
# External Libraries
# ============================================
EXTERNAL_ROOT = $(IOCOMPOSER_HOME)/external
NRFX_ROOT = $(EXTERNAL_ROOT)/nrfx
SDK_NRF_BM_ROOT = $(EXTERNAL_ROOT)/sdk-nrf-bm
SDK_NRFXLIB_ROOT = $(EXTERNAL_ROOT)/sdk-nrfxlib
NRF5_SDK_ROOT = $(EXTERNAL_ROOT)/nRF5_SDK
NRF5_SDK_MESH_ROOT = $(EXTERNAL_ROOT)/nRF5_SDK_Mesh
BSEC_ROOT = $(EXTERNAL_ROOT)/BSEC
FUSION_ROOT = $(EXTERNAL_ROOT)/Fusion
LVGL_ROOT = $(EXTERNAL_ROOT)/lvgl
LWIP_ROOT = $(EXTERNAL_ROOT)/lwip
FREERTOS_KERNEL_ROOT = $(EXTERNAL_ROOT)/FreeRTOS-Kernel

# ============================================
# Additional IOsonata Modules
# ============================================
FATFS_ROOT = $(IOSONATA_ROOT)/fatfs
LITTLEFS_ROOT = $(IOSONATA_ROOT)/littlefs
MICRO_ECC_ROOT = $(IOSONATA_ROOT)/micro-ecc

# ============================================
# Common Include Paths (for -I flags)
# ============================================
IOSONATA_INCLUDES = -I$(IOSONATA_INCLUDE) \
                    -I$(IOSONATA_INCLUDE)/bluetooth \
                    -I$(IOSONATA_INCLUDE)/audio \
                    -I$(IOSONATA_INCLUDE)/converters \
                    -I$(IOSONATA_INCLUDE)/coredev \
                    -I$(IOSONATA_INCLUDE)/display \
                    -I$(IOSONATA_INCLUDE)/imu \
                    -I$(IOSONATA_INCLUDE)/miscdev \
                    -I$(IOSONATA_INCLUDE)/pwrmgnt \
                    -I$(IOSONATA_INCLUDE)/sensors \
                    -I$(IOSONATA_INCLUDE)/storage \
                    -I$(IOSONATA_INCLUDE)/sys \
                    -I$(IOSONATA_INCLUDE)/usb

ARM_INCLUDES = -I$(ARM_INCLUDE) \
               -I$(ARM_CMSIS_INCLUDE)

RISCV_INCLUDES = -I$(RISCV_INCLUDE)

# ============================================
# Environment Variables (optional)
# ============================================
export NRFX_HOME := $(NRFX_ROOT)
export NRFXLIB_HOME := $(SDK_NRFXLIB_ROOT)
export NRF5_SDK_HOME := $(NRF5_SDK_ROOT)
export NRF5_SDK_MESH_HOME := $(NRF5_SDK_MESH_ROOT)
export BSEC_HOME := $(BSEC_ROOT)
EOF

  # Add toolchain paths with absolute locations
  cat >> "$MAKEFILE_PATH_MK" <<EOF

# ============================================
# Toolchain Paths (installed in fixed location)
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
EOF
  
  echo "✅ makefile_path.mk created at: $MAKEFILE_PATH_MK"
  echo "   Projects can include it with: include \$(IOSONATA_ROOT)/makefile_path.mk"
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
  echo "iosonata_loc is configured in Eclipse installation"
  echo
  echo "Usage in .cproject files:"
  echo "  \${system_property:iosonata_loc}/IOsonata/include"
  echo "  \${system_property:iosonata_loc}/IOsonata/ARM/include"
  echo "  \${system_property:iosonata_loc}/IOsonata/ARM/CMSIS/Include"
  echo
  echo "The variable is available in ALL workspaces."
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
    generate_makefile_paths
    display_report
    ;;
  uninstall)
    uninstall_all
    ;;
esac
