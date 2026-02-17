#!/bin/zsh
# =========================================================
#  clone_iosonata_sdk_macos
# ---------------------------------------------------------
#  Purpose: Clone IOsonata SDK and dependencies with colors
#  Platform: macOS (zsh)
#  Version: v1.1.0
# =========================================================

set -e  # Exit on first error

SCRIPT_VERSION="v1.3.1"

# --- Define colors ---
RED="\033[0;31m"
GREEN="\033[0;32m"
YELLOW="\033[1;33m"
BLUE="\033[0;34m"
BOLD="\033[1m"
RESET="\033[0m"

# --- Banner ---
print_banner() {
  echo ""
  echo "${BLUE}=========================================================${RESET}"
  echo "${BOLD}     IOsonata SDK Cloning Utility for macOS${RESET}"
  echo "${BOLD}     Version: $SCRIPT_VERSION${RESET}"
  echo "${BLUE}=========================================================${RESET}"
}

# --- Help Menu ---
print_help() {
  print_banner
  echo "${BOLD}Usage:${RESET} clone_iosonata_sdk_macos [options]"
  echo ""
  echo "${BOLD}Options:${RESET}"
  echo "  --home <path>       Set custom root directory (default: ~/IOcomposer)"
  echo "  --mode <mode>       Clone mode: 'normal' (default) or 'force'"
  echo "  --eclipse           Configure Eclipse system properties
  --no-build          Skip IOsonata library auto-build step"
  echo "  --help, -h          Show this help message and exit"

  echo ""
  echo "${BOLD}Examples:${RESET}"
  echo "  ./clone_iosonata_sdk_macos                      # Clone into ~/IOcomposer"
  echo "  ./clone_iosonata_sdk_macos --home ~/DevEnv      # Custom directory"
  echo "  ./clone_iosonata_sdk_macos --mode force         # Force re-clone all repos"
  echo "  ./clone_iosonata_sdk_macos --eclipse            # Also configure Eclipse"
  echo ""
  echo "${BOLD}Repositories cloned (during normal execution):${RESET}"
  echo "  • IOsonata main repository"
  echo "  • Nordic Semiconductor nrfx"
  echo "  • nrfconnect SDKs (sdk-nrf-bm, sdk-nrfxlib)"
  echo "  • IOsonata nRF5 SDK & Mesh"
  echo "  • Bosch BSEC2 library"
  echo "  • xioTechnologies Fusion"
  echo "  • LVGL graphics library"
  echo "  • lwIP TCP/IP stack"
  echo "  • FreeRTOS-Kernel"
  echo "  • TinyUSB"
  echo ""
  exit 0
}

# --- Default values ---
ROOT="$HOME/IOcomposer"
MODE="normal"
CONFIGURE_ECLIPSE=false
NO_BUILD=false

# --- Parse Arguments ---
while [[ $# -gt 0 ]]; do
  case $1 in
    --home)
      ROOT="$2"
      shift 2
      ;;
    --mode)
      MODE="$2"
      shift 2
      ;;
    --eclipse)
      CONFIGURE_ECLIPSE=true
      shift
      ;;
    --no-build)
      NO_BUILD=true
      shift
      ;;
    --help|-h)
      print_help
      ;;
    *)
      echo "${RED}✗ Unknown argument:${RESET} $1"
      echo "Use '${YELLOW}--help${RESET}' for usage information."
      exit 1
      ;;
  esac
done

# --- Prepare directories safely ---
mkdir -p "$ROOT"
ROOT=$(cd "$ROOT" && pwd)
EXT="$ROOT/external"
mkdir -p "$EXT"

print_banner
echo "${BOLD}Root directory:${RESET} $ROOT"
echo "${BOLD}Mode:${RESET} $MODE"
echo "${BOLD}External path:${RESET} $EXT"
echo "${BOLD}Configure Eclipse:${RESET} $CONFIGURE_ECLIPSE"
echo "${BOLD}Skip auto-build:${RESET} $NO_BUILD"
echo "${BLUE}---------------------------------------------------------${RESET}"
echo ""

# --- Function: clone or update repo ---
clone_or_update_repo() {
  local repo_url="$1"
  local target_dir="$2"
  local recurse_submodules="${3:-false}"

  if [[ -d "$target_dir" ]]; then
    if [[ "$MODE" == "force" ]]; then
      echo "${YELLOW}⚠️  Re-cloning${RESET} $target_dir ..."
      rm -rf "$target_dir"
      if [[ "$recurse_submodules" == "true" ]]; then
        git clone --depth=1 --recurse-submodules "$repo_url" "$target_dir"
      else
        git clone --depth=1 "$repo_url" "$target_dir"
      fi
    else
      echo "${YELLOW}🔄 Updating${RESET} $target_dir ..."
      (cd "$target_dir" && git pull --rebase)
      if [[ "$recurse_submodules" == "true" ]]; then
        echo "${BLUE}   ↳ Updating submodules...${RESET}"
        (cd "$target_dir" && git submodule update --init --recursive)
      fi
    fi
  else
    echo "${BLUE}⬇️  Cloning${RESET} $target_dir ..."
    if [[ "$recurse_submodules" == "true" ]]; then
      git clone --depth=1 --recurse-submodules "$repo_url" "$target_dir"
      echo "${GREEN}   ✓ Submodules initialized${RESET}"
    else
      git clone --depth=1 "$repo_url" "$target_dir"
    fi
  fi
}

# --- IOsonata Library Build Function ---

# ---------------------------------------------------------
# Clone IOsonata
# ---------------------------------------------------------
echo "${GREEN}🚀 Cloning IOsonata...${RESET}"
clone_or_update_repo "https://github.com/IOsonata/IOsonata.git" "$ROOT/IOsonata"

# ---------------------------------------------------------
# Clone External Repos
# ---------------------------------------------------------
echo ""
echo "${BLUE}📦 Cloning dependencies into:${RESET} $EXT"
echo ""

# Standard repos (no submodules)
repos=(
  "https://github.com/NordicSemiconductor/nrfx.git"
  "https://github.com/IOsonata/sdk-nrf-bm.git"
  "https://github.com/nrfconnect/sdk-nrfxlib.git"
  "https://github.com/nrfconnect/sdk-oberon-psa-crypto.git"
  "https://github.com/nrfconnect/sdk-mbedtls.git"
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
  target="$EXT/$name"
  clone_or_update_repo "$repo" "$target" false
done

# ---------------------------------------------------------
# Clone FreeRTOS-Kernel
# ---------------------------------------------------------
echo ""
echo "${BOLD}${BLUE}📦 Cloning FreeRTOS-Kernel...${RESET}"
clone_or_update_repo "https://github.com/FreeRTOS/FreeRTOS-Kernel.git" "$EXT/FreeRTOS-Kernel" false

echo ""
echo "${GREEN}✅ All repositories cloned successfully!${RESET}"

# ---------------------------------------------------------
# Install IDAP Tools
# ---------------------------------------------------------
echo ""
echo "${BLUE}>>> Installing IDAP Tools...${RESET}"
IDAP_DIR="$ROOT/IDAP"
IDAP_PROG="$IDAP_DIR/IDAPnRFProg"
mkdir -p "$IDAP_DIR"

if [[ ! -f "$IDAP_PROG" || "$MODE" == "force" ]]; then
  echo "   Downloading IDAPnRFProg tool..."
  IDAP_URL="https://sourceforge.net/projects/idaplinkfirmware/files/OSX/IDAPnRFProg_OSX_2_1_240807.zip/download"
  IDAP_TMP=$(mktemp)
  if curl -fL "$IDAP_URL" -o "$IDAP_TMP"; then
    unzip -o -j "$IDAP_TMP" -d "$IDAP_DIR" 2>/dev/null || true
    chmod +x "$IDAP_DIR/IDAPnRFProg" 2>/dev/null || true
    rm -f "$IDAP_TMP"
    echo "${GREEN}   ✅ IDAPnRFProg installed at: $IDAP_PROG${RESET}"
  else
    echo "${YELLOW}   ⚠️  Failed to download IDAPnRFProg. You can download manually from:${RESET}"
    echo "      https://sourceforge.net/projects/idaplinkfirmware/files/OSX/"
    rm -f "$IDAP_TMP"
  fi
else
  echo "${GREEN}   ✅ IDAPnRFProg already installed (skipping)${RESET}"
fi

echo ""
echo "${GREEN}✅ All repositories cloned successfully!${RESET}"

# ---------------------------------------------------------
# Generate makefile_path.mk
# ---------------------------------------------------------
echo ""
echo "📝 Generating makefile_path.mk for Makefile-based builds..."

MAKEFILE_PATH_MK="$ROOT/IOsonata/makefile_path.mk"

cat > "$MAKEFILE_PATH_MK" <<'EOF'
# makefile_path.mk
# Auto-generated by clone_iosonata_sdk_macos.zsh
# This file contains path macros for IOsonata projects
# Include this file in your project Makefile: include $(IOSONATA_ROOT)/makefile_path.mk

# ============================================
# IOsonata Paths
# ============================================
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
TINYUSB_ROOT = $(EXTERNAL_ROOT)/tinyusb

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
ifndef IOCOMPOSER_HOME
$(error IOCOMPOSER_HOME is not set. Please set it to your IOcomposer root directory)
endif

export NRFX_HOME := $(NRFX_ROOT)
export NRFXLIB_HOME := $(SDK_NRFXLIB_ROOT)
export NRF5_SDK_HOME := $(NRF5_SDK_ROOT)
export NRF5_SDK_MESH_HOME := $(NRF5_SDK_MESH_ROOT)
export BSEC_HOME := $(BSEC_ROOT)
EOF

echo "${GREEN}✅ makefile_path.mk created at: $MAKEFILE_PATH_MK${RESET}"
echo "   Projects can include it with: include \$(IOSONATA_ROOT)/makefile_path.mk"
echo "   Make sure to set IOCOMPOSER_HOME=$ROOT in your environment or Makefile"
echo ""

# ---------------------------------------------------------
# Install IOsonata Eclipse Plugin (helper function)
# ---------------------------------------------------------
install_iosonata_plugin() {
  local eclipse_app="$1"
  
  echo ""
  echo "${BLUE}>>> Installing IOsonata Eclipse Plugin...${RESET}"
  
  local plugin_dir="$ROOT/IOsonata/Installer/eclipse_plugin"
  local dropins_dir="$eclipse_app/Contents/Eclipse/dropins"
  
  # Check if plugin directory exists
  if [[ ! -d "$plugin_dir" ]]; then
    echo "${YELLOW}⚠️  Plugin directory not found at $plugin_dir${RESET}"
    echo "    Skipping plugin installation."
    return
  fi
  
  # Find the latest plugin jar file
  local latest_plugin
  latest_plugin=$(ls -1 "$plugin_dir"/org.iosonata.embedcdt.templates.wizard_*.jar 2>/dev/null | sort -V | tail -n1)
  
  if [[ -z "$latest_plugin" ]]; then
    echo "${YELLOW}⚠️  No IOsonata plugin jar file found in $plugin_dir${RESET}"
    echo "    Skipping plugin installation."
    return
  fi
  
  echo "${GREEN}   → Found plugin: $(basename "$latest_plugin")${RESET}"
  
  # Create dropins directory if it doesn't exist
  sudo mkdir -p "$dropins_dir"
  
  # Remove old versions of the plugin
  local old_plugins
  old_plugins=$(sudo find "$dropins_dir" -name "org.iosonata.embedcdt.templates.wizard_*.jar" 2>/dev/null || true)
  
  if [[ -n "$old_plugins" ]]; then
    echo "${YELLOW}   → Removing old plugin versions...${RESET}"
    echo "$old_plugins" | while read -r old_plugin; do
      if [[ -f "$old_plugin" ]]; then
        echo "      - Removing: $(basename "$old_plugin")"
        sudo rm -f "$old_plugin"
      fi
    done
  fi
  
  # Copy the latest plugin to dropins
  echo "   → Installing plugin to $dropins_dir"
  sudo cp "$latest_plugin" "$dropins_dir/"
  
  # Set proper permissions
  sudo chmod 644 "$dropins_dir/$(basename "$latest_plugin")"
  
  echo "${GREEN}✅ IOsonata Eclipse Plugin installed: $(basename "$latest_plugin")${RESET}"
}

# ---------------------------------------------------------
# Configure Eclipse (if requested)
# ---------------------------------------------------------
if [[ "$CONFIGURE_ECLIPSE" == "true" ]]; then
  echo ""
  echo "${BLUE}=========================================================${RESET}"
  echo "${BOLD}Configuring Eclipse System Properties${RESET}"
  echo "${BLUE}=========================================================${RESET}"
  echo ""
  
  ECLIPSE_APP="/Applications/Eclipse.app"
  ECLIPSE_INI="$ECLIPSE_APP/Contents/Eclipse/eclipse.ini"
  
  if [[ ! -f "$ECLIPSE_INI" ]]; then
    echo "${RED}✗ Eclipse not found at: $ECLIPSE_APP${RESET}"
    echo "${YELLOW}  Please install Eclipse first or skip Eclipse configuration.${RESET}"
  else
    echo "${GREEN}✓ Eclipse found${RESET}"
    echo ""
    echo ">>> Configuring IOsonata and IOcomposer system properties..."
    
    # Remove old properties if they exist
    sudo sed -i.bak '/^-Diosonata\.home=/d' "$ECLIPSE_INI" 2>/dev/null || true
    sudo sed -i '' '/^-Diosonata_loc=/d' "$ECLIPSE_INI" 2>/dev/null || true
    sudo sed -i '' '/^-Diocomposer_home=/d' "$ECLIPSE_INI" 2>/dev/null || true
    
    # Find the -vmargs line and insert after it
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
    
    echo "${GREEN}✅ Eclipse system properties configured:${RESET}"
    echo "   ${BOLD}iosonata_loc${RESET} = $ROOT"
    echo "   ${BOLD}iocomposer_home${RESET} = $ROOT"
    echo ""
    echo "${YELLOW}ℹ️  Usage in CDT projects:${RESET}"
    echo "   \${system_property:iosonata_loc}/IOsonata/include"
    echo "   \${system_property:iosonata_loc}/external/nrfx"
    echo ""
    
    # Install Eclipse plugin
    install_iosonata_plugin "$ECLIPSE_APP"
    echo ""
    
    echo "${YELLOW}⚠️  Restart Eclipse for changes to take effect.${RESET}"
  fi
fi

# ---------------------------------------------------------
# Summary
# ---------------------------------------------------------
echo ""
echo "${BLUE}=========================================================${RESET}"
echo "${BOLD}Summary${RESET}"
echo "${BLUE}=========================================================${RESET}"
echo "${BOLD}IOsonata Root:${RESET}       $ROOT"
echo "${BOLD}IOsonata Framework:${RESET}  $ROOT/IOsonata"
echo "${BOLD}External Repos:${RESET}      $EXT"
echo "${BOLD}IDAP Tools:${RESET}          $ROOT/IDAP"
echo ""
echo "${BOLD}Cloned repositories:${RESET}"
echo "  • IOsonata"
echo "  • nrfx"
echo "  • sdk-nrf-bm"
echo "  • sdk-nrfxlib"
echo "  • nRF5_SDK"
echo "  • nRF5_SDK_Mesh"
echo "  • BSEC"
echo "  • Fusion"
echo "  • lvgl"
echo "  • lwip"
echo "  • FreeRTOS-Kernel"
echo "  • tinyusb"
echo ""

if [[ "$CONFIGURE_ECLIPSE" == "true" && -f "$ECLIPSE_INI" ]]; then
  echo "${BOLD}Eclipse Configuration:${RESET}"
  echo "  ✓ System properties configured"
  echo "  ✓ iosonata_loc = $ROOT"
  echo "  ✓ iocomposer_home = $ROOT"
  echo ""
fi

echo "${BLUE}---------------------------------------------------------${RESET}"

# --- Detect Eclipse Installation ---
ECLIPSE_APP="/Applications/Eclipse.app"
ECLIPSE_INSTALLED=false
if [[ -d "$ECLIPSE_APP" ]]; then
  ECLIPSE_INSTALLED=true
  echo ""
  echo "${GREEN}✓ Eclipse detected at $ECLIPSE_APP${RESET}"
fi

# --- Detect Toolchain Installation ---
TOOLCHAIN_INSTALLED=false
if command -v arm-none-eabi-gcc &>/dev/null; then
  TOOLCHAIN_INSTALLED=true
  echo -e "${GREEN}✓ ARM toolchain detected${RESET}"
fi

# --- Detect Toolchain Installation ---
TOOLCHAIN_INSTALLED=false
if command -v arm-none-eabi-gcc &>/dev/null; then
  TOOLCHAIN_INSTALLED=true
  echo "${GREEN}✓ ARM toolchain detected${RESET}"
fi

# --- Auto-Build IOsonata Libraries (if Eclipse detected) ---
if [[ "$NO_BUILD" != "true" && "$ECLIPSE_INSTALLED" == "true" && "$TOOLCHAIN_INSTALLED" == "true" ]]; then
  BUILD_SCRIPT="$ROOT/IOsonata/Installer/build_iosonata_lib_macos.sh"
  if [[ -f "$BUILD_SCRIPT" ]]; then
    echo ""
    echo "${BLUE}=========================================================${RESET}"
    echo "${BOLD}   IOsonata Library Auto-Build${RESET}"
    echo "${BLUE}=========================================================${RESET}"
    echo ""
    "$BUILD_SCRIPT" --home "$ROOT" || true
  else
    echo ""
    echo "${YELLOW}Note: Build script not found at:${RESET}"
    echo "      $BUILD_SCRIPT"
    echo ""
    echo "To build libraries, download build script from:"
    echo "  https://github.com/IOsonata/IOsonata"
    echo ""
  fi
elif [[ "$ECLIPSE_INSTALLED" != "true" ]]; then
  echo ""
  echo "${YELLOW}Note: Eclipse not detected.${RESET}"
  echo "      To build IOsonata libraries, install Eclipse and run:"
  echo "      ${BOLD}./install_iocdevtools_macos.sh${RESET}"
  echo ""
elif [[ "$TOOLCHAIN_INSTALLED" != "true" ]]; then
  echo ""
  echo "${YELLOW}Note: ARM toolchain not detected.${RESET}"
  echo "      To build IOsonata libraries, install toolchains first by running:"
  echo "      ${BOLD}./install_iocdevtools_macos.sh${RESET}"
  echo ""
fi

echo "${GREEN}Done. Happy building! 🔧${RESET}"
echo ""
