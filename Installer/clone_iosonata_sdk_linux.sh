#!/usr/bin/env bash
set -euo pipefail

# =========================================================
#  clone_iosonata_sdk_linux
# ---------------------------------------------------------
#  Purpose: Clone IOsonata SDK and dependencies with colors
#  Platform: linux (bash)
#  Version: v1.3.0
# =========================================================

SCRIPT_VERSION="v1.3.0"

# ---------------------------------------------------------
# COLOR DEFINITIONS
# ---------------------------------------------------------
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
BOLD='\033[1m'
RESET='\033[0m'

# ---------------------------------------------------------
# BANNER
# ---------------------------------------------------------
echo ""
echo -e "${BLUE}=========================================================${RESET}"
echo -e "${BOLD}     IOsonata SDK Cloning Utility for Linux${RESET}"
echo -e "${BOLD}     Version: $SCRIPT_VERSION${RESET}"
echo -e "${BLUE}=========================================================${RESET}"

# ---------------------------------------------------------
# DEPENDENCY CHECK AND INSTALL
# ---------------------------------------------------------
check_and_install_dependencies() {
  local missing_dependencies=()

  # Check for git and make
  command -v git &>/dev/null || missing_dependencies+=("git")
  command -v make &>/dev/null || missing_dependencies+=("make")

  # No missing dependencies (So end function)
  if [[ ${#missing_dependencies[@]} -eq 0 ]]; then
    echo -e "${GREEN}✓ All required dependencies are installed${RESET}"
    return 0;
  fi

  echo -e "${YELLOW}⚠️  Missing dependencies, installing: ${missing_dependencies[*]}${RESET}"

  # Detect package manager and install
  if command -v apt-get &>/dev/null; then
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
    echo -e "${RED}❌ Could not detect package manager. Please install manually: ${missing_dependencies[*]}${RESET}"
    exit 1
  fi

  echo "✓ Dependencies installed successfully"
}

check_and_install_dependencies

# ---------------------------------------------------------
# DEFAULT CONFIGURATION VALUES
# ---------------------------------------------------------
ROOT="$HOME/IOcomposer"
MODE="normal"
CONFIGURE_ECLIPSE=false

# ------------------------------------------------------------------------------
# ARGUMENT PARSING (Added validation similar to build_iosonata_lib_linux.sh)
# ------------------------------------------------------------------------------

while [[ $# -gt 0 ]]; do
  case "$1" in
    --home)
      if [[ -z "${2:-}" ]]; then
        echo -e "${RED}✗ Missing path after --home${RESET}"
        exit 1
      fi
      ROOT="$2"
      shift 2
      ;;
    --mode)
      if [[ -z "${2:-}" ]]; then
        echo -e "${RED}✗ Missing mode after --mode${RESET}"
        exit 1
      fi
      MODE="$2"
      shift 2
      ;;
    --eclipse)
      CONFIGURE_ECLIPSE=true
      shift
      ;;
    --help|-h)
      echo -e "${BOLD}Usage:${RESET} clone_iosonata_sdk_linux.sh [options]"
      echo ""
      echo -e "${BOLD}Options:${RESET}"
      echo "  --home <path>       Set custom root directory (default: ~/IOcomposer)"
      echo "  --mode <mode>       Clone mode: 'normal' (default) or 'force'"
      echo "  --eclipse           Configure Eclipse system properties"
      echo "  --help, -h          Show this help message and exit"
      echo ""
      echo -e "${BOLD}Examples:${RESET}"
      echo "  ./clone_iosonata_sdk_linux.sh                      # Clone into ~/IOcomposer"
      echo "  ./clone_iosonata_sdk_linux.sh --home ~/DevEnv      # Custom directory"
      echo "  ./clone_iosonata_sdk_linux.sh --mode force         # Force re-clone all repos"
      echo "  ./clone_iosonata_sdk_linux.sh --eclipse            # Also configure Eclipse"
      echo ""
      echo -e "${BOLD}Repositories cloned:${RESET}"
      echo "  • IOsonata main repository"
      echo "  • Nordic Semiconductor nrfx"
      echo "  • nrfconnect SDKs (sdk-nrf-bm, sdk-nrfxlib)"
      echo "  • IOsonata nRF5 SDK & Mesh"
      echo "  • Bosch BSEC2 library"
      echo "  • xioTechnologies Fusion"
      echo "  • VQF orientation library"
      echo "  • LVGL graphics library"
      echo "  • lwIP TCP/IP stack"
      echo "  • FreeRTOS-Kernel"
      echo "  • TinyUSB"
      echo ""
      exit 0
      ;;
    *)
      echo -e "${RED}✗ Unknown argument:${RESET} $1"
      echo -e "Use '${YELLOW}--help${RESET}' for usage information."
      exit 1
      ;;
  esac
done

# ---------------------------------------------------------
# DIRECTORY PREPARATION AND DISPLAY
# ---------------------------------------------------------

mkdir -p "$ROOT"
ROOT=$(cd "$ROOT" && pwd)
EXT="$ROOT/external"
mkdir -p "$EXT"

echo -e "${BOLD}Root directory:${RESET} $ROOT"
echo -e "${BOLD}Mode:${RESET} $MODE"
echo -e "${BOLD}External path:${RESET} $EXT"
echo -e "${BOLD}Configure Eclipse:${RESET} $CONFIGURE_ECLIPSE"
echo -e "${BLUE}---------------------------------------------------------${RESET}"
echo ""

# ---------------------------------------------------------
# CLONE / UPDATE REPOSITORY
# ---------------------------------------------------------

clone_or_update_repo() {
  local repo_url="$1"
  local target_dir="$2"
  local recurse_submodules="${3:-false}"

  # Target directory found
  if [[ -d "$target_dir" ]]; then

    # When forced, delete and re-clone the repository
    if [[ "$MODE" == "force" ]]; then
      echo -e "${YELLOW}⚠️  Re-cloning${RESET} $target_dir ..."
      rm -rf "$target_dir"
      
      if [[ "$recurse_submodules" == "true" ]]; then
        git clone --depth=1 --recurse-submodules "$repo_url" "$target_dir"
      else
        git clone --depth=1 "$repo_url" "$target_dir"
      fi

    # Otherwise update (git pull)
    else
      echo -e "${YELLOW}🔄 Updating${RESET} $target_dir ..."
      # Added fallback logic for failed updates
      (cd "$target_dir" && git pull --rebase) || {
        echo -e "${RED}   ✗ Update failed, trying fresh clone...${RESET}"
        rm -rf "$target_dir"
        git clone --depth=1 "$repo_url" "$target_dir"
      }
      if [[ "$recurse_submodules" == "true" ]]; then
        echo -e "${BLUE}   ↳ Updating submodules...${RESET}"
        (cd "$target_dir" && git submodule update --init --recursive)
      fi
    fi

  # If target directory not found, create fresh clone
  else
    echo -e "${BLUE}⬇️  Cloning${RESET} $target_dir ..."
    if [[ "$recurse_submodules" == "true" ]]; then
      git clone --depth=1 --recurse-submodules "$repo_url" "$target_dir"
      echo -e "${GREEN}   ✓ Submodules initialized${RESET}"
    else
      git clone --depth=1 "$repo_url" "$target_dir"
    fi
  fi
}

# ---------------------------------------------------------
# IOSONATA LIBRARY BUILD FUNCTION
# ---------------------------------------------------------

# Clone IOsonata
echo -e "${GREEN}🚀 Cloning IOsonata...${RESET}"
clone_or_update_repo "https://github.com/IOsonata/IOsonata.git" "$ROOT/IOsonata"

# Clone External Repos
echo ""
echo -e "${BLUE}📦 Cloning dependencies into:${RESET} $EXT"
echo ""

# Standard repos (no submodules)
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
  target="$EXT/$name"
  clone_or_update_repo "$repo" "$target" false
done

# Clone FreeRTOS-Kernel
echo ""
echo -e "${BOLD}${BLUE}📦 Cloning FreeRTOS-Kernel...${RESET}"
clone_or_update_repo "https://github.com/FreeRTOS/FreeRTOS-Kernel.git" "$EXT/FreeRTOS-Kernel" false

echo ""
echo -e "${GREEN}✅ All repositories cloned successfully!${RESET}"

# Generate makefile_path.mk
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

echo -e "${GREEN}✅ makefile_path.mk created at: $MAKEFILE_PATH_MK${RESET}"
echo "   Projects can include it with: include \$(IOSONATA_ROOT)/makefile_path.mk"
echo "   Make sure to set IOCOMPOSER_HOME=$ROOT in your environment or Makefile"
echo ""

# ---------------------------------------------------------
# INSTALL IOSONATA ECLIPSE PLUGIN (HELPER FUNCTION)
# ---------------------------------------------------------

install_iosonata_plugin() {
  local eclipse_dir="$1"
  
  echo ""
  echo -e "${BLUE}>>> Installing IOsonata Eclipse Plugin...${RESET}"
  
  local plugin_dir="$ROOT/IOsonata/Installer/eclipse_plugin"
  local dropins_dir="$eclipse_dir/dropins"
  
  # Check if plugin directory exists
  if [[ ! -d "$plugin_dir" ]]; then
    echo -e "${YELLOW}⚠️  Plugin directory not found at $plugin_dir${RESET}"
    echo "    Skipping plugin installation."
    return
  fi
  
  # Find the latest plugin jar file
  local latest_plugin
  latest_plugin=$(ls -1 "$plugin_dir"/org.iosonata.embedcdt.templates.firmware_*.jar 2>/dev/null | sort -V | tail -n1)
  
  if [[ -z "$latest_plugin" ]]; then
    echo -e "${YELLOW}⚠️  No IOsonata plugin jar file found in $plugin_dir${RESET}"
    echo "    Skipping plugin installation."
    return
  fi
  
  echo -e "${GREEN}   → Found plugin: $(basename "$latest_plugin")${RESET}"
  
  # Create dropins directory if it doesn't exist
  # Try without sudo first, fall back to sudo if needed
  if [[ -w "$eclipse_dir" ]]; then
    mkdir -p "$dropins_dir"
  else
    sudo mkdir -p "$dropins_dir"
  fi
  
  # Remove old versions of the plugin
  local old_plugins
  if [[ -w "$dropins_dir" ]]; then
    old_plugins=$(find "$dropins_dir" -name "org.iosonata.embedcdt.templates.firmware_*.jar" 2>/dev/null || true)
  else
    old_plugins=$(sudo find "$dropins_dir" -name "org.iosonata.embedcdt.templates.firmware_*.jar" 2>/dev/null || true)
  fi
  
  if [[ -n "$old_plugins" ]]; then
    echo -e "${YELLOW}   → Removing old plugin versions...${RESET}"
    echo "$old_plugins" | while read -r old_plugin; do
      if [[ -f "$old_plugin" ]]; then
        echo "      - Removing: $(basename "$old_plugin")"
        if [[ -w "$old_plugin" ]]; then
          rm -f "$old_plugin"
        else
          sudo rm -f "$old_plugin"
        fi
      fi
    done
  fi
  
  # Copy the latest plugin to dropins
  echo "   → Installing plugin to $dropins_dir"
  if [[ -w "$dropins_dir" ]]; then
    cp "$latest_plugin" "$dropins_dir/"
    chmod 644 "$dropins_dir/$(basename "$latest_plugin")"
  else
    sudo cp "$latest_plugin" "$dropins_dir/"
    sudo chmod 644 "$dropins_dir/$(basename "$latest_plugin")"
  fi
  
  echo -e "${GREEN}✅ IOsonata Eclipse Plugin installed: $(basename "$latest_plugin")${RESET}"
}

# ---------------------------------------------------------
# ECLIPSE CONFIGURATIONS
# ---------------------------------------------------------

if [[ "$CONFIGURE_ECLIPSE" == "true" ]]; then
  echo ""
  echo -e "${BLUE}=========================================================${RESET}"
  echo -e "${BOLD}Configuring Eclipse System Properties${RESET}"
  echo -e "${BLUE}=========================================================${RESET}"
  echo ""
  
  # Find Eclipse installation
  ECLIPSE_DIR=""
  ECLIPSE_INI=""
  
  # Search common locations
  for candidate in \
    "${HOME}/eclipse/embedcdt" \
    "${HOME}/eclipse/cpp" \
    "${HOME}/eclipse" \
    "/opt/eclipse" \
    "/usr/local/eclipse" \
    "/snap/eclipse/current"
  do
    if [[ -f "$candidate/eclipse.ini" ]]; then
      ECLIPSE_DIR="$candidate"
      ECLIPSE_INI="$candidate/eclipse.ini"
      break
    fi
  done
  
  if [[ -z "$ECLIPSE_INI" ]]; then
    echo -e "${RED}✗ Eclipse not found${RESET}"
    echo -e "${YELLOW}  Searched: ~/eclipse/*, /opt/eclipse, /usr/local/eclipse${RESET}"
    echo -e "${YELLOW}  Please install Eclipse first or skip Eclipse configuration.${RESET}"
  else
    echo -e "${GREEN}✓ Eclipse found at: $ECLIPSE_DIR${RESET}"
    echo ""
    echo ">>> Configuring IOsonata and IOcomposer system properties..."
    
    # Create backup
    cp "$ECLIPSE_INI" "$ECLIPSE_INI.bak.$(date +%Y%m%d%H%M%S)" 2>/dev/null || \
      sudo cp "$ECLIPSE_INI" "$ECLIPSE_INI.bak.$(date +%Y%m%d%H%M%S)"
    
    # Remove old properties if they exist (GNU sed syntax)
    if [[ -w "$ECLIPSE_INI" ]]; then
      sed -i '/^-Diosonata\.home=/d' "$ECLIPSE_INI" 2>/dev/null || true
      sed -i '/^-Diosonata_loc=/d' "$ECLIPSE_INI" 2>/dev/null || true
      sed -i '/^-Diocomposer_home=/d' "$ECLIPSE_INI" 2>/dev/null || true
    else
      sudo sed -i '/^-Diosonata\.home=/d' "$ECLIPSE_INI" 2>/dev/null || true
      sudo sed -i '/^-Diosonata_loc=/d' "$ECLIPSE_INI" 2>/dev/null || true
      sudo sed -i '/^-Diocomposer_home=/d' "$ECLIPSE_INI" 2>/dev/null || true
    fi
    
    # Add new properties after -vmargs
    if grep -q "^-vmargs" "$ECLIPSE_INI"; then
      # Insert after -vmargs line (GNU sed syntax)
      if [[ -w "$ECLIPSE_INI" ]]; then
        sed -i '/^-vmargs$/a -Diosonata_loc='"$ROOT" "$ECLIPSE_INI"
        sed -i '/^-Diosonata_loc=/a -Diocomposer_home='"$ROOT" "$ECLIPSE_INI"
      else
        sudo sed -i '/^-vmargs$/a -Diosonata_loc='"$ROOT" "$ECLIPSE_INI"
        sudo sed -i '/^-Diosonata_loc=/a -Diocomposer_home='"$ROOT" "$ECLIPSE_INI"
      fi
    else
      # No -vmargs section, add it at the end
      if [[ -w "$ECLIPSE_INI" ]]; then
        echo "-vmargs" >> "$ECLIPSE_INI"
        echo "-Diosonata_loc=$ROOT" >> "$ECLIPSE_INI"
        echo "-Diocomposer_home=$ROOT" >> "$ECLIPSE_INI"
      else
        echo "-vmargs" | sudo tee -a "$ECLIPSE_INI" > /dev/null
        echo "-Diosonata_loc=$ROOT" | sudo tee -a "$ECLIPSE_INI" > /dev/null
        echo "-Diocomposer_home=$ROOT" | sudo tee -a "$ECLIPSE_INI" > /dev/null
      fi
    fi
    
    echo -e "${GREEN}✅ Eclipse system properties configured:${RESET}"
    echo -e "   ${BOLD}iosonata_loc${RESET} = $ROOT"
    echo -e "   ${BOLD}iocomposer_home${RESET} = $ROOT"
    echo ""
    echo -e "${YELLOW}ℹ️  Usage in CDT projects:${RESET}"
    echo '   ${system_property:iosonata_loc}/IOsonata/include'
    echo '   ${system_property:iosonata_loc}/external/nrfx'
    echo ""
    
    # Install Eclipse plugin
    install_iosonata_plugin "$ECLIPSE_DIR"
    echo ""
    
    echo -e "${YELLOW}⚠️  Restart Eclipse for changes to take effect.${RESET}"
  fi
fi

# ---------------------------------------------------------
# SUMMARY DISPLAY
# ---------------------------------------------------------
echo ""
echo -e "${BLUE}=========================================================${RESET}"
echo -e "${BOLD}Summary${RESET}"
echo -e "${BLUE}=========================================================${RESET}"
echo -e "${BOLD}IOsonata Root:${RESET}       $ROOT"
echo -e "${BOLD}IOsonata Framework:${RESET}  $ROOT/IOsonata"
echo -e "${BOLD}External Repos:${RESET}      $EXT"
echo ""
echo -e "${BOLD}Cloned repositories:${RESET}"
echo "  • IOsonata"
echo "  • nrfx"
echo "  • sdk-nrf-bm"
echo "  • sdk-nrfxlib"
echo "  • nRF5_SDK"
echo "  • nRF5_SDK_Mesh"
echo "  • BSEC"
echo "  • Fusion"
echo "  • vqf"
echo "  • lvgl"
echo "  • lwip"
echo "  • FreeRTOS-Kernel"
echo "  • tinyusb"
echo ""

if [[ "$CONFIGURE_ECLIPSE" == "true" && -n "$ECLIPSE_INI" && -f "$ECLIPSE_INI" ]]; then
  echo -e "${BOLD}Eclipse Configuration:${RESET}"
  echo "  ✓ System properties configured"
  echo "  ✓ iosonata_loc = $ROOT"
  echo "  ✓ iocomposer_home = $ROOT"
  echo ""
fi

echo -e "${BLUE}---------------------------------------------------------${RESET}"

# ---------------------------------------------------------
# AUTO BUILD TRIGGER
# ---------------------------------------------------------

# --- Detect Eclipse Installation ---
ECLIPSE_INSTALLED=false
ECLIPSE_BIN=""

for candidate in \
  "${HOME}/eclipse/embedcdt/eclipse" \
  "${HOME}/eclipse/cpp/eclipse" \
  "${HOME}/eclipse/eclipse" \
  "/opt/eclipse/eclipse" \
  "/usr/local/eclipse/eclipse"
do
  if [[ -x "$candidate" ]]; then
    ECLIPSE_INSTALLED=true
    ECLIPSE_BIN="$candidate"
    break
  fi
done

if [[ "$ECLIPSE_INSTALLED" == "true" ]]; then
  echo ""
  echo -e "${GREEN}✓ Eclipse detected at $(dirname "$ECLIPSE_BIN")${RESET}"
fi

# --- Auto-Build IOsonata Libraries (if Eclipse detected) ---
if [[ "$ECLIPSE_INSTALLED" == "true" ]]; then
  BUILD_SCRIPT="$ROOT/IOsonata/Installer/build_iosonata_lib_linux.sh"
  if [[ -f "$BUILD_SCRIPT" ]]; then
    echo ""
    echo -e "${BLUE}=========================================================${RESET}"
    echo -e "${BOLD}   IOsonata Library Auto-Build${RESET}"
    echo -e "${BLUE}=========================================================${RESET}"
    echo ""
    chmod +x "$BUILD_SCRIPT" 2>/dev/null || true
    "$BUILD_SCRIPT" --home "$ROOT" || true
  else
    echo ""
    echo -e "${YELLOW}Note: Build script not found at:${RESET}"
    echo "      $BUILD_SCRIPT"
    echo ""
    echo "To build libraries, download build script from:"
    echo "  https://github.com/IOsonata/IOsonata"
    echo ""
  fi
else
  echo ""
  echo -e "${YELLOW}Note: Eclipse not detected.${RESET}"
  echo "      To build IOsonata libraries, install Eclipse and run:"
  echo -e "      ${BOLD}./install_iocdevtools_linux.sh${RESET}"
  echo ""
fi

echo -e "${GREEN}Done. Happy building! 🔧${RESET}"
echo ""