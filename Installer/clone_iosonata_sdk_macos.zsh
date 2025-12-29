#!/bin/zsh
# =========================================================
#  clone_iosonata_sdk_macos
# ---------------------------------------------------------
#  Purpose: Clone IOsonata SDK and dependencies with colors
#  Platform: macOS (zsh)
#  Version: v1.1.0
# =========================================================

set -e  # Exit on first error

SCRIPT_VERSION="v1.1.0"

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
  echo "  --eclipse           Configure Eclipse system properties"
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
  echo "  • FreeRTOS (with submodules)"
  echo ""
  exit 0
}

# --- Default values ---
ROOT="$HOME/IOcomposer"
MODE="normal"
CONFIGURE_ECLIPSE=false

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
  target="$EXT/$name"
  clone_or_update_repo "$repo" "$target" false
done

# ---------------------------------------------------------
# Clone FreeRTOS with submodules
# ---------------------------------------------------------
echo ""
echo "${BOLD}${BLUE}📦 Cloning FreeRTOS (with submodules)...${RESET}"
clone_or_update_repo "https://github.com/FreeRTOS/FreeRTOS.git" "$EXT/FreeRTOS" true

echo ""
echo "${GREEN}✅ All repositories cloned successfully!${RESET}"

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
echo "  • FreeRTOS ${GREEN}(with submodules)${RESET}"
echo ""

if [[ "$CONFIGURE_ECLIPSE" == "true" && -f "$ECLIPSE_INI" ]]; then
  echo "${BOLD}Eclipse Configuration:${RESET}"
  echo "  ✓ System properties configured"
  echo "  ✓ iosonata_loc = $ROOT"
  echo "  ✓ iocomposer_home = $ROOT"
  echo ""
fi

echo "${BLUE}---------------------------------------------------------${RESET}"
echo "${GREEN}Done. Happy building! 🔧${RESET}"
echo ""
