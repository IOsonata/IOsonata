#!/bin/zsh
# =========================================================
#  clone_iosonata_sdk_macos
# ---------------------------------------------------------
#  Purpose: Clone IOsonata SDK and dependencies with colors
#  Platform: macOS (zsh)
# =========================================================

set -e  # Exit on first error

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
  echo "${BOLD}     🧩 IOsonata SDK Cloning Utility for macOS 🌀${RESET}"
  echo "${BLUE}=========================================================${RESET}"
}

# --- Help Menu ---
print_help() {
  print_banner
  echo "${BOLD}Usage:${RESET} clone_iosonata_sdk_macos [options]"
  echo ""
  echo "${BOLD}Options:${RESET}"
  echo "  --home <path>     Set custom root directory (default: ~/IOcomposer)"
  echo "  --mode <mode>     Clone mode: 'normal' (default) or 'force'"
  echo "  --help, -h        Show this help message and exit"
  echo ""
  echo "${BOLD}Examples:${RESET}"
  echo "  ./clone_iosonata_sdk_macos                  # Clone into ~/IOcomposer"
  echo "  ./clone_iosonata_sdk_macos --home ~/DevEnv  # Custom directory"
  echo "  ./clone_iosonata_sdk_macos --mode force     # Force re-clone all repos"
  echo ""
  echo "${BOLD}Repositories cloned (during normal execution):${RESET}"
  echo "  • IOsonata main repository"
  echo "  • Nordic Semiconductor nrfx"
  echo "  • nrfconnect SDKs (sdk-nrf-bm, sdk-nrfxlib)"
  echo "  • IOsonata nRF5 SDK & Mesh"
  echo "  • Bosch BSEC2 library"
  echo ""
  exit 0
}

# --- Default values ---
ROOT="$HOME/IOcomposer"
MODE="normal"

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
    --help|-h)
      print_help
      ;;
    *)
      echo "${RED}❌ Unknown argument:${RESET} $1"
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
echo "${BLUE}---------------------------------------------------------${RESET}"
echo ""

# --- Function: clone or update repo ---
clone_or_update_repo() {
  local repo_url="$1"
  local target_dir="$2"

  if [[ -d "$target_dir" ]]; then
    if [[ "$MODE" == "force" ]]; then
      echo "${YELLOW}⚠️  Re-cloning${RESET} $target_dir ..."
      rm -rf "$target_dir"
      git clone --depth=1 "$repo_url" "$target_dir"
    else
      echo "${YELLOW}🔄 Updating${RESET} $target_dir ..."
      (cd "$target_dir" && git pull --rebase)
    fi
  else
    echo "${BLUE}⬇️  Cloning${RESET} $target_dir ..."
    git clone --depth=1 "$repo_url" "$target_dir"
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

repos=(
  "https://github.com/NordicSemiconductor/nrfx.git"
  "https://github.com/nrfconnect/sdk-nrf-bm.git"
  "https://github.com/nrfconnect/sdk-nrfxlib.git"
  "https://github.com/IOsonata/nRF5_SDK.git"
  "https://github.com/IOsonata/nRF5_SDK_Mesh.git"
  "https://github.com/boschsensortec/Bosch-BSEC2-Library.git"
)

for repo in "${repos[@]}"; do
  name=$(basename "$repo" .git)
  [[ "$name" == "Bosch-BSEC2-Library" ]] && name="BSEC"
  target="$EXT/$name"
  clone_or_update_repo "$repo" "$target"
done

echo ""
echo "${GREEN}✅ All repositories cloned successfully!${RESET}"
echo "${BOLD}SDK Root:${RESET}      $ROOT"
echo "${BOLD}External Repos:${RESET} $EXT"
echo "${BLUE}---------------------------------------------------------${RESET}"
echo "${GREEN}Done. Happy building 🧠${RESET}"
echo ""
