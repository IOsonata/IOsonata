#!/usr/bin/env bash
# =========================================================
#  clone_iosonata_sdk_linux.sh
# ---------------------------------------------------------
#  Purpose: Clone IOsonata SDK and dependencies (Arch/Ubuntu/Fedora)
#  Platform: Linux (Bash)
# =========================================================

set -Eeuo pipefail  # Safer error handling
IFS=$'\n\t'

# --- Define colors (ANSI safe for Arch) ---
RED=$(tput setaf 1 || echo "")
GREEN=$(tput setaf 2 || echo "")
YELLOW=$(tput setaf 3 || echo "")
BLUE=$(tput setaf 4 || echo "")
BOLD=$(tput bold || echo "")
RESET=$(tput sgr0 || echo "")

# --- Banner ---
print_banner() {
  echo -e "\n${BLUE}=========================================================${RESET}"
  echo -e "${BOLD}     🧩 IOsonata SDK Cloning Utility for Linux 🐧${RESET}"
  echo -e "${BLUE}=========================================================${RESET}"
}

# --- Help Menu ---
print_help() {
  print_banner
  echo -e "${BOLD}Usage:${RESET} clone_iosonata_sdk_linux.sh [--home <path>] [--mode <normal|force>] [--help]"
  echo ""
  echo -e "${BOLD}Options:${RESET}"
  echo "  --home <path>     Set custom root directory (default: ~/IOcomposer)"
  echo "  --mode <mode>     Clone mode: 'normal' (default) or 'force'"
  echo "  --help, -h        Show this help message and exit"
  echo ""
  echo -e "${BOLD}Examples:${RESET}"
  echo "  ./clone_iosonata_sdk_linux.sh"
  echo "  ./clone_iosonata_sdk_linux.sh --home ~/DevEnv"
  echo "  ./clone_iosonata_sdk_linux.sh --mode force"
  echo ""
  echo -e "${BOLD}Repositories cloned (during normal execution):${RESET}"
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
  case "$1" in
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
      echo -e "${RED}❌ Unknown argument:${RESET} $1"
      echo -e "Use '${YELLOW}--help${RESET}' for usage information."
      exit 1
      ;;
  esac
done

# --- Check for git ---
if ! command -v git >/dev/null 2>&1; then
  echo -e "${YELLOW}⚠️  Git is not installed.${RESET}"
  if [[ -f /etc/arch-release ]]; then
    read -rp "Install git via pacman? (y/N): " yn
    if [[ "$yn" =~ ^[Yy]$ ]]; then
      sudo pacman -Sy --noconfirm git
    else
      echo -e "${RED}Git is required. Aborting.${RESET}"
      exit 1
    fi
  elif command -v apt >/dev/null 2>&1; then
    read -rp "Install git via apt? (y/N): " yn
    if [[ "$yn" =~ ^[Yy]$ ]]; then
      sudo apt update && sudo apt install -y git
    else
      echo -e "${RED}Git is required. Aborting.${RESET}"
      exit 1
    fi
  else
    echo -e "${RED}Please install Git manually using your package manager.${RESET}"
    exit 1
  fi
fi

# --- Prepare directories ---
mkdir -p "$ROOT"
ROOT=$(cd "$ROOT" && pwd)
EXT="$ROOT/external"
mkdir -p "$EXT"

print_banner
echo -e "${BOLD}Root directory:${RESET} $ROOT"
echo -e "${BOLD}Mode:${RESET} $MODE"
echo -e "${BOLD}External path:${RESET} $EXT"
echo -e "${BLUE}---------------------------------------------------------${RESET}\n"

# --- Function: clone or update repo ---
clone_or_update_repo() {
  local repo_url="$1"
  local target_dir="$2"

  if [[ -d "$target_dir" ]]; then
    if [[ "$MODE" == "force" ]]; then
      echo -e "${YELLOW}⚠️  Re-cloning${RESET} $target_dir ..."
      rm -rf "$target_dir"
      git clone --depth=1 "$repo_url" "$target_dir"
    else
      echo -e "${YELLOW}🔄 Updating${RESET} $target_dir ..."
      (cd "$target_dir" && git pull --rebase)
    fi
  else
    echo -e "${BLUE}⬇️  Cloning${RESET} $target_dir ..."
    git clone --depth=1 "$repo_url" "$target_dir"
  fi
}

# ---------------------------------------------------------
# Clone IOsonata
# ---------------------------------------------------------
echo -e "${GREEN}🚀 Cloning IOsonata...${RESET}"
clone_or_update_repo "https://github.com/IOsonata/IOsonata.git" "$ROOT/IOsonata"

# ---------------------------------------------------------
# Clone External Repos
# ---------------------------------------------------------
echo -e "\n${BLUE}📦 Cloning dependencies into:${RESET} $EXT\n"

repos=(
  "https://github.com/NordicSemiconductor/nrfx.git"
  "https://github.com/nrfconnect/sdk-nrf-bm.git"
  "https://github.com/nrfconnect/sdk-nrfxlib.git"
  "https://github.com/IOsonata/nRF5_SDK.git"
  "https://github.com/IOsonata/nRF5_SDK_Mesh.git"
  "https://github.com/boschsensortec/Bosch-BSEC2-Library.git"
  "https://github.com/xioTechnologies/Fusion.git"
)

for repo in "${repos[@]}"; do
  name=$(basename "$repo" .git)
  [[ "$name" == "Bosch-BSEC2-Library" ]] && name="BSEC"
  target="$EXT/$name"
  clone_or_update_repo "$repo" "$target"
done

echo -e "\n${GREEN}✅ All repositories cloned successfully!${RESET}"
echo -e "${BOLD}SDK Root:${RESET}      $ROOT"
echo -e "${BOLD}External Repos:${RESET} $EXT"
echo -e "${BLUE}---------------------------------------------------------${RESET}"
echo -e "${GREEN}Done. Happy building 🧠${RESET}\n"
