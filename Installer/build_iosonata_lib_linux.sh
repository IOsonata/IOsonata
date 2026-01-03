#!/usr/bin/env bash
set -euo pipefail

# =========================================================
#  build_iosonata_lib_linux.sh
# ---------------------------------------------------------
#  Purpose: Build IOsonata libraries for selected MCU
#  Platform: Linux
#  Version: v1.0.0
# =========================================================

SCRIPT_VERSION="v1.0.0"
ROOT="${HOME}/IOcomposer"
ECLIPSE_DIR="/opt/eclipse"

# Parse arguments
while [[ $# -gt 0 ]]; do
  case "$1" in
    --home)
      shift
      if [[ -z "${1:-}" ]]; then
        echo "❌ Missing path after --home"
        exit 1
      fi
      ROOT="$1"
      shift
      ;;
    --help|-h)
      echo "Usage: $0 [--home <path>]"
      echo ""
      echo "Build IOsonata libraries for selected MCU target."
      echo ""
      echo "Options:"
      echo "  --home <path>    Set IOsonata SDK root (default: ~/IOcomposer)"
      echo "  --help, -h       Show this help"
      exit 0
      ;;
    *)
      echo "❌ Unknown argument: $1"
      exit 1
      ;;
  esac
done

echo "========================================================="
echo "  IOsonata Library Builder (Linux)"
echo "  Version: $SCRIPT_VERSION"
echo "========================================================="
echo

# Check Eclipse installation
if [[ ! -d "$ECLIPSE_DIR" ]]; then
  echo "❌ ERROR: Eclipse not found at $ECLIPSE_DIR"
  echo ""
  echo "Please install Eclipse: ./install_iocdevtools_linux.sh"
  exit 1
fi

if [[ ! -x "$ECLIPSE_DIR/eclipse" ]]; then
  echo "❌ ERROR: Eclipse executable not found"
  exit 1
fi

echo "✓ Eclipse found at: $ECLIPSE_DIR"
echo

# Check IOsonata
if [[ ! -d "$ROOT/IOsonata" ]]; then
  echo "❌ ERROR: IOsonata not found at $ROOT/IOsonata"
  echo ""
  echo "Please clone: ./clone_iosonata_sdk_linux.sh --home $ROOT"
  exit 1
fi

echo "✓ IOsonata SDK found at: $ROOT/IOsonata"
echo

# Discover projects
mcu_families=()
mcu_paths=()

while IFS= read -r proj_file; do
  proj_root=$(dirname "$proj_file")
  rel_path="${proj_root#$ROOT/IOsonata/}"
  mcu_families+=("$rel_path")
  mcu_paths+=("$proj_root")
done < <(find "$ROOT/IOsonata" -type f -path "*/lib/Eclipse/.project" 2>/dev/null | sort)

if [[ ${#mcu_families[@]} -eq 0 ]]; then
  echo "⚠️  No IOsonata Eclipse projects found"
  exit 1
fi

echo "Available projects:"
echo
for i in "${!mcu_families[@]}"; do
  printf "  %2d) %s\n" $((i+1)) "${mcu_families[$i]}"
done
echo "   0) Exit"
echo

# User selection
selection=""
while true; do
  read -r -p "Select project to build (0-${#mcu_families[@]}): " selection
  if [[ "$selection" =~ ^[0-9]+$ ]] && [[ "$selection" -ge 0 ]] && [[ "$selection" -le ${#mcu_families[@]} ]]; then
    break
  fi
  echo "Invalid selection."
done

if [[ "$selection" -eq 0 ]]; then
  echo "Exiting."
  exit 0
fi

# Get selection
selected_idx=$((selection - 1))
selected_family="${mcu_families[$selected_idx]}"
selected_path="${mcu_paths[$selected_idx]}"

# Validate
if [[ ! -f "$selected_path/.project" || ! -f "$selected_path/.cproject" ]]; then
  echo "❌ ERROR: Not a valid Eclipse CDT project"
  exit 1
fi

# Extract project name
proj_name=$(grep -m1 -oE '<n>[^<]+' "$selected_path/.project" | sed 's/<n>//' || true)
if [[ -z "${proj_name:-}" ]]; then
  echo "❌ ERROR: Could not determine project name"
  exit 1
fi

echo
echo ">>> Building: $selected_family"
echo "    Project: $proj_name"
echo "    Path:    $selected_path"
echo

# Create workspace
WS=$(mktemp -d /tmp/iosonata_build_ws.XXXX)
echo ">>> Workspace: $WS"
echo ">>> Running Eclipse headless build..."
echo

# Prefer console launcher
ECL_BIN="$ECLIPSE_DIR/eclipsec"
if [[ ! -x "$ECL_BIN" ]]; then
  ECL_BIN="$ECLIPSE_DIR/eclipse"
fi

LOGFILE="/tmp/build_iosonata_lib.log"
rm -f "$LOGFILE" || true

# Build with AWK filter
set +e
"$ECL_BIN" \
  --launcher.suppressErrors \
  -nosplash \
  -application org.eclipse.cdt.managedbuilder.core.headlessbuild \
  -data "$WS" \
  -no-indexer \
  -import "$selected_path" \
  -cleanBuild "${proj_name}/.*" \
  -printErrorMarkers \
  2>&1 \
  | tee "$LOGFILE" \
  | awk 'BEGIN{drop=0} /^Java was started but returned exit code=/{drop=1} drop==0{print}'
BUILD_EXIT=${PIPESTATUS[0]}
set -e

if [[ "$BUILD_EXIT" -ne 0 ]]; then
  echo
  echo "❌ Build failed (exit=$BUILD_EXIT)"
  echo "   Log: $LOGFILE"
  echo "   Workspace: $WS"
  exit 1
fi

echo
echo "✅ Build completed for $selected_family"
echo
echo "Libraries:"
ls -lh "$selected_path/Debug/"libIOsonata*.a 2>/dev/null || echo "  (Debug not found)"
ls -lh "$selected_path/Release/"libIOsonata*.a 2>/dev/null || echo "  (Release not found)"
echo

rm -rf "$WS" || true

echo "========================================================="
echo "Build complete!"
echo "========================================================="
echo
