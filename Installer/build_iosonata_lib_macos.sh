#!/usr/bin/env bash
set -euo pipefail

# =========================================================
#  build_iosonata_lib_macos.sh
# ---------------------------------------------------------
#  Purpose: Build IOsonata libraries for selected MCU
#  Platform: macOS
#  Version: v1.0.0
# =========================================================
#
# This script can be run standalone or called by installer/clone scripts.
# It will prompt for MCU selection and build the IOsonata library.
#
# Usage:
#   ./build_iosonata_lib_macos.sh [--home <path>]
#
# Examples:
#   ./build_iosonata_lib_macos.sh                    # Use default ~/IOcomposer
#   ./build_iosonata_lib_macos.sh --home ~/DevEnv   # Custom path
#
# =========================================================

SCRIPT_VERSION="v1.0.0"
ROOT="${HOME}/IOcomposer"
ECLIPSE_APP="/Applications/Eclipse.app"

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
      echo ""
      echo "This script will:"
      echo "  1. Discover available IOsonata Eclipse library projects"
      echo "  2. Present an interactive menu"
      echo "  3. Build Debug and Release configurations"
      echo "  4. Output libraries to project's Debug/Release directories"
      exit 0
      ;;
    *)
      echo "❌ Unknown argument: $1"
      echo "Use --help for usage information"
      exit 1
      ;;
  esac
done

echo "========================================================="
echo "  IOsonata Library Builder (macOS)"
echo "  Version: $SCRIPT_VERSION"
echo "========================================================="
echo

# Check Eclipse installation
if [[ ! -d "$ECLIPSE_APP" ]]; then
  echo "❌ ERROR: Eclipse not found at $ECLIPSE_APP"
  echo ""
  echo "Please install Eclipse Embedded CDT:"
  echo "  1. Run: ./install_iocdevtools_macos.sh"
  echo "  2. Or download from: https://eclipse.org/downloads/"
  exit 1
fi

if [[ ! -x "$ECLIPSE_APP/Contents/MacOS/eclipse" ]]; then
  echo "❌ ERROR: Eclipse executable not found or not executable"
  exit 1
fi

echo "✓ Eclipse found at: $ECLIPSE_APP"
echo

# Check IOsonata installation
if [[ ! -d "$ROOT/IOsonata" ]]; then
  echo "❌ ERROR: IOsonata directory not found at $ROOT/IOsonata"
  echo ""
  echo "Please clone IOsonata SDK:"
  echo "  ./clone_iosonata_sdk_macos.sh --home $ROOT"
  exit 1
fi

echo "✓ IOsonata SDK found at: $ROOT/IOsonata"
echo

# Discover available MCU library projects
mcu_families=()
mcu_paths=()

while IFS= read -r proj_file; do
  proj_root=$(dirname "$proj_file")
  rel_path="${proj_root#$ROOT/IOsonata/}"
  mcu_families+=("$rel_path")
  mcu_paths+=("$proj_root")
done < <(find "$ROOT/IOsonata" -type f -path "*/lib/Eclipse/.project" 2>/dev/null | sort)

if [[ ${#mcu_families[@]} -eq 0 ]]; then
  echo "⚠️  WARNING: No IOsonata Eclipse library projects found."
  echo ""
  echo "Expected to find .project files at: */lib/Eclipse/.project"
  echo "Searched in: $ROOT/IOsonata"
  exit 1
fi

echo "Available IOsonata library projects:"
echo
for i in "${!mcu_families[@]}"; do
  printf "  %2d) %s\n" $((i+1)) "${mcu_families[$i]}"
done
echo "   0) Exit"
echo

# Get user selection
selection=""
while true; do
  read -r -p "Select project to build (0-${#mcu_families[@]}): " selection
  if [[ "$selection" =~ ^[0-9]+$ ]] && [[ "$selection" -ge 0 ]] && [[ "$selection" -le ${#mcu_families[@]} ]]; then
    break
  fi
  echo "Invalid selection. Please try again."
done

if [[ "$selection" -eq 0 ]]; then
  echo "Exiting."
  exit 0
fi

# Get selected project details
selected_idx=$((selection - 1))
selected_family="${mcu_families[$selected_idx]}"
selected_path="${mcu_paths[$selected_idx]}"

# Validate project
if [[ ! -f "$selected_path/.project" || ! -f "$selected_path/.cproject" ]]; then
  echo "❌ ERROR: Selected path is not a valid Eclipse CDT project:"
  echo "   $selected_path"
  echo "   (missing .project or .cproject)"
  exit 1
fi

# Extract project name
proj_name=$(grep -m1 -oE '<name>[^<]+' "$selected_path/.project" | sed 's/<name>//' || true)
if [[ -z "${proj_name:-}" ]]; then
  echo "❌ ERROR: Could not determine project name from:"
  echo "   $selected_path/.project"
  exit 1
fi

echo
echo ">>> Building IOsonata libraries for: $selected_family"
echo "    Project: $proj_name"
echo "    Path:    $selected_path"
echo

# Create temp workspace
WS=$(mktemp -d /tmp/iosonata_build_ws.XXXX)
echo ">>> Using temp workspace: $WS"
echo ">>> Running Eclipse headless build (indexing disabled)..."
echo

# Prefer console launcher
ECL_BIN="$ECLIPSE_APP/Contents/MacOS/eclipsec"
if [[ ! -x "$ECL_BIN" ]]; then
  ECL_BIN="$ECLIPSE_APP/Contents/MacOS/eclipse"
fi

LOGFILE="/tmp/build_iosonata_lib.log"
rm -f "$LOGFILE" || true

# Run headless build with AWK filter to remove Java verbosity
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
  echo "❌ IOsonata library build failed for $selected_family (exit=$BUILD_EXIT)"
  echo "   Log: $LOGFILE"
  echo "   Temp workspace kept at: $WS"
  echo
  echo "Tip: The printed error markers above are the real compiler/linker failures."
  exit 1
fi

echo
echo "✅ IOsonata library build completed for $selected_family"
echo
echo "Libraries created:"
ls -lh "$selected_path/Debug/"libIOsonata*.a 2>/dev/null || echo "  (Debug configuration not found)"
ls -lh "$selected_path/Release/"libIOsonata*.a 2>/dev/null || echo "  (Release configuration not found)"
echo

# Cleanup temp workspace
rm -rf "$WS" || true

echo "========================================================="
echo "Build complete! You can now use these libraries in your"
echo "firmware projects."
echo "========================================================="
echo
