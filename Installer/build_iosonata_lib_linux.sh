#!/usr/bin/env bash
set -euo pipefail

# =========================================================
#  build_iosonata_lib_linux.sh
# ---------------------------------------------------------
#  Purpose: Build IOsonata libraries for selected MCU
#  Platform: Linux
#  Version: v2.2.0
# =========================================================

# ---------------------------------------------------------
# CONFIGURATION VARIABLES
# ---------------------------------------------------------

SCRIPT_VERSION="v2.2.0"
ROOT="${HOME}/IOcomposer"
ECLIPSE_DIR="/opt/eclipse"

# ---------------------------------------------------------
# ARGUMENT PARSING
# ---------------------------------------------------------

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
    # Added this to specify directory for eclipse installation
    --eclipse)
      shift
        if [[ -z "${1:-}" ]]; then
          echo "❌ Missing path after --eclipse"
          exit 1
        fi
        ECLIPSE_DIR="$1"
      shift
      ;;
    --help|-h)
      echo "Usage: $0 [--home <path>] [--eclipse <path>]"
      echo ""
      echo "Build IOsonata libraries for selected MCU target."
      echo ""
      echo "Options:"
      echo "  --home <path>      Set IOsonata SDK root (default: ~/IOcomposer)"
      echo "  --eclipse <path>   Set Eclipse install dir (default: ~/eclipse/embedcdt)"
      echo "  --help, -h         Show this help"
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

# ---------------------------------------------------------
# BANNER DISPLAY
# ---------------------------------------------------------

echo "========================================================="
echo "  IOsonata Library Builder (Linux)"
echo "  Version: $SCRIPT_VERSION"
echo "========================================================="
echo

# ------------------------------------------------------------
# ECLIPSE VALIDATION (Finding where the Eclipse executable is)
# ------------------------------------------------------------
# Linux doesn't have a standard Eclipse install location, so we test it in common locations

ECLIPSE_BIN=""
if [[ -x "$ECLIPSE_DIR/eclipse" ]]; then
  ECLIPSE_BIN="$ECLIPSE_DIR/eclipse"
elif [[ -x "$ECLIPSE_DIR/eclipsec" ]]; then
  ECLIPSE_BIN="$ECLIPSE_DIR/eclipsec"
elif command -v eclipse &>/dev/null; then
  ECLIPSE_BIN=$(command -v eclipse)
  ECLIPSE_DIR=$(dirname "$ECLIPSE_BIN")
elif [[ -x "/opt/eclipse/eclipse" ]]; then
  ECLIPSE_BIN="/opt/eclipse/eclipse"
  ECLIPSE_DIR="/opt/eclipse"
elif [[ -x "/usr/local/eclipse/eclipse" ]]; then
  ECLIPSE_BIN="/usr/local/eclipse/eclipse"
  ECLIPSE_DIR="/usr/local/eclipse"
elif [[ -x "/snap/eclipse/current/eclipse" ]]; then
  ECLIPSE_BIN="/snap/eclipse/current/eclipse"
  ECLIPSE_DIR="/snap/eclipse/current"
fi

if [[ -z "$ECLIPSE_BIN" ]]; then
  echo "❌ ERROR: Eclipse not found"
  echo ""
  echo "Searched locations:"
  echo "  - $ECLIPSE_DIR/eclipse"
  echo "  - /opt/eclipse/eclipse"
  echo "  - /usr/local/eclipse/eclipse"
  echo "  - System PATH"
  echo ""
  echo "Please install Eclipse Embedded CDT:"
  echo "  1. Run: ./install_iocdevtools_linux.sh"
  echo "  2. Or download from: https://eclipse.org/downloads/"
  echo "  3. Or specify location: $0 --eclipse /path/to/eclipse"
  exit 1
fi

echo "✓ Eclipse found at: $ECLIPSE_BIN"
echo

# ---------------------------------------------------------
# IOSONATA SDK VALIDATION
# ---------------------------------------------------------

if [[ ! -d "$ROOT/IOsonata" ]]; then
  echo "❌ ERROR: IOsonata directory not found at $ROOT/IOsonata"
  echo ""
  echo "Please clone IOsonata SDK:"
  echo "  ./clone_iosonata_sdk_linux.sh --home $ROOT"
  exit 1
fi

echo "✓ IOsonata SDK found at: $ROOT/IOsonata"
echo

# ---------------------------------------------------------
# PROJECT DISCOVERY
# ---------------------------------------------------------

mcu_families=()
mcu_paths=()

while IFS= read -r proj_file; do
  proj_root=$(dirname "$proj_file")
  rel_path="${proj_root#$ROOT/IOsonata}"

  # Ignores Windows/macOS specific lib projects
  if [[ "$rel_path" =~ ^Win/lib/Eclipse$ ]] || \
     [[ "$rel_path" =~ /Win/lib/Eclipse$ ]] || \
     [[ "$rel_path" =~ ^Windows/lib/Eclipse$ ]] || \
     [[ "$rel_path" =~ /Windows/lib/Eclipse$ ]] || \
     [[ "$rel_path" =~ ^macOS/lib/Eclipse$ ]] || \
     [[ "$rel_path" =~ /macOS/lib/Eclipse$ ]] || \
     [[ "$rel_path" =~ ^OSX/lib/Eclipse$ ]] || \
     [[ "$rel_path" =~ /OSX/lib/Eclipse$ ]]; then
    continue
  fi

  # Add to arrays
  mcu_families+=("$rel_path")
  mcu_paths+=("$proj_root")

done < <(find "$ROOT/IOsonata" -type f -ipath "*lib/eclipse/.project" 2>/dev/null | sort)

# ---------------------------------------------------------
# EMPTY RESULTS CHECK
# ---------------------------------------------------------

if [[ ${#mcu_families[@]} -eq 0 ]]; then
  echo "⚠️  WARNING: No IOsonata Eclipse library projects found."
  echo ""
  echo "Expected to find .project files at: */lib/Eclipse/.project"
  echo "Searched in: $ROOT/IOsonata"
  exit 1
fi

# ---------------------------------------------------------
# MENU DISPLAY
# ---------------------------------------------------------

echo "Available IOsonata library projects:"
echo
for i in "${!mcu_families[@]}"; do
  printf "  %2d) %s\n" $((i+1)) "${mcu_families[$i]}"
done
echo "   A) Build All"
echo "   0) Exit"
echo

# ---------------------------------------------------------
# USER INPUT HANDLING
# ---------------------------------------------------------

selection=""
while true; do
  read -r -p "Select project to build (0-${#mcu_families[@]} or A): " selection
  selection_upper=$(echo "$selection" | tr '[:lower:]' '[:upper:]')
  
  if [[ "$selection_upper" == "A" ]]; then
    selection="A"
    break
  elif [[ "$selection" =~ ^[0-9]+$ ]] && [[ "$selection" -ge 0 ]] && [[ "$selection" -le ${#mcu_families[@]} ]]; then
    break
  fi
  echo "Invalid selection. Please try again."
done

if [[ "$selection" == "A" ]]; then
  :  # Build All selected, continue
elif [[ "$selection" -eq 0 ]]; then
  echo "Exiting."
  exit 0
fi

# ---------------------------------------------------------
# BUILD FUNCTION
# ---------------------------------------------------------

build_project() {
  local proj_path="$1"
  local proj_family="$2"

  # Validate project
  if [[ ! -f "$proj_path/.project" || ! -f "$proj_path/.cproject" ]]; then
    echo "❌ ERROR: Selected path is not a valid Eclipse CDT project:"
    echo "   $proj_path"
    echo "   (missing .project or .cproject)"
    return 1
  fi

  echo
  echo ">>> Building IOsonata libraries for: $proj_family"
  echo "    Path: $proj_path"
  echo

  # Create temp workspace
  local WS
  WS=$(mktemp -d /tmp/iosonata_build_ws.XXXX)
  echo ">>> Workspace: $WS"
  echo ">>> Running Eclipse headless build..."
  echo

  # Prefer console launcher
  local ECL_BIN="$ECLIPSE_DIR/eclipsec"
  if [[ ! -x "$ECL_BIN" ]]; then
    ECL_BIN="$ECLIPSE_DIR/eclipse"
  fi

  # Log File Setup
  local LOGFILE="/tmp/build_iosonata_lib_$$.log"
  rm -f "$LOGFILE" || true

  # Run headless build with AWK filter
  set +e
  "$ECLIPSE_BIN" \
    --launcher.suppressErrors \
    -nosplash \
    -application org.eclipse.cdt.managedbuilder.core.headlessbuild \
    -data "$WS" \
    -no-indexer \
    -import "$proj_path" \
    -cleanBuild all \
    -printErrorMarkers \
    2>&1 \
    | tee "$LOGFILE" \
    | awk 'BEGIN{drop=0} /^Java was started but returned exit code=/{drop=1} drop==0{print}'
  local BUILD_EXIT=${PIPESTATUS[0]}
  set -e

  # Cleanup temp workspace
  rm -rf "$WS" || true

  # Result checking
  if [[ "$BUILD_EXIT" -ne 0 ]]; then
    echo
    echo "❌ IOsonata library build failed for $proj_family (exit=$BUILD_EXIT)"
    echo "   Log: $LOGFILE"
    echo
    echo "Tip: The printed error markers above are the real compiler/linker failures."
    return 1
  fi

  # Success Reporting
  echo
  echo "✅ IOsonata library build completed for $proj_family"
  echo
  echo "Libraries created:"
  ls -lh "$proj_path/Debug/"libIOsonata*.a 2>/dev/null || echo "  (Debug configuration not found)"
  ls -lh "$proj_path/Release/"libIOsonata*.a 2>/dev/null || echo "  (Release configuration not found)"
  echo
  
  return 0
}

# ---------------------------------------------------------
# BUILD ALL LOGIC
# ---------------------------------------------------------

if [[ "$selection" == "A" ]]; then
  # Banner
  echo
  echo "========================================================="
  echo "Building ALL projects (${#mcu_families[@]} total)"
  echo "========================================================="

  # Set up Ctrl-C handler (User Cancellation)
  interrupted=0
  trap 'interrupted=1' INT

  # Success Tracking in arrays
  failed_builds=()
  successful_builds=()

  for i in "${!mcu_families[@]}"; do
    # Check if interrupted
    if [[ $interrupted -eq 1 ]]; then
      echo
      echo "========================================================="
      echo "Build interrupted by user (Ctrl-C)"
      echo "========================================================="
      break
    fi

    # Banner
    echo
    echo "─────────────────────────────────────────────────────────"
    echo "Building [$((i+1))/${#mcu_families[@]}]: ${mcu_families[$i]}"
    echo "─────────────────────────────────────────────────────────"

    # Result Classification
    if build_project "${mcu_paths[$i]}" "${mcu_families[$i]}"; then
      successful_builds+=("${mcu_families[$i]}")
    else
      failed_builds+=("${mcu_families[$i]}")
    fi
  done

  # Restore default signal handler
  trap - INT

  # Banner
  echo
  echo "========================================================="
  if [[ $interrupted -eq 1 ]]; then
    echo "Build All Summary (INTERRUPTED)"
  else
    echo "Build All Summary"
  fi
  echo "========================================================="

  # Summary Output
  echo "✅ Successful: ${#successful_builds[@]}/${#mcu_families[@]}"
  for proj in "${successful_builds[@]}"; do
    echo "   ✓ $proj"
  done

  if [[ ${#failed_builds[@]} -gt 0 ]]; then
    echo
    echo "❌ Failed: ${#failed_builds[@]}/${#mcu_families[@]}"
    for proj in "${failed_builds[@]}"; do
      echo "   ✗ $proj"
    done
    echo
    echo "Check individual log files in /tmp/"
  fi
  
  # Exit Code Handling
  if [[ $interrupted -eq 1 ]]; then
    echo
    echo "Build process was interrupted by user."
    exit 130
  fi
  
  if [[ ${#failed_builds[@]} -gt 0 ]]; then
    exit 1
  fi

# ---------------------------------------------------------
# SINGLE BUILD EXECUTION
# ---------------------------------------------------------

else
  selected_idx=$((selection - 1))
  selected_family="${mcu_families[$selected_idx]}"
  selected_path="${mcu_paths[$selected_idx]}"
  
  if ! build_project "$selected_path" "$selected_family"; then
    exit 1
  fi
fi

# ---------------------------------------------------------
# COMPLETION MESSAGE
# ---------------------------------------------------------

echo "========================================================="
echo "Build complete! You can now use these libraries in your"
echo "firmware projects."
echo "========================================================="
echo