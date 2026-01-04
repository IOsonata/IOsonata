#!/usr/bin/env bash
set -euo pipefail

# =========================================================
#  build_iosonata_lib_linux.sh
# ---------------------------------------------------------
#  Purpose: Build IOsonata libraries for selected MCU
#  Platform: Linux
#  Version: v2.2.0
# =========================================================

SCRIPT_VERSION="v2.2.0"
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

# Discover projects (filter platform-specific)
mcu_families=()
mcu_paths=()

while IFS= read -r proj_file; do
  proj_root=$(dirname "$proj_file")
  rel_path="${proj_root#$ROOT/IOsonata/}"
  
  # Filter out Windows/macOS specific lib projects
  if [[ "$rel_path" =~ ^Win/lib/Eclipse$ ]] || \
     [[ "$rel_path" =~ /Win/lib/Eclipse$ ]] || \
     [[ "$rel_path" =~ ^Windows/lib/Eclipse$ ]] || \
     [[ "$rel_path" =~ /Windows/lib/Eclipse$ ]] || \
     [[ "$rel_path" =~ ^macOS/lib/Eclipse$ ]] || \
     [[ "$rel_path" =~ /macOS/lib/Eclipse$ ]]; then
    continue
  fi
  
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
echo "   A) Build All"
echo "   0) Exit"
echo

# User selection
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
  echo "Invalid selection."
done

if [[ "$selection" == "A" ]]; then
  # Build All selected, continue to build loop
  :
elif [[ "$selection" -eq 0 ]]; then
  echo "Exiting."
  exit 0
fi

# Function to build a single project
build_project() {
  local proj_path="$1"
  local proj_family="$2"
  
  # Validate
  if [[ ! -f "$proj_path/.project" || ! -f "$proj_path/.cproject" ]]; then
    echo "❌ ERROR: Not a valid Eclipse CDT project"
    return 1
  fi
  
  echo
  echo ">>> Building: $proj_family"
  echo "    Path: $proj_path"
  echo
  
  # Create workspace
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
  
  local LOGFILE="/tmp/build_iosonata_lib_$$.log"
  rm -f "$LOGFILE" || true
  
  # Build with AWK filter
  set +e
  "$ECL_BIN" \
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
  
  rm -rf "$WS" || true
  
  if [[ "$BUILD_EXIT" -ne 0 ]]; then
    echo
    echo "❌ Build failed for $proj_family (exit=$BUILD_EXIT)"
    echo "   Log: $LOGFILE"
    return 1
  fi
  
  echo
  echo "✅ Build completed for $proj_family"
  echo
  echo "Libraries:"
  ls -lh "$proj_path/Debug/"libIOsonata*.a 2>/dev/null || echo "  (Debug not found)"
  ls -lh "$proj_path/Release/"libIOsonata*.a 2>/dev/null || echo "  (Release not found)"
  echo
  
  return 0
}

# Build All or Single
if [[ "$selection" == "A" ]]; then
  echo
  echo "========================================================="
  echo "Building ALL projects (${#mcu_families[@]} total)"
  echo "========================================================="
  
  # Set up Ctrl-C handler
  interrupted=0
  trap 'interrupted=1' INT
  
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
    
    echo
    echo "─────────────────────────────────────────────────────────"
    echo "Building [$((i+1))/${#mcu_families[@]}]: ${mcu_families[$i]}"
    echo "─────────────────────────────────────────────────────────"
    
    if build_project "${mcu_paths[$i]}" "${mcu_families[$i]}"; then
      successful_builds+=("${mcu_families[$i]}")
    else
      failed_builds+=("${mcu_families[$i]}")
    fi
  done
  
  # Restore default signal handler
  trap - INT
  
  echo
  echo "========================================================="
  if [[ $interrupted -eq 1 ]]; then
    echo "Build All Summary (INTERRUPTED)"
  else
    echo "Build All Summary"
  fi
  echo "========================================================="
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
  
  if [[ $interrupted -eq 1 ]]; then
    echo
    echo "Build process was interrupted by user."
    exit 130  # Standard exit code for Ctrl-C
  fi
  
  if [[ ${#failed_builds[@]} -gt 0 ]]; then
    exit 1
  fi
  
else
  # Build single project
  selected_idx=$((selection - 1))
  selected_family="${mcu_families[$selected_idx]}"
  selected_path="${mcu_paths[$selected_idx]}"
  
  if ! build_project "$selected_path" "$selected_family"; then
    exit 1
  fi
fi

echo "========================================================="
echo "Build complete!"
echo "========================================================="
echo
