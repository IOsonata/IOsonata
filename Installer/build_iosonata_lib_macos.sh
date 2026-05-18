#!/usr/bin/env bash
set -euo pipefail

# =========================================================
#  build_iosonata_lib_macos.sh
# ---------------------------------------------------------
#  Purpose: Build IOsonata libraries for selected MCU
#           and all TaktOS ARM + RISCV libraries
#  Platform: macOS
#  Version: v2.4.0
#
#  v2.4.0: Build all TaktOS lib projects (ARM + RISCV) by
#          default after the IOsonata build. Lib projects
#          only, Benchmark/KVB/test projects are skipped.
#          Added --taktos and --no-taktos options.
#  v2.3.0: Prefer IOcomposer; fall back to Eclipse if not
#          found. IOcomposer is Eclipse-based, so the same
#          headless build application is used regardless.
# =========================================================
#
# This script can be run standalone or called by installer/clone scripts.
# It will prompt for IOsonata MCU selection and build the libraries.
#
# Usage:
#   ./build_iosonata_lib_macos.sh [--home <path>] [--iocomposer <path>] [--eclipse <path>] [--taktos <path>] [--no-taktos]
#
# Examples:
#   ./build_iosonata_lib_macos.sh                    # Use default ~/IOcomposer
#   ./build_iosonata_lib_macos.sh --home ~/DevEnv   # Custom path
#
# =========================================================

SCRIPT_VERSION="v2.4.0"
ROOT="${HOME}/IOcomposer"
IOCOMPOSER_APP="/Applications/IOcomposer.app"
ECLIPSE_APP="/Applications/Eclipse.app"

# TaktOS source dir. Empty means derive from ROOT after
# argument parsing (ROOT/TaktOS, cloned next to IOsonata).
TAKTOS_DIR=""
NO_TAKTOS=0

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
    --iocomposer)
      shift
      if [[ -z "${1:-}" ]]; then
        echo "❌ Missing path after --iocomposer"
        exit 1
      fi
      IOCOMPOSER_APP="$1"
      shift
      ;;
    --eclipse)
      shift
      if [[ -z "${1:-}" ]]; then
        echo "❌ Missing path after --eclipse"
        exit 1
      fi
      ECLIPSE_APP="$1"
      shift
      ;;
    --taktos)
      shift
      if [[ -z "${1:-}" ]]; then
        echo "❌ Missing path after --taktos"
        exit 1
      fi
      TAKTOS_DIR="$1"
      shift
      ;;
    --no-taktos)
      NO_TAKTOS=1
      shift
      ;;
    --help|-h)
      echo "Usage: $0 [--home <path>] [--iocomposer <path>] [--eclipse <path>] [--taktos <path>] [--no-taktos]"
      echo ""
      echo "Build IOsonata libraries for selected MCU target,"
      echo "then build all TaktOS ARM + RISCV libraries."
      echo ""
      echo "Options:"
      echo "  --home <path>         Set SDK root (default: ~/IOcomposer)"
      echo "  --iocomposer <path>   Set IOcomposer.app path (default: /Applications/IOcomposer.app)"
      echo "  --eclipse <path>      Set Eclipse.app path (fallback) (default: /Applications/Eclipse.app)"
      echo "  --taktos <path>       Set TaktOS source dir (default: <home>/TaktOS)"
      echo "  --no-taktos           Do not build TaktOS, IOsonata only"
      echo "  --help, -h            Show this help"
      echo ""
      echo "This script will:"
      echo "  1. Locate IOcomposer (preferred) or Eclipse Embedded CDT"
      echo "  2. Discover available IOsonata Eclipse library projects"
      echo "  3. Present an interactive menu for IOsonata"
      echo "  4. Discover TaktOS ARM + RISCV library projects (lib only)"
      echo "  5. Build Debug and Release configurations"
      echo "  6. Output libraries to each project's Debug/Release directories"
      exit 0
      ;;
    *)
      echo "❌ Unknown argument: $1"
      echo "Use --help for usage information"
      exit 1
      ;;
  esac
done

# Derive TaktOS dir from final ROOT unless set explicitly
if [[ -z "$TAKTOS_DIR" ]]; then
  TAKTOS_DIR="$ROOT/TaktOS"
fi

echo "========================================================="
echo "  IOsonata + TaktOS Library Builder (macOS)"
echo "  Version: $SCRIPT_VERSION"
echo "========================================================="
echo

# ---------------------------------------------------------
# IDE DETECTION (prefer IOcomposer, fall back to Eclipse)
# ---------------------------------------------------------
# IOcomposer is Eclipse-based. Inside the .app bundle the
# launcher may be named 'iocomposer'/'iocomposerc' or still
# 'eclipse'/'eclipsec'. Try both.

IDE_BIN=""
IDE_APP=""
IDE_NAME=""

# Helper: try to locate a usable launcher inside an .app bundle.
# Sets IDE_BIN/IDE_APP/IDE_NAME on success.
find_launcher_in_app() {
  local app="$1"
  local name="$2"
  local exe
  if [[ ! -d "$app" ]]; then
    return 1
  fi
  for exe in iocomposerc iocomposer eclipsec eclipse; do
    if [[ -x "$app/Contents/MacOS/$exe" ]]; then
      IDE_BIN="$app/Contents/MacOS/$exe"
      IDE_APP="$app"
      IDE_NAME="$name"
      return 0
    fi
  done
  return 1
}

# IOcomposer candidate locations
iocomposer_candidates=(
  "$IOCOMPOSER_APP"
  "$ROOT/IOcomposer.app"
  "$HOME/Applications/IOcomposer.app"
  "/Applications/IOcomposer.app"
)

for app in "${iocomposer_candidates[@]}"; do
  if find_launcher_in_app "$app" "IOcomposer"; then
    break
  fi
done

# Fall back to Eclipse
if [[ -z "$IDE_BIN" ]]; then
  eclipse_candidates=(
    "$ECLIPSE_APP"
    "$HOME/Applications/Eclipse.app"
    "/Applications/Eclipse.app"
  )
  for app in "${eclipse_candidates[@]}"; do
    if find_launcher_in_app "$app" "Eclipse"; then
      break
    fi
  done
fi

if [[ -z "$IDE_BIN" ]]; then
  echo "❌ ERROR: Neither IOcomposer nor Eclipse found"
  echo ""
  echo "Searched IOcomposer locations:"
  for app in "${iocomposer_candidates[@]}"; do
    echo "  - $app"
  done
  echo ""
  echo "Searched Eclipse locations:"
  echo "  - $ECLIPSE_APP"
  echo "  - $HOME/Applications/Eclipse.app"
  echo "  - /Applications/Eclipse.app"
  echo ""
  echo "Please install IOcomposer (preferred) or Eclipse Embedded CDT:"
  echo "  1. Run: ./install_iocdevtools_macos.sh"
  echo "  2. Or specify location: $0 --iocomposer /path/to/IOcomposer.app"
  echo "  3. Or:                  $0 --eclipse     /path/to/Eclipse.app"
  exit 1
fi

echo "✓ $IDE_NAME found at: $IDE_APP"
echo "  Launcher: $IDE_BIN"
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

# Discover available IOsonata MCU library projects (filter platform-specific)
mcu_families=()
mcu_paths=()

while IFS= read -r proj_file; do
  proj_root=$(dirname "$proj_file")
  rel_path="${proj_root#$ROOT/IOsonata/}"

  # Filter out Windows/Linux specific lib projects
  if [[ "$rel_path" =~ ^Win/lib/Eclipse$ ]] || \
     [[ "$rel_path" =~ /Win/lib/Eclipse$ ]] || \
     [[ "$rel_path" =~ ^Windows/lib/Eclipse$ ]] || \
     [[ "$rel_path" =~ /Windows/lib/Eclipse$ ]] || \
     [[ "$rel_path" =~ ^Linux/lib/Eclipse$ ]] || \
     [[ "$rel_path" =~ /Linux/lib/Eclipse$ ]]; then
    continue
  fi

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

# ---------------------------------------------------------
# TAKTOS PROJECT DISCOVERY (lib only: ARM + RISCV)
# ---------------------------------------------------------
# TaktOS library projects live at:
#   <TaktOS>/ARM/<core>/Eclipse
#   <TaktOS>/RISCV/<core>/Eclipse
# Everything under Benchmark/, KVB/ and test/ is skipped
# because those relative paths do not match the predicate.

taktos_families=()
taktos_paths=()

if [[ "$NO_TAKTOS" -eq 1 ]]; then
  echo "TaktOS build disabled (--no-taktos)"
  echo
elif [[ ! -d "$TAKTOS_DIR" ]]; then
  echo "⚠️  WARNING: TaktOS directory not found at $TAKTOS_DIR"
  echo "   IOsonata will still be built. Use --taktos <path> to set it,"
  echo "   or --no-taktos to silence this warning."
  echo
else
  while IFS= read -r proj_file; do
    proj_root=$(dirname "$proj_file")
    rel="${proj_root#$TAKTOS_DIR/}"

    # Keep only top level ARM/<core>/Eclipse or RISCV/<core>/Eclipse
    if [[ "$rel" =~ ^ARM/[^/]+/Eclipse$ ]] || \
       [[ "$rel" =~ ^RISCV/[^/]+/Eclipse$ ]]; then
      taktos_families+=("$rel")
      taktos_paths+=("$proj_root")
    fi
  done < <(find "$TAKTOS_DIR" -type f -name ".project" 2>/dev/null | sort)

  if [[ ${#taktos_families[@]} -eq 0 ]]; then
    echo "⚠️  WARNING: No TaktOS ARM/RISCV library projects found in $TAKTOS_DIR"
    echo
  else
    echo "✓ TaktOS found at: $TAKTOS_DIR (${#taktos_families[@]} lib projects)"
    echo
  fi
fi

echo "Available IOsonata library projects:"
echo
for i in "${!mcu_families[@]}"; do
  printf "  %2d) %s\n" $((i+1)) "${mcu_families[$i]}"
done
echo "   A) Build All"
echo "   0) Exit"
echo
if [[ ${#taktos_families[@]} -gt 0 ]]; then
  echo "Note: all ${#taktos_families[@]} TaktOS ARM/RISCV libraries are built automatically."
  echo
fi

# Get user selection
selection=""
while true; do
  read -r -p "Select IOsonata project to build (0-${#mcu_families[@]} or A): " selection
  selection_upper=$(echo "$selection" | tr '[:lower:]' '[:upper:]')

  if [[ "$selection_upper" == "A" ]]; then
    selection="A"
    break
  elif [[ "$selection" =~ ^[0-9]+$ ]] && [[ "$selection" -ge 0 ]] && [[ "$selection" -le ${#mcu_families[@]} ]]; then
    break
  fi
  echo "Invalid selection. Please try again."
done

if [[ "$selection" != "A" ]] && [[ "$selection" -eq 0 ]]; then
  echo "Exiting."
  exit 0
fi

# Function to build a single project
build_project() {
  local proj_path="$1"
  local proj_family="$2"
  local lib_glob="${3:-libIOsonata*.a}"

  # Validate project
  if [[ ! -f "$proj_path/.project" || ! -f "$proj_path/.cproject" ]]; then
    echo "❌ ERROR: Selected path is not a valid Eclipse CDT project:"
    echo "   $proj_path"
    echo "   (missing .project or .cproject)"
    return 1
  fi

  echo
  echo ">>> Building libraries for: $proj_family"
  echo "    Path: $proj_path"
  echo

  # Create temp workspace
  local WS
  WS=$(mktemp -d /tmp/iosonata_build_ws.XXXX)
  echo ">>> Using temp workspace: $WS"
  echo ">>> Running $IDE_NAME headless build (indexing disabled)..."
  echo

  local LOGFILE="/tmp/build_lib_$$.log"
  rm -f "$LOGFILE" || true

  # Run headless build with AWK filter to remove Java verbosity
  set +e
  "$IDE_BIN" \
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

  if [[ "$BUILD_EXIT" -ne 0 ]]; then
    echo
    echo "❌ Library build failed for $proj_family (exit=$BUILD_EXIT)"
    echo "   Log: $LOGFILE"
    echo
    echo "Tip: The printed error markers above are the real compiler/linker failures."
    return 1
  fi

  echo
  echo "✅ Library build completed for $proj_family"
  echo
  echo "Libraries created:"
  ls -lh "$proj_path/Debug/"$lib_glob 2>/dev/null || echo "  (Debug configuration not found)"
  ls -lh "$proj_path/Release/"$lib_glob 2>/dev/null || echo "  (Release configuration not found)"
  echo

  return 0
}

# ---------------------------------------------------------
# BUILD WORK LIST
# ---------------------------------------------------------
# IOsonata selection (single or all) followed by every
# TaktOS ARM/RISCV library project.

work_names=()
work_paths=()
work_globs=()

if [[ "$selection" == "A" ]]; then
  for i in "${!mcu_families[@]}"; do
    work_names+=("IOsonata/${mcu_families[$i]}")
    work_paths+=("${mcu_paths[$i]}")
    work_globs+=("libIOsonata*.a")
  done
else
  sel_idx=$((selection - 1))
  work_names+=("IOsonata/${mcu_families[$sel_idx]}")
  work_paths+=("${mcu_paths[$sel_idx]}")
  work_globs+=("libIOsonata*.a")
fi

if [[ ${#taktos_families[@]} -gt 0 ]]; then
  for i in "${!taktos_families[@]}"; do
    work_names+=("TaktOS/${taktos_families[$i]}")
    work_paths+=("${taktos_paths[$i]}")
    work_globs+=("libTaktOS*.a")
  done
fi

# ---------------------------------------------------------
# BUILD EXECUTION
# ---------------------------------------------------------

echo
echo "========================================================="
echo "Building ${#work_names[@]} project(s)"
echo "========================================================="

# Set up Ctrl-C handler
interrupted=0
trap 'interrupted=1' INT

failed_builds=()
successful_builds=()

for i in "${!work_names[@]}"; do
  if [[ $interrupted -eq 1 ]]; then
    echo
    echo "========================================================="
    echo "Build interrupted by user (Ctrl-C)"
    echo "========================================================="
    break
  fi

  echo
  echo "---------------------------------------------------------"
  echo "Building [$((i+1))/${#work_names[@]}]: ${work_names[$i]}"
  echo "---------------------------------------------------------"

  if build_project "${work_paths[$i]}" "${work_names[$i]}" "${work_globs[$i]}"; then
    successful_builds+=("${work_names[$i]}")
  else
    failed_builds+=("${work_names[$i]}")
  fi
done

# Restore default signal handler
trap - INT

# ---------------------------------------------------------
# SUMMARY
# ---------------------------------------------------------

echo
echo "========================================================="
if [[ $interrupted -eq 1 ]]; then
  echo "Build Summary (INTERRUPTED)"
else
  echo "Build Summary"
fi
echo "========================================================="
echo "Successful: ${#successful_builds[@]}/${#work_names[@]}"
for proj in "${successful_builds[@]:-}"; do
  [[ -n "$proj" ]] && echo "   + $proj"
done

if [[ ${#failed_builds[@]} -gt 0 ]]; then
  echo
  echo "Failed: ${#failed_builds[@]}/${#work_names[@]}"
  for proj in "${failed_builds[@]}"; do
    echo "   - $proj"
  done
  echo
  echo "Check the build log in /tmp/"
fi

if [[ $interrupted -eq 1 ]]; then
  echo
  echo "Build process was interrupted by user."
  exit 130  # Standard exit code for Ctrl-C
fi

if [[ ${#failed_builds[@]} -gt 0 ]]; then
  exit 1
fi

echo
echo "========================================================="
echo "Build complete! You can now use these libraries in your"
echo "firmware projects."
echo "========================================================="
echo
