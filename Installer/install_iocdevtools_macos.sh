#!/usr/bin/env bash
set -euo pipefail

SCRIPT_NAME="install_iocdevtools_macos"
SCRIPT_VERSION="v1.1.0"

ROOT="$HOME/IOcomposer"
TOOLS="/opt/xPacks"
BIN="/usr/local/bin"
ECLIPSE_APP="/Applications/IOcomposer.app"

echo "=============================================="
echo "   IOcomposer MCU Dev Tools Installer (macOS) "
echo "   Script: $SCRIPT_NAME"
echo "   Version: $SCRIPT_VERSION"
echo "=============================================="
echo

# ---------------------------------------------------------
# CLI
# ---------------------------------------------------------
show_help() {
  cat <<EOF
Usage: ./$SCRIPT_NAME.sh [OPTION]

Options:
  --with-llvm        Install optional LLVM compiler (failure does not stop setup)
  --help           Show help and exit
  --version        Show version and exit
  --home <path>    Set custom SDK installation root (default: ~/IOcomposer)
  --pyocd-only     Install/update pyOCD without reinstalling other tools
  --force-update   Force reinstall
  --iocomposer <path> Set custom IOcomposer installation path
  --uninstall      Remove toolchains + IOcomposer (keep repos/workspaces)
  (no option)      Install/update (skip if already installed)
EOF
}

WITH_LLVM=0
LLVM_STATUS="skipped (optional; use --with-llvm)"
MODE="install"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --with-llvm) WITH_LLVM=1 ;;
    --help) show_help; exit 0 ;;
    --version) echo "$SCRIPT_NAME $SCRIPT_VERSION"; exit 0 ;;
    --pyocd-only) MODE="pyocd" ;;
    --force-update) MODE="force"; echo ">>> Force update mode enabled" ;;
    --uninstall) MODE="uninstall"; echo ">>> Uninstall mode enabled" ;;
		--iocomposer|--eclipse)
			shift
			[[ -n "${1:-}" ]] || { echo "Missing IOcomposer path" >&2; exit 1; }
			ECLIPSE_APP="$1"
			;;
    --home)
      shift
      if [[ -z "${1:-}" ]]; then
        echo "❌ Missing path after --home"; exit 1
      fi
      ROOT="$1"
      echo ">>> Using custom home folder: $ROOT"
      ;;
    *) echo "Unknown option: $1"; exit 1 ;;
  esac
  shift
done

EXT="$ROOT/external"
PYOCD_DIR="$ROOT/Tools/Python"
SRECORD_DIR="$ROOT/Tools/SRecord"
LLVM_DIR="$ROOT/Tools/LLVM"
PYOCD="$PYOCD_DIR/bin/pyocd"
PYTHON3="$PYOCD_DIR/bin/python3"

# ---------------------------------------------------------
# UNINSTALL
# ---------------------------------------------------------
if [[ "$MODE" == "uninstall" ]]; then
  echo "⚠️  Removing toolchains + IOcomposer (repos and workspace kept)..."
  read -r -p "Proceed? (y/N) " c; [[ "$c" =~ ^[Yy]$ ]] || exit 0

  echo ">>> Checking xPacks in $TOOLS..."
  for dir in "$TOOLS"/*; do
    [ -d "$dir" ] || continue
    base=$(basename "$dir")
    echo "   Checking $base..."
    case "$base" in
      xpack-arm-none-eabi-*|xpack-riscv-none-elf-*|xpack-openocd-*)
        echo "   - Removing $dir"
        sudo rm -rf "$dir"
        ;;
      *) echo "   - Skipping $dir";;
    esac
  done

  echo ">>> Removing symlinks..."
  sudo rm -f "$BIN/arm-none-eabi-gcc" "$BIN/riscv-none-elf-gcc" "$BIN/openocd" || true

  echo ">>> Removing IOcomposer installation..."
  sudo rm -rf "$ECLIPSE_APP" || true


	if [[ -L "$BIN/pyocd" && "$(readlink "$BIN/pyocd")" == "$PYOCD" ]]; then
		sudo rm -f "$BIN/pyocd"
	fi
	if [[ -L "$LLVM_DIR" ]]; then
		rm -f "$LLVM_DIR"
	elif [[ -f "$LLVM_DIR/.iocomposer-managed" ]]; then
		rm -rf "$LLVM_DIR"
	fi
	rm -rf "$PYOCD_DIR" "$SRECORD_DIR"
  echo ">>> Removing IDAP tools..."
  rm -rf "$ROOT/Tools/IDAP-Link" || true

  # Prompt for IOsonata and external SDK removal
  echo
  read -r -p "Also remove IOsonata and external SDK folders? (y/N) " ans2
  if [[ "$ans2" =~ ^[Yy]$ ]]; then
    if [[ -d "$ROOT/IOsonata" ]]; then
      rm -rf "$ROOT/IOsonata"
      echo "   ✅ IOsonata removed."
    fi
    if [[ -d "$ROOT/external" ]]; then
      rm -rf "$ROOT/external"
      echo "   ✅ External SDK removed."
    fi
    if [[ -d "$ROOT/.iocomposer" ]]; then
      rm -rf "$ROOT/.iocomposer"
      echo "   ✅ .iocomposer removed."
    fi
  else
    echo ">>> Repositories under $ROOT and workspace dirs were kept."
  fi


  # ---------------------------------------------------------
  # Remove $ROOT if it is truly empty (no files/dirs).
  # Uses Bash globbing (more reliable than 'find' across environments)
  # and prints the first entry if not empty for easy debugging.
  # ---------------------------------------------------------
  if [[ -d "$ROOT" ]]; then
    # Include dotfiles in the check; treat missing matches as empty.
    shopt -s nullglob dotglob
    entries=( "$ROOT"/* )
    shopt -u nullglob dotglob

    if (( ${#entries[@]} == 0 )); then
      # Avoid "directory busy" if caller happens to be inside $ROOT.
      if ( cd "$HOME" && rmdir "$ROOT" 2>/dev/null ); then
        echo "   ✅ Removed empty root folder: $ROOT"
      else
        echo "   ⚠️  Root folder is empty but could not be removed: $ROOT"
      fi
    else
      echo "   ℹ️  Keeping $ROOT (not empty). First entry: ${entries[0]}"
    fi
  fi

  echo ">>> Uninstall complete!"
  exit 0
fi

mkdir -p "$ROOT"
sudo mkdir -p "$BIN"
if [[ "$MODE" != "pyocd" ]]; then
  mkdir -p "$EXT"
  sudo mkdir -p "$TOOLS"
fi

# ---------------------------------------------------------
# Arch detect
# ---------------------------------------------------------
ARCH=$(uname -m)
case "$ARCH" in
  arm64) PLATFORM="darwin-arm64"; ECLIPSE_ARCH="aarch64" ;;
  x86_64|amd64) PLATFORM="darwin-x64"; ECLIPSE_ARCH="x86_64" ;;
  *) echo "Unsupported architecture: $ARCH"; exit 1 ;;
esac
echo ">>> Detected architecture: $ARCH -> Eclipse arch=$ECLIPSE_ARCH"

# ---------------------------------------------------------
# Helpers
# ---------------------------------------------------------
resolve_path() {
  local target=$1
  if command -v greadlink >/dev/null 2>&1; then greadlink -f "$target"
  elif command -v realpath >/dev/null 2>&1; then realpath "$target"
  else /usr/bin/stat -f "%Y" "$target"
  fi
}

java_hash() {
  "$PYTHON3" - "$1" <<'EOF'
import sys
s = sys.argv[1]; h = 0
for c in s: h = (31*h + ord(c)) & 0xFFFFFFFF
if h >= 2**31: h -= 2**32
print(h)
EOF
}

# Helper function to validate a download URL
# Checks if the Content-Length is greater than 1MB (1,000,000 bytes)
is_valid_dmg_url() {
  local url="$1"
  # --- DEBUG ECHO REMOVED ---
  # echo "   Checking URL: $(basename "$url")"

  local headers
  headers=$(curl -fsSL -I --connect-timeout 10 --max-time 30 "$url") || return 1

  local length_line
  length_line=$(echo "$headers" | grep -i '^Content-Length:' | tail -n 1)

  if [[ -z "$length_line" ]]; then
    # --- DEBUG ECHO REMOVED ---
    # echo "   -> Failed: No Content-Length header found."
    return 1
  fi

  local length
  length=$(echo "$length_line" | awk '{print $2}' | tr -d '\r')

  if ! [[ "$length" =~ ^[0-9]+$ ]]; then
    # --- DEBUG ECHO REMOVED ---
    # echo "   -> Failed: Content-Length is not a number ($length)."
    return 1
  fi

  if (( length > 1000000 )); then
    # --- DEBUG ECHO REMOVED ---
    # echo "   -> Success: Found file, size ${length} bytes."
    return 0 # Success
  else
    # --- DEBUG ECHO REMOVED ---
    # echo "   -> Failed: File size ($length bytes) is too small. Likely a 404 page."
    return 1 # Failure
  fi
}

# ---------------------------------------------------------
# Cleanup (DMG mount/temp)
# ---------------------------------------------------------
TMP_ECLIPSE_DMG=""
ECLIPSE_MNT=""

cleanup() {
  if [[ -n "${ECLIPSE_MNT:-}" ]] && mount | grep -q " ${ECLIPSE_MNT} "; then
    hdiutil detach "$ECLIPSE_MNT" -quiet || true
  fi
  if [[ -n "${TMP_ECLIPSE_DMG:-}" ]]; then
    rm -f "$TMP_ECLIPSE_DMG" || true
  fi
  if [[ -n "${ECLIPSE_MNT:-}" ]]; then
    rm -rf "$ECLIPSE_MNT" || true
  fi
}
trap cleanup EXIT

# IOcomposer plugins and branding are installed here, not by the website launcher.
# Eclipse Embedded CDT supplies the runtime; retain its launcher filenames.
iocomposer_plugin_url() {
	local plugin_id="$1"
	local listing="$2"
	"$PYTHON3" - "$plugin_id" "$listing" <<'PY'
import json
import re
import sys

plugin_id, listing = sys.argv[1:]
with open(listing) as stream:
	items = json.load(stream)
versions = []
for item in items:
	name = item.get("name", "")
	match = re.fullmatch(re.escape(plugin_id) + r"_([0-9]+(?:\.[0-9]+)*)\.jar", name)
	if match:
		versions.append((tuple(int(part) for part in match[1].split(".")), name))
if not versions:
	raise SystemExit("No published IOcomposer plugin found: " + plugin_id)
print("https://raw.githubusercontent.com/I-SYST/iocomposer.io/main/plugin/" + max(versions)[1])
PY
}

install_iocomposer() (
	# Subshell cleanup does not replace the main installer's cleanup trap.
	local runtime="$1"
	local stage
	stage=$(mktemp -d)
	trap 'rm -rf "$stage"' EXIT
	local plugin url
	curl -fsSL "https://api.github.com/repos/I-SYST/iocomposer.io/contents/plugin?ref=main" -o "$stage/plugins.json"
	for plugin in com.iocomposer.embedcdt.ai com.iocomposer.embedcdt.ui; do
		url=""
		if [[ "$plugin" == "com.iocomposer.embedcdt.ai" ]]; then
			url="${IOCOMPOSER_AI_PLUGIN_URL:-}"
		fi
		if [[ -z "$url" ]]; then
			url=$(iocomposer_plugin_url "$plugin" "$stage/plugins.json")
		fi
		curl -fsSL "$url" -o "$stage/$plugin.jar"
		# Reject HTML/error responses before replacing installed plugins.
		unzip -tq "$stage/$plugin.jar" >/dev/null
	done
	# Both required plugins have downloaded successfully before either is replaced.
	sudo mkdir -p "$runtime/dropins"
	for plugin in com.iocomposer.embedcdt.ai com.iocomposer.embedcdt.ui; do
		sudo cp "$stage/$plugin.jar" "$runtime/dropins/$plugin.jar"
		sudo chmod 644 "$runtime/dropins/$plugin.jar"
		sudo find "$runtime/dropins" -maxdepth 1 -type f -name "${plugin}_*.jar" -delete
	done

	# Keep SDK path preferences and one customization hook in the runtime ini.
	"$PYTHON3" - "$runtime" "$stage" <<'PY'
from pathlib import Path
import sys

runtime, stage = map(Path, sys.argv[1:])
ini = (runtime / "eclipse.ini").read_text().splitlines()
result = []
skip = False
for line in ini:
	if skip:
		skip = False
		continue
	if line in ("-name", "-pluginCustomization"):
		skip = True
		continue
	result.append(line)
if "-vmargs" not in result:
	result.append("-vmargs")
index = result.index("-vmargs")
custom = "../Eclipse/plugin_customization.ini" if runtime.parent.name == "Contents" else "plugin_customization.ini"
result[index:index] = ["-name", "IOcomposer", "-pluginCustomization", custom]
if runtime.parent.name == "Contents" and "-Dswt.autoScale=false" not in result:
	result.append("-Dswt.autoScale=false")
(stage / "eclipse.ini").write_text("\n".join(result) + "\n")
preferences = {
	"org.eclipse.ui/showIntro": "false",
	"org.eclipse.ui/defaultPerspectiveId": "com.iocomposer.embedcdt.ui.perspective",
}
for package in ("embedcpp", "embedcpp.ui", "cpp", "common"):
	preferences["org.eclipse.epp.package." + package + "/showNewsOnStartup"] = "false"
preferences["org.eclipse.epp.mpc.ui/showNewsOnStartup"] = "false"
custom_file = runtime / "plugin_customization.ini"
lines = custom_file.read_text().splitlines() if custom_file.exists() else []
lines = [line for line in lines if line.split("=", 1)[0] not in preferences]
lines += [key + "=" + value for key, value in preferences.items()]
(stage / "plugin_customization.ini").write_text("\n".join(lines) + "\n")
PY
	sudo cp "$stage/eclipse.ini" "$runtime/eclipse.ini"
	sudo cp "$stage/plugin_customization.ini" "$runtime/plugin_customization.ini"

	if curl -fsSL "https://raw.githubusercontent.com/I-SYST/iocomposer.io/main/plugin/splash.bmp" -o "$stage/splash.bmp"; then
		sudo cp "$stage/splash.bmp" "$runtime/splash.bmp"
		while IFS= read -r destination; do
			sudo cp "$stage/splash.bmp" "$destination"
		done < <(find "$runtime/plugins" -name splash.bmp -type f)
	else
		echo "Warning: IOcomposer splash download failed; keeping the existing splash." >&2
	fi
	echo "IOcomposer AI and UI plugins installed at $runtime"
)

build_iocomposer_index() {
	local index="$ROOT/IOsonata/Installer/build_external_index.py"
	if [[ -f "$index" ]]; then
		"$PYOCD_DIR/bin/python3" "$index" --sdk-root "$ROOT/external" || {
			echo "Warning: SDK index failed. Retry: python3 $index --sdk-root $ROOT/external" >&2
		}
	else
		echo "Warning: SDK index script not found: $index" >&2
	fi
}

# ---------------------------------------------------------
# Install IOsonata Eclipse Plugin
# ---------------------------------------------------------
install_iosonata_plugin() {
  echo ">>> Installing IOsonata Eclipse Plugin..."
  
  local plugin_dir="$ROOT/IOsonata/Installer/eclipse_plugin"
  local dropins_dir="$ECLIPSE_APP/Contents/Eclipse/dropins"
  
  # Check if plugin directory exists
  if [[ ! -d "$plugin_dir" ]]; then
    echo "⚠️ Plugin directory not found at $plugin_dir"
    echo "   Skipping plugin installation."
    return
  fi
  
  # Find the latest plugin jar file
  local latest_plugin
  latest_plugin=$(ls -1 "$plugin_dir"/org.iosonata.embedcdt.templates.wizard_*.jar 2>/dev/null | sort -V | tail -n1)
  
  if [[ -z "$latest_plugin" ]]; then
    echo "⚠️ No IOsonata plugin jar file found in $plugin_dir"
    echo "   Skipping plugin installation."
    return
  fi
  
  echo "   → Found plugin: $(basename "$latest_plugin")"
  
  # Create dropins directory if it doesn't exist
  sudo mkdir -p "$dropins_dir"
  
  # Remove old versions of the plugin
  local old_plugins
  old_plugins=$(sudo find "$dropins_dir" -name "org.iosonata.embedcdt.templates.wizard_*.jar" 2>/dev/null || true)
  
  if [[ -n "$old_plugins" ]]; then
    echo "   → Removing old plugin versions..."
    echo "$old_plugins" | while read -r old_plugin; do
      if [[ -f "$old_plugin" ]]; then
        echo "     - Removing: $(basename "$old_plugin")"
        sudo rm -f "$old_plugin"
      fi
    done
  fi
  
  # Copy the latest plugin to dropins
  echo "   → Installing plugin to $dropins_dir"
  sudo cp "$latest_plugin" "$dropins_dir/"
  
  # Set proper permissions
  sudo chmod 644 "$dropins_dir/$(basename "$latest_plugin")"
  
  echo "✅ IOsonata Eclipse Plugin installed: $(basename "$latest_plugin")"
}


# ---------------------------------------------------------
# Install xPack toolchain
# ---------------------------------------------------------
install_xpack() {
  local repo=$1 tool=$2 name=$3

  echo ">>> Checking $name..." >&2
  local latest_json=$(curl -s https://api.github.com/repos/xpack-dev-tools/${repo}/releases/latest)
  local latest_tag=$(echo "$latest_json" | grep '"tag_name"' | cut -d '"' -f 4)
  local latest_norm=$(echo "$latest_tag" | sed 's/^v//')
  local latest_base=$(echo "$latest_norm" | cut -d- -f1)
  local latest_url=$(echo "$latest_json" | grep "browser_download_url" | grep "$PLATFORM" | cut -d '"' -f 4 | head -n1)

  local installed_ver=""
  if command -v "$tool" >/dev/null; then
  #  installed_ver=$($tool --version | head -n1 | grep -oE '[0-9]+\.[0-9]+\.[0-9]+')
    installed_ver=$($tool --version 2>&1 | head -n1 | grep -oE '[0-9]+\.[0-9]+\.[0-9]+')
  fi

  echo ">>> Installed: ${installed_ver:-none} | Latest: $latest_base" >&2

  if [[ "$MODE" != "force" && -n "$installed_ver" && "$installed_ver" == "$latest_base" ]]; then
    echo "✅ $name already up-to-date ($installed_ver)" >&2
    bin_path=$(command -v "$tool")
    real_bin=$(resolve_path "$bin_path")
    instdir=$(dirname "$(dirname "$real_bin")")
    echo "$instdir"
    return 0
  fi

  echo "⬇️ Installing $name $latest_norm..." >&2
  curl -L "$latest_url" | sudo tar -xJ -C "$TOOLS"

  origdir=$(ls -d "$TOOLS"/${repo}-* "$TOOLS"/xpack-${repo/-xpack/}-* 2>/dev/null | grep "$latest_norm" | head -n1 || true)
  if [[ -z "$origdir" ]]; then
    echo "❌ Could not find extracted folder for $name ($latest_norm)" >&2
    exit 1
  fi

  basever=$(echo "$latest_norm" | cut -d- -f1)
  targetdir="$TOOLS/$(echo $(basename "$origdir") | cut -d- -f1-4)-$basever"
  if [[ "$origdir" != "$targetdir" ]]; then
    sudo rm -rf "$targetdir" || true
    sudo mv "$origdir" "$targetdir"
  fi

  # Create only generic alias
  sudo ln -sf "$targetdir/bin/$tool" "$BIN/$tool"

  echo "✅ $name installed at $targetdir" >&2
  echo "$targetdir"
}

# ---------------------------------------------------------
# Install pyOCD in an isolated, user-owned Python environment
# ---------------------------------------------------------
# One LLVM installation provides both Arm and RISC-V code generation.
verify_llvm() (
	local directory="$1"
	local stage tool
	stage=$(mktemp -d)
	trap 'rm -rf "$stage"' EXIT
	for tool in clang clang++ ld.lld llvm-ar llvm-objcopy llvm-size; do
		[[ -x "$directory/bin/$tool" ]] || {
			echo "Missing LLVM tool: $directory/bin/$tool" >&2
			return 1
		}
	done
	printf 'int probe(void) { return 7; }\n' > "$stage/probe.c"
	printf 'extern "C" int probe(void) { return 7; }\n' > "$stage/probe.cpp"
	local target language
	for target in arm-none-eabi riscv32-unknown-elf; do
		local flags=()
		if [[ "$target" == "arm-none-eabi" ]]; then
			flags=(-mcpu=cortex-m4 -mthumb -mfloat-abi=soft)
		else
			flags=(-march=rv32imac -mabi=ilp32)
		fi
		for language in c cpp; do
			local compiler=clang
			[[ "$language" != "cpp" ]] || compiler=clang++
			"$directory/bin/$compiler" --target="$target" "${flags[@]}" \
				-ffreestanding -fno-exceptions -fno-rtti -nostdlib --ld-path="$directory/bin/ld.lld" \
				-Wl,-e,probe "$stage/probe.$language" -o "$stage/$target-$language.elf" || return 1
			"$directory/bin/llvm-objcopy" -O ihex "$stage/$target-$language.elf" "$stage/$target-$language.hex" || return 1
		done
	done
	"$directory/bin/clang" --version
)

install_llvm() {
	if [[ -x "$LLVM_DIR/bin/clang" ]]; then
		verify_llvm "$LLVM_DIR"
		return $?
	fi
	local prefix="" lld=""
	if [[ "$(uname -s)" == "Darwin" ]]; then
		local brew_cmd
		brew_cmd=$(command -v brew || true)
		if [[ -z "$brew_cmd" ]]; then
			for candidate in /opt/homebrew/bin/brew /usr/local/bin/brew; do
				[[ ! -x "$candidate" ]] || { brew_cmd="$candidate"; break; }
			done
		fi
		if [[ -z "$brew_cmd" ]]; then
			echo "Install Homebrew for LLVM, or provide a complete LLVM installation at $LLVM_DIR." >&2
			return 1
		fi
		"$brew_cmd" install llvm lld || return 1
		prefix=$("$brew_cmd" --prefix llvm) || return 1
		lld="$("$brew_cmd" --prefix lld)/bin/ld.lld"
	else
		if command -v apt-get >/dev/null; then
			sudo apt-get update
			sudo apt-get install -y clang llvm lld || return 1
		elif command -v dnf >/dev/null; then
			sudo dnf install -y clang llvm llvm-devel lld || return 1
		elif command -v yum >/dev/null; then
			sudo yum install -y clang llvm llvm-devel lld || return 1
		elif command -v pacman >/dev/null; then
			sudo pacman -S --needed --noconfirm clang llvm lld || return 1
		elif command -v zypper >/dev/null; then
			sudo zypper install -y clang llvm llvm-devel lld || return 1
		else
			echo "Provide a complete LLVM installation at $LLVM_DIR and retry." >&2
			return 1
		fi
		prefix=$(llvm-config --prefix) || return 1
	fi
	# Keep package-managed libraries in place and expose one complete tool directory.
	if [[ -e "$LLVM_DIR" || -L "$LLVM_DIR" ]]; then
		echo "LLVM destination already exists; inspect it before retrying: $LLVM_DIR" >&2
		return 1
	fi
	mkdir -p "$(dirname "$LLVM_DIR")"
	local stage entry
	stage=$(mktemp -d "$(dirname "$LLVM_DIR")/llvm-stage.XXXXXXXX") || return 1
	mkdir -p "$stage/bin"
	for entry in lib lib64 libexec include share; do
		[[ ! -e "$prefix/$entry" ]] || ln -s "$prefix/$entry" "$stage/$entry"
	done
	for entry in clang clang++ ld.lld llvm-ar llvm-objcopy llvm-size; do
		ln -s "$prefix/bin/$entry" "$stage/bin/$entry"
	done
	if [[ -n "$lld" ]]; then
		ln -sf "$lld" "$stage/bin/ld.lld"
	fi
	if ! verify_llvm "$stage"; then
		rm -rf "$stage"
		return 1
	fi
	touch "$stage/.iocomposer-managed"
	mv "$stage" "$LLVM_DIR"

}

install_optional_llvm() {
	if [[ "$WITH_LLVM" != 1 ]]; then
		echo "LLVM: $LLVM_STATUS"
		return 0
	fi
	if (install_llvm); then
		LLVM_STATUS="$LLVM_DIR (verified Arm and RISC-V)"
	else
		LLVM_STATUS="unavailable (optional installation failed)"
		echo "Warning: LLVM installation failed; continuing with GCC and remaining tools." >&2
	fi
}

install_macos_python() {
	local stage pkg python
	stage=$(mktemp -d)
	pkg="$stage/python.pkg"
	python="/Library/Frameworks/Python.framework/Versions/3.12/bin/python3"
	echo "Installing Python 3.12.4 from the official python.org macOS package..." >&2
	if ! curl -fL "https://www.python.org/ftp/python/3.12.4/python-3.12.4-macos11.pkg" -o "$pkg"; then
		rm -rf "$stage"
		echo "Failed to download the Python 3.12.4 macOS installer." >&2
		return 1
	fi
	if ! pkgutil --check-signature "$pkg" >/dev/null 2>&1; then
		rm -rf "$stage"
		echo "Python package signature verification failed." >&2
		return 1
	fi
	if ! sudo installer -pkg "$pkg" -target / >&2; then
		rm -rf "$stage"
		echo "Python 3.12.4 installation failed." >&2
		return 1
	fi
	rm -rf "$stage"
	if [[ ! -x "$python" ]] || ! "$python" -c 'import sys; sys.exit(sys.version_info < (3, 9))'; then
		echo "Python 3.12.4 was not found after package installation." >&2
		return 1
	fi
	echo "$python"
}

# Firmware tools share the SDK Tools directory. Python packages stay isolated.
install_python_tools() {
	local managed_python="${PYTHON3:-$PYOCD_DIR/bin/python3}"
	local bootstrap_python=""

	if [[ -x "$managed_python" ]]; then
		"$managed_python" -c 'import sys; sys.exit(sys.version_info < (3, 9))' || {
			echo "Managed Python 3.9 or newer is required for pyOCD: $managed_python" >&2
			return 1
		}
	else
		bootstrap_python=$(command -v python3 || true)
		if [[ -n "$bootstrap_python" ]] && ! "$bootstrap_python" -c 'import sys; sys.exit(sys.version_info < (3, 9))'; then
			bootstrap_python=""
		fi
		if [[ -z "$bootstrap_python" ]]; then
			bootstrap_python=$(install_macos_python) || return 1
		fi
		if ! "$bootstrap_python" -c 'import venv, ensurepip' 2>/dev/null; then
			if command -v apt-get >/dev/null; then
				sudo apt-get update
				sudo apt-get install -y python3-venv
			else
				echo "Install the venv/ensurepip components for python3, then retry." >&2
				return 1
			fi
		fi
		"$bootstrap_python" -m venv "$PYOCD_DIR" || return 1
	fi
	"$managed_python" -m pip install --upgrade pyocd || return 1
	"$PYOCD" --version || return 1
	if [[ -L "$BIN/pyocd" && "$(readlink "$BIN/pyocd")" == "${ROOT:-}/tools/pyocd/bin/pyocd" ]]; then
		sudo ln -sf "$PYOCD" "$BIN/pyocd"
	elif [[ ! -e "$BIN/pyocd" && ! -L "$BIN/pyocd" ]]; then
		sudo ln -s "$PYOCD" "$BIN/pyocd"
	fi
}

install_srecord() {
	if ! command -v srec_cat >/dev/null; then
		if [[ "$(uname -s)" == "Darwin" ]]; then
			local brew_cmd="" os_major="" os_tag="" bottle_tag="" brew_json=""
			if command -v port >/dev/null; then
				sudo port install srecord || return 1
			else
				brew_cmd=$(command -v brew || true)
				[[ -n "$brew_cmd" ]] || {
					for candidate in /opt/homebrew/bin/brew /usr/local/bin/brew; do
						[[ ! -x "$candidate" ]] || { brew_cmd="$candidate"; break; }
					done
				}
				if [[ -z "$brew_cmd" ]]; then
					echo "SRecord requires MacPorts, a compatible Homebrew bottle, or srec_cat already on PATH." >&2
					return 1
				fi

				os_major=$(sw_vers -productVersion | cut -d. -f1)
				case "$os_major" in
					11) os_tag="big_sur" ;;
					12) os_tag="monterey" ;;
					13) os_tag="ventura" ;;
					14) os_tag="sonoma" ;;
					15) os_tag="sequoia" ;;
					26) os_tag="tahoe" ;;
					27) os_tag="golden_gate" ;;
				esac
				if [[ -z "$os_tag" ]]; then
					echo "No Homebrew SRecord bottle check is defined for macOS $(sw_vers -productVersion)." >&2
					echo "Install SRecord with MacPorts or provide srec_cat on PATH, then rerun." >&2
					return 1
				fi
				bottle_tag="$os_tag"
				[[ "$(uname -m)" != "arm64" ]] || bottle_tag="arm64_$os_tag"

				if ! brew_json=$("$brew_cmd" info --json=v2 srecord); then
					echo "Could not query Homebrew SRecord package information." >&2
					return 1
				fi
				if ! printf '%s' "$brew_json" | "$PYTHON3" -c '
import json
import sys
data = json.load(sys.stdin)
formulae = data.get("formulae")
formula = formulae[0] if formulae else data
files = (((formula.get("bottle") or {}).get("stable") or {}).get("files") or {})
sys.exit(0 if sys.argv[1] in files else 1)
' "$bottle_tag"; then
					echo "Homebrew has no SRecord bottle for $bottle_tag; source builds are not used by this installer." >&2
					echo "Install SRecord with MacPorts or provide srec_cat on PATH, then rerun." >&2
					return 1
				fi

				"$brew_cmd" install srecord || return 1
				export PATH="$("$brew_cmd" --prefix)/bin:$PATH"
			fi
		elif command -v apt-get >/dev/null; then
			sudo apt-get update
			sudo apt-get install -y srecord || return 1
		elif command -v dnf >/dev/null; then
			sudo dnf install -y srecord || return 1
		elif command -v yum >/dev/null; then
			sudo yum install -y srecord || return 1
		elif command -v pacman >/dev/null; then
			sudo pacman -S --needed --noconfirm srecord || return 1
		elif command -v zypper >/dev/null; then
			sudo zypper install -y srecord || return 1
		else
			echo "Install SRecord with your package manager and retry." >&2
			return 1
		fi
	fi
	mkdir -p "$SRECORD_DIR/bin"
	local tool executable destination
	for tool in srec_cat srec_cmp srec_info; do
		executable=$(command -v "$tool") || return 1
		destination="$SRECORD_DIR/bin/$tool"
		if [[ "$executable" != "$destination" ]]; then
			ln -sf "$executable" "$destination" || return 1
		fi
	done
	"$SRECORD_DIR/bin/srec_cat" -VERSion || return 1
}

install_python_tools
if [[ "$MODE" == "pyocd" ]]; then
  exit 0
fi

install_optional_llvm
install_srecord
export PATH="$SRECORD_DIR/bin:$PYOCD_DIR/bin:$ROOT/Tools/IDAP-Link:$PATH"

ARM_DIR=$(install_xpack "arm-none-eabi-gcc-xpack" "arm-none-eabi-gcc" "ARM GCC")
RISCV_DIR=$(install_xpack "riscv-none-elf-gcc-xpack" "riscv-none-elf-gcc" "RISC-V GCC")
OPENOCD_DIR=$(install_xpack "openocd-xpack" "openocd" "OpenOCD")

if [[ -n "$RISCV_DIR" && ! -f "$RISCV_DIR/bin/riscv-none-elf-gcc" && -f "$RISCV_DIR/bin/riscv64-unknown-elf-gcc" ]]; then
  sudo ln -sf "$RISCV_DIR/bin/riscv64-unknown-elf-gcc" "$RISCV_DIR/bin/riscv-none-elf-gcc"
  sudo ln -sf "$RISCV_DIR/bin/riscv-none-elf-gcc" "$BIN/riscv-none-elf-gcc"
fi

# ---------------------------------------------------------
# Install IDAP tools
# ---------------------------------------------------------
IDAP_DIR="$ROOT/Tools/IDAP-Link"
mkdir -p "$IDAP_DIR"

IDAP_PROG="$IDAP_DIR/IDAPnRFProg"
if [[ ! -x "$IDAP_PROG" || "$MODE" == "force" ]]; then
  echo
  echo ">>> Downloading IDAPnRFProg tool..."
  IDAP_URL="https://sourceforge.net/projects/idaplinkfirmware/files/OSX/IDAPnRFProg_OSX_2_1_240807.zip/download"
	IDAP_PARENT=$(dirname "$IDAP_DIR")
	IDAP_STAGE=$(mktemp -d "$IDAP_PARENT/.idap-stage.XXXXXXXX")
	IDAP_BACKUP="$IDAP_PARENT/.idap-backup.$"
	if ! curl -fL "$IDAP_URL" -o "$IDAP_STAGE/archive.zip"; then
		rm -rf "$IDAP_STAGE"
		echo "Failed to download IDAPnRFProg; existing installation preserved." >&2
		exit 1
	fi
	if ! unzip -o -j "$IDAP_STAGE/archive.zip" -d "$IDAP_STAGE/unpacked" \
		|| [[ ! -s "$IDAP_STAGE/unpacked/IDAPnRFProg" ]]; then
		rm -rf "$IDAP_STAGE"
		echo "Invalid IDAPnRFProg archive; existing installation preserved." >&2
		exit 1
	fi
	mkdir -p "$IDAP_STAGE/candidate"
	if [[ -d "$IDAP_DIR" ]]; then
		cp -R "$IDAP_DIR/." "$IDAP_STAGE/candidate/" || {
			rm -rf "$IDAP_STAGE"
			echo "Failed to stage the existing IDAP-Link installation." >&2
			exit 1
		}
	fi
	if ! cp -R "$IDAP_STAGE/unpacked/." "$IDAP_STAGE/candidate/" \
		|| ! chmod +x "$IDAP_STAGE/candidate/IDAPnRFProg"; then
		rm -rf "$IDAP_STAGE"
		echo "Failed to stage IDAPnRFProg; existing installation preserved." >&2
		exit 1
	fi
	rm -rf "$IDAP_BACKUP"
	if [[ -d "$IDAP_DIR" ]]; then
		mv "$IDAP_DIR" "$IDAP_BACKUP"
	fi
	if ! mv "$IDAP_STAGE/candidate" "$IDAP_DIR"; then
		rm -rf "$IDAP_DIR"
		[[ ! -d "$IDAP_BACKUP" ]] || mv "$IDAP_BACKUP" "$IDAP_DIR"
		rm -rf "$IDAP_STAGE"
		echo "Failed to install IDAPnRFProg; previous installation restored." >&2
		exit 1
	fi
	rm -rf "$IDAP_BACKUP" "$IDAP_STAGE"
	echo "IDAPnRFProg installed at: $IDAP_PROG"
else
  echo "✅ IDAPnRFProg already installed (skipping)"
fi

# ---------------------------------------------------------
# Install Eclipse Embedded CDT
# ---------------------------------------------------------
echo
echo "💻 Installing IOcomposer runtime (Eclipse Embedded CDT)..."
# Try the primary server before an alternate mirror. Keep TLS verification on.
if [[ "$MODE" == "force" || ! -x "$ECLIPSE_APP/Contents/MacOS/eclipse" ]]; then
ECLIPSE_MIRRORS=(
  "https://download.eclipse.org/technology/epp/downloads/release"
  "https://ftp.osuosl.org/pub/eclipse/technology/epp/downloads/release"
)

ECLIPSE_URL=""
for MIRROR in "${ECLIPSE_MIRRORS[@]}"; do
  echo ">>> Checking Eclipse releases: $MIRROR"
  if ! RELEASE_INDEX=$(curl -fsSL --connect-timeout 10 --max-time 30 "$MIRROR/"); then
    echo "Warning: Could not read Eclipse release listing; trying next server." >&2
    continue
  fi

  # Listings may use relative or absolute links and either quote style.
  RELEASES=$(printf '%s\n' "$RELEASE_INDEX" | grep -oE "href=['\"][^'\"]*20[0-9]{2}-[0-9]{2}/?['\"]" | grep -oE '20[0-9]{2}-[0-9]{2}' | sort -ru || true)
  if [[ -z "$RELEASES" ]]; then
    echo "Warning: No Eclipse releases found; trying next server." >&2
    continue
  fi

  for release in $RELEASES; do
    URL_EMBEDCDT="$MIRROR/$release/R/eclipse-embedcdt-$release-R-macosx-cocoa-$ECLIPSE_ARCH.dmg"
    URL_EMBEDCPP="$MIRROR/$release/R/eclipse-embedcpp-$release-R-macosx-cocoa-$ECLIPSE_ARCH.dmg"

    if is_valid_dmg_url "$URL_EMBEDCDT"; then
      ECLIPSE_URL="$URL_EMBEDCDT"
      break
    elif is_valid_dmg_url "$URL_EMBEDCPP"; then
      ECLIPSE_URL="$URL_EMBEDCPP"
      break
    fi
  done

  if [[ -n "$ECLIPSE_URL" ]]; then
    break
  fi
done

if [[ -z "$ECLIPSE_URL" ]]; then
  echo "Error: Could not find an Eclipse Embedded CDT download on any server." >&2
  echo "Check the curl errors above and your HTTPS connection." >&2
  exit 1
fi

echo "⬇️ Downloading Eclipse: $ECLIPSE_URL"

TMP_ECLIPSE_DMG=$(mktemp)
if ! curl -fL "$ECLIPSE_URL" -o "$TMP_ECLIPSE_DMG"; then
  echo "❌ Eclipse download failed." >&2
  exit 1
fi

ECLIPSE_MNT=$(mktemp -d /tmp/eclipse.XXXX)

if ! hdiutil attach "$TMP_ECLIPSE_DMG" -mountpoint "$ECLIPSE_MNT" -nobrowse -quiet; then
  echo "❌ Failed to mount Eclipse DMG." >&2
  exit 1
fi

ECLIPSE_STAGE="$ECLIPSE_APP.stage.$"
ECLIPSE_BACKUP="$ECLIPSE_APP.backup.$"
sudo rm -rf "$ECLIPSE_STAGE" "$ECLIPSE_BACKUP"
if ! sudo ditto "$ECLIPSE_MNT/Eclipse.app" "$ECLIPSE_STAGE" \
	|| [[ ! -x "$ECLIPSE_STAGE/Contents/MacOS/eclipse" ]]; then
	sudo rm -rf "$ECLIPSE_STAGE"
	echo "❌ Failed to stage the IOcomposer runtime; existing installation preserved." >&2
	exit 1
fi

if [[ -e "$ECLIPSE_APP" ]]; then
	sudo mv "$ECLIPSE_APP" "$ECLIPSE_BACKUP"
fi
if ! sudo mv "$ECLIPSE_STAGE" "$ECLIPSE_APP"; then
	sudo rm -rf "$ECLIPSE_APP"
	[[ ! -e "$ECLIPSE_BACKUP" ]] || sudo mv "$ECLIPSE_BACKUP" "$ECLIPSE_APP"
	echo "❌ Failed to replace the IOcomposer runtime; previous installation restored." >&2
	exit 1
fi
sudo rm -rf "$ECLIPSE_BACKUP" || true

hdiutil detach "$ECLIPSE_MNT" -quiet || true
rm -f "$TMP_ECLIPSE_DMG" || true
rm -rf "$ECLIPSE_MNT" || true

TMP_ECLIPSE_DMG=""
ECLIPSE_MNT=""

echo "✅ IOcomposer runtime installed at $ECLIPSE_APP"

else
	echo "IOcomposer runtime already installed at $ECLIPSE_APP"
fi

# ---------------------------------------------------------
# Seed Eclipse prefs
# ---------------------------------------------------------

ECLIPSE_SETTINGS="$ECLIPSE_APP/Contents/Eclipse/configuration/.settings"
sudo mkdir -p "$ECLIPSE_SETTINGS"

ARM_HASH=$(java_hash "xPack GNU Arm Embedded GCC")
if (( ARM_HASH < 0 )); then ARM_HASH=$((ARM_HASH + 4294967296)); fi

RISCV_HASH=$(java_hash "xPack GNU RISC-V Embedded GCC")
if (( RISCV_HASH < 0 )); then RISCV_HASH=$((RISCV_HASH + 4294967296)); fi
RISCV_HASH=$((RISCV_HASH + 1))

sudo sh -c "cat > '$ECLIPSE_SETTINGS/org.eclipse.embedcdt.core.prefs' <<EOF
eclipse.preferences.version=1
xpack.arm.toolchain.path=$ARM_DIR/bin
xpack.riscv.toolchain.path=$RISCV_DIR/bin
xpack.openocd.path=$OPENOCD_DIR/bin
xpack.strict=true
EOF"

sudo sh -c "cat > '$ECLIPSE_SETTINGS/org.eclipse.embedcdt.managedbuild.cross.arm.core.prefs' <<EOF
toolchain.path.$ARM_HASH=$ARM_DIR/bin
toolchain.path.1287942917=$ARM_DIR/bin
toolchain.path.strict=true
EOF"

sudo sh -c "cat > '$ECLIPSE_SETTINGS/org.eclipse.embedcdt.managedbuild.cross.riscv.core.prefs' <<EOF
toolchain.path.$RISCV_HASH=$RISCV_DIR/bin
toolchain.path.strict=true
EOF"

sudo sh -c "cat > '$ECLIPSE_SETTINGS/org.eclipse.embedcdt.debug.gdbjtag.openocd.core.prefs' <<EOF
install.folder=$OPENOCD_DIR/bin
install.folder.strict=true
EOF"

sudo sh -c "cat > '$ECLIPSE_SETTINGS/org.eclipse.core.runtime.prefs' <<EOF
eclipse.preferences.version=1
environment/project/LLVM_HOME/value=$LLVM_DIR
environment/project/ARM_LLVM_HOME/value=$LLVM_DIR
environment/project/RISCV_LLVM_HOME/value=$LLVM_DIR
environment/project/IOCOMPOSER_HOME/value=$ROOT
environment/project/PATH/value=$SRECORD_DIR/bin:$PYOCD_DIR/bin:$ROOT/Tools/IDAP-Link
environment/project/PATH/operation=prepend
environment/project/PATH/delimiter=:
environment/project/ARM_GCC_HOME/value=$ARM_DIR/bin
environment/project/RISCV_GCC_HOME/value=$RISCV_DIR/bin
environment/project/OPENOCD_HOME/value=$OPENOCD_DIR/bin
environment/project/NRFX_HOME/value=$EXT/nrfx
environment/project/NRFXLIB_HOME/value=$EXT/sdk-nrfxlib
environment/project/NRF5_SDK_HOME/value=$EXT/nRF5_SDK
environment/project/NRF5_SDK_MESH_HOME/value=$EXT/nRF5_SDK_Mesh
environment/project/BSEC_HOME/value=$EXT/BSEC
EOF"

echo "✅ Eclipse preferences seeded (ARM, RISC-V, OpenOCD, macros)."

# ---------------------------------------------------------
# Configure iosonata_loc and iocomposer_home (Java & Eclipse Core)
# ---------------------------------------------------------
echo
echo ">>> Configuring IOsonata and IOcomposer properties in eclipse.ini..."

ECLIPSE_INI="$ECLIPSE_APP/Contents/Eclipse/eclipse.ini"
CUSTOM_INI="$ECLIPSE_APP/Contents/Eclipse/plugin_customization.ini"

# 1. Create the plugin_customization.ini file for Eclipse Core (Linked Folders)
echo "⚙️  Generating plugin_customization.ini for Workspace Path Variables..."
sudo sh -c "cat > '$CUSTOM_INI' <<EOF
org.eclipse.core.resources/pathvariable.IOSONATA_LOC=$ROOT
org.eclipse.core.resources/pathvariable.IOCOMPOSER_HOME=$ROOT
EOF"

# 2. Clean up old properties from eclipse.ini if they exist
sudo sed -i.bak '/^-Diosonata\.home=/d' "$ECLIPSE_INI"
sudo sed -i '' '/^-Diosonata_loc=/d' "$ECLIPSE_INI"
sudo sed -i '' '/^-Diocomposer_home=/d' "$ECLIPSE_INI"
sudo sed -i '' '/^-pluginCustomization/d' "$ECLIPSE_INI"
sudo sed -i '' '\|plugin_customization\.ini|d' "$ECLIPSE_INI"

# 3. Inject configurations into eclipse.ini
if grep -q "^-vmargs" "$ECLIPSE_INI"; then
    # Safely insert -pluginCustomization BEFORE -vmargs using awk
    sudo awk '
    /^-vmargs/ {
        print "-pluginCustomization"
        print "../Eclipse/plugin_customization.ini"
    }
    { print }
    ' "$ECLIPSE_INI" > "/tmp/eclipse_ini.tmp"
    sudo mv "/tmp/eclipse_ini.tmp" "$ECLIPSE_INI"

    # Insert Java System Properties AFTER -vmargs using sed
    sudo sed -i '' '/^-vmargs$/a\
-Diosonata_loc='"$ROOT"'
' "$ECLIPSE_INI"
    sudo sed -i '' '/^-Diosonata_loc=/a\
-Diocomposer_home='"$ROOT"'
' "$ECLIPSE_INI"
else
    # No -vmargs section found, append everything to the bottom
    echo "-pluginCustomization" | sudo tee -a "$ECLIPSE_INI" > /dev/null
    echo "../Eclipse/plugin_customization.ini" | sudo tee -a "$ECLIPSE_INI" > /dev/null
    echo "-vmargs" | sudo tee -a "$ECLIPSE_INI" > /dev/null
    echo "-Diosonata_loc=$ROOT" | sudo tee -a "$ECLIPSE_INI" > /dev/null
    echo "-Diocomposer_home=$ROOT" | sudo tee -a "$ECLIPSE_INI" > /dev/null
fi

echo "✅ System properties configured in eclipse.ini:"
echo "   iosonata_loc=$ROOT"
echo "   iocomposer_home=$ROOT"
echo "✅ Eclipse Path Variables configured in plugin_customization.ini:"
echo "   IOSONATA_LOC=$ROOT"
echo

# Step 1: Ensure Eclipse has initialized its instance folder
if [[ -d "$ECLIPSE_APP" ]]; then
  echo "⏳ Initializing Eclipse to create instance configuration..."
  "$ECLIPSE_APP/Contents/MacOS/eclipse" -nosplash -initialize || true
fi

# Step 2: Find the newest instance configuration folder under ~/.eclipse
INSTANCE_CFG=""
if [[ -x "$PYTHON3" ]]; then
  INSTANCE_CFG=$("$PYTHON3" - <<'PY'
import os, sys
base = os.path.expanduser("~/.eclipse")
cands = []
if os.path.isdir(base):
  for root, dirs, _ in os.walk(base):
    for d in dirs:
      if d == "configuration":
        p = os.path.join(root, d)
        try:
          cands.append((os.path.getmtime(p), p))
        except OSError:
          pass
if not cands:
  sys.exit(1)
cands.sort(reverse=True)
print(cands[0][1])
PY
) || INSTANCE_CFG=""
fi

if [[ -z "$INSTANCE_CFG" ]]; then
  echo "⚠️ Could not find Eclipse instance configuration folder under ~/.eclipse."
  echo "   Eclipse may not have been started yet."
else
  echo "📂 Found Eclipse settings: $INSTANCE_CFG"
  mkdir -p "$INSTANCE_CFG/.settings"

  cat > "$INSTANCE_CFG/.settings/org.eclipse.embedcdt.core.prefs" <<EOF
eclipse.preferences.version=1
xpack.arm.toolchain.path=$ARM_DIR/bin
xpack.riscv.toolchain.path=$RISCV_DIR/bin
xpack.openocd.path=$OPENOCD_DIR/bin
xpack.strict=true
EOF

  cat > "$INSTANCE_CFG/.settings/org.eclipse.embedcdt.managedbuild.cross.arm.core.prefs" <<EOF
eclipse.preferences.version=1
toolchain.path.$ARM_HASH=$ARM_DIR/bin
toolchain.path.1287942917=$ARM_DIR/bin
toolchain.path.strict=true
EOF

  cat > "$INSTANCE_CFG/.settings/org.eclipse.embedcdt.managedbuild.cross.riscv.core.prefs" <<EOF
eclipse.preferences.version=1
toolchain.path.$RISCV_HASH=$RISCV_DIR/bin
toolchain.path.strict=true
EOF

  cat > "$INSTANCE_CFG/.settings/org.eclipse.embedcdt.debug.gdbjtag.openocd.core.prefs" <<EOF
eclipse.preferences.version=1
install.folder=$OPENOCD_DIR/bin
install.folder.strict=true
EOF

  cat > "$INSTANCE_CFG/.settings/org.eclipse.core.runtime.prefs" <<EOF
eclipse.preferences.version=1
environment/project/LLVM_HOME/value=$LLVM_DIR
environment/project/ARM_LLVM_HOME/value=$LLVM_DIR
environment/project/RISCV_LLVM_HOME/value=$LLVM_DIR
environment/project/IOCOMPOSER_HOME/value=$ROOT
environment/project/PATH/value=$SRECORD_DIR/bin:$PYOCD_DIR/bin:$ROOT/Tools/IDAP-Link
environment/project/PATH/operation=prepend
environment/project/PATH/delimiter=:
environment/project/ARM_GCC_HOME/value=$ARM_DIR/bin
environment/project/RISCV_GCC_HOME/value=$RISCV_DIR/bin
environment/project/OPENOCD_HOME/value=$OPENOCD_DIR/bin
environment/project/NRFX_HOME/value=$EXT/nrfx
environment/project/NRFXLIB_HOME/value=$EXT/sdk-nrfxlib
environment/project/NRF5_SDK_HOME/value=$EXT/nRF5_SDK
environment/project/NRF5_SDK_MESH_HOME/value=$EXT/nRF5_SDK_Mesh
environment/project/BSEC_HOME/value=$EXT/BSEC
EOF
fi

# ---------------------------------------------------------
# Sync IOsonata + external SDKs (single source of truth)
# ---------------------------------------------------------
echo
echo "📦 Syncing IOsonata + external SDKs..."

CLONE_URL="https://raw.githubusercontent.com/IOsonata/IOsonata/${IOSONATA_INSTALLER_REF:-main}/Installer/clone_iosonata_sdk_macos.sh"
TMP_CLONE=$(mktemp /tmp/clone_iosonata_sdk_macos.XXXXXX.sh)
CLONE_LOG=$(mktemp /tmp/clone_iosonata_sdk_macos.XXXXXX.log)

if ! curl -fsSL "$CLONE_URL" -o "$TMP_CLONE"; then
  echo "❌ Failed to download clone script:"
  echo "   $CLONE_URL"
  exit 1
fi

chmod +x "$TMP_CLONE"

# Prefer --no-build if supported by the clone script (newer versions). If not supported,
# retry without it. We capture output so we can detect whether the builder ran.
CLONE_ARGS=(--home "$ROOT" --no-build)
if [[ "$MODE" == "force" ]]; then
  CLONE_ARGS+=(--mode force)
fi

run_clone() {
  # shellcheck disable=SC2068
  zsh "$TMP_CLONE" "$@" 2>&1 | tee -a "$CLONE_LOG"
}

if ! run_clone "${CLONE_ARGS[@]}"; then
	echo "SDK synchronization failed. See $CLONE_LOG" >&2
	rm -f "$TMP_CLONE"
	exit 1
fi

rm -f "$TMP_CLONE" || true

# Detect whether the clone script already invoked the IOsonata library builder.
CLONE_INVOKED_BUILD=0
if grep -qE "IOsonata Library (Auto-)?Build|IOsonata Library Builder|Available IOsonata library projects|Select project to build" "$CLONE_LOG"; then
  CLONE_INVOKED_BUILD=1
fi

rm -f "$CLONE_LOG" || true

# ---------------------------------------------------------
# Install IOsonata Eclipse Plugin
# ---------------------------------------------------------
echo
install_iosonata_plugin

install_iocomposer "$ECLIPSE_APP/Contents/Eclipse"
sudo /usr/libexec/PlistBuddy -c "Set :CFBundleName IOcomposer" "$ECLIPSE_APP/Contents/Info.plist"
sudo /usr/libexec/PlistBuddy -c "Set :CFBundleDisplayName IOcomposer" "$ECLIPSE_APP/Contents/Info.plist" 2>/dev/null || \
	sudo /usr/libexec/PlistBuddy -c "Add :CFBundleDisplayName string IOcomposer" "$ECLIPSE_APP/Contents/Info.plist"

# ---------------------------------------------------------
# Generate makefile_path.mk
# ---------------------------------------------------------
echo
echo "📝 Generating makefile_path.mk for Makefile-based builds..."

MAKEFILE_PATH_MK="$ROOT/IOsonata/makefile_path.mk"

cat > "$MAKEFILE_PATH_MK" <<EOF
# makefile_path.mk
# Auto-generated by install_iocdevtools_macos.sh $SCRIPT_VERSION
# This file contains all path macros required to compile IOsonata projects using Makefiles
# Include this file in your project Makefile: include \$(IOSONATA_ROOT)/makefile_path.mk

# ============================================
# Toolchain Paths
# ============================================
ARM_GCC_ROOT = $ARM_DIR
ARM_GCC_BIN = $ARM_DIR/bin
ARM_GCC = \$(ARM_GCC_BIN)/arm-none-eabi-gcc
ARM_GPP = \$(ARM_GCC_BIN)/arm-none-eabi-g++
ARM_AS = \$(ARM_GCC_BIN)/arm-none-eabi-as
ARM_LD = \$(ARM_GCC_BIN)/arm-none-eabi-ld
ARM_AR = \$(ARM_GCC_BIN)/arm-none-eabi-ar
ARM_OBJCOPY = \$(ARM_GCC_BIN)/arm-none-eabi-objcopy
ARM_OBJDUMP = \$(ARM_GCC_BIN)/arm-none-eabi-objdump
ARM_SIZE = \$(ARM_GCC_BIN)/arm-none-eabi-size
ARM_GDB = \$(ARM_GCC_BIN)/arm-none-eabi-gdb

RISCV_GCC_ROOT = $RISCV_DIR
RISCV_GCC_BIN = $RISCV_DIR/bin
RISCV_GCC = \$(RISCV_GCC_BIN)/riscv-none-elf-gcc
RISCV_GPP = \$(RISCV_GCC_BIN)/riscv-none-elf-g++
RISCV_AS = \$(RISCV_GCC_BIN)/riscv-none-elf-as
RISCV_LD = \$(RISCV_GCC_BIN)/riscv-none-elf-ld
RISCV_AR = \$(RISCV_GCC_BIN)/riscv-none-elf-ar
RISCV_OBJCOPY = \$(RISCV_GCC_BIN)/riscv-none-elf-objcopy
RISCV_OBJDUMP = \$(RISCV_GCC_BIN)/riscv-none-elf-objdump
RISCV_SIZE = \$(RISCV_GCC_BIN)/riscv-none-elf-size
RISCV_GDB = \$(RISCV_GCC_BIN)/riscv-none-elf-gdb

OPENOCD_ROOT = $OPENOCD_DIR
OPENOCD = $OPENOCD_DIR/bin/openocd
LLVM_ROOT = $LLVM_DIR
ARM_LLVM_ROOT = $LLVM_DIR
RISCV_LLVM_ROOT = $LLVM_DIR
CLANG = $LLVM_DIR/bin/clang
CLANGXX = $LLVM_DIR/bin/clang++
LLVM_LD = $LLVM_DIR/bin/ld.lld
LLVM_AR = $LLVM_DIR/bin/llvm-ar
LLVM_OBJCOPY = $LLVM_DIR/bin/llvm-objcopy
ARM_LLVM_TARGET = --target=arm-none-eabi
RISCV_LLVM_TARGET = --target=riscv32-unknown-elf
SDK_TOOLS_ROOT = $ROOT/Tools
SRECORD_ROOT = $SRECORD_DIR
SREC_CAT = $SRECORD_DIR/bin/srec_cat
PYTHON = $PYOCD_DIR/bin/python3
PYOCD_ROOT = $PYOCD_DIR
PYOCD = $PYOCD

# ============================================
# IOsonata Paths
# ============================================
# IOCOMPOSER_HOME must be set to your IOcomposer root directory
ifndef IOCOMPOSER_HOME
\$(error IOCOMPOSER_HOME is not set. Please set it to your IOcomposer root directory)
endif

IOSONATA_ROOT = \$(IOCOMPOSER_HOME)/IOsonata
IOSONATA_INCLUDE = \$(IOSONATA_ROOT)/include
IOSONATA_SRC = \$(IOSONATA_ROOT)/src

# ============================================
# ARM-specific Paths
# ============================================
ARM_ROOT = \$(IOSONATA_ROOT)/ARM
ARM_CMSIS = \$(ARM_ROOT)/CMSIS
ARM_CMSIS_INCLUDE = \$(ARM_CMSIS)/Core/Include
ARM_INCLUDE = \$(ARM_ROOT)/include
ARM_SRC = \$(ARM_ROOT)/src
ARM_LDSCRIPT = \$(ARM_ROOT)/ldscript

# Vendor-specific paths
ARM_NORDIC = \$(ARM_ROOT)/Nordic
ARM_NXP = \$(ARM_ROOT)/NXP
ARM_ST = \$(ARM_ROOT)/ST
ARM_MICROCHIP = \$(ARM_ROOT)/Microchip
ARM_RENESAS = \$(ARM_ROOT)/Renesas

# ============================================
# RISC-V-specific Paths
# ============================================
RISCV_ROOT = \$(IOSONATA_ROOT)/RISCV
RISCV_INCLUDE = \$(RISCV_ROOT)/include
RISCV_SRC = \$(RISCV_ROOT)/src
RISCV_LDSCRIPT = \$(RISCV_ROOT)/ldscript

# Vendor-specific paths
RISCV_ESPRESSIF = \$(RISCV_ROOT)/Espressif
RISCV_NORDIC = \$(RISCV_ROOT)/Nordic
RISCV_RENESAS = \$(RISCV_ROOT)/Renesas

# ============================================
# External Libraries
# ============================================
EXTERNAL_ROOT = \$(IOCOMPOSER_HOME)/external
NRFX_ROOT = \$(EXTERNAL_ROOT)/nrfx
SDK_NRF_BM_ROOT = \$(EXTERNAL_ROOT)/sdk-nrf-bm
SDK_NRFXLIB_ROOT = \$(EXTERNAL_ROOT)/sdk-nrfxlib
NRF5_SDK_ROOT = \$(EXTERNAL_ROOT)/nRF5_SDK
NRF5_SDK_MESH_ROOT = \$(EXTERNAL_ROOT)/nRF5_SDK_Mesh
BSEC_ROOT = \$(EXTERNAL_ROOT)/BSEC
FUSION_ROOT = \$(EXTERNAL_ROOT)/Fusion
LVGL_ROOT = \$(EXTERNAL_ROOT)/lvgl
LWIP_ROOT = \$(EXTERNAL_ROOT)/lwip
FREERTOS_KERNEL_ROOT = \$(EXTERNAL_ROOT)/FreeRTOS-Kernel
TINYUSB_ROOT = \$(EXTERNAL_ROOT)/tinyusb

# ============================================
# Additional IOsonata Modules
# ============================================
FATFS_ROOT = \$(IOSONATA_ROOT)/fatfs
LITTLEFS_ROOT = \$(IOSONATA_ROOT)/littlefs
MICRO_ECC_ROOT = \$(IOSONATA_ROOT)/micro-ecc

# ============================================
# Common Include Paths (for -I flags)
# ============================================
IOSONATA_INCLUDES = -I\$(IOSONATA_INCLUDE) \\
                    -I\$(IOSONATA_INCLUDE)/bluetooth \\
                    -I\$(IOSONATA_INCLUDE)/audio \\
                    -I\$(IOSONATA_INCLUDE)/converters \\
                    -I\$(IOSONATA_INCLUDE)/coredev \\
                    -I\$(IOSONATA_INCLUDE)/display \\
                    -I\$(IOSONATA_INCLUDE)/imu \\
                    -I\$(IOSONATA_INCLUDE)/miscdev \\
                    -I\$(IOSONATA_INCLUDE)/pwrmgnt \\
                    -I\$(IOSONATA_INCLUDE)/sensors \\
                    -I\$(IOSONATA_INCLUDE)/storage \\
                    -I\$(IOSONATA_INCLUDE)/sys \\
                    -I\$(IOSONATA_INCLUDE)/usb

ARM_INCLUDES = -I\$(ARM_INCLUDE) \\
               -I\$(ARM_CMSIS_INCLUDE)

RISCV_INCLUDES = -I\$(RISCV_INCLUDE)

# ============================================
# Environment Variables (optional)
# ============================================
export ARM_GCC_HOME := $ARM_DIR/bin
export RISCV_GCC_HOME := $RISCV_DIR/bin
export OPENOCD_HOME := $OPENOCD_DIR/bin
export NRFX_HOME := \$(NRFX_ROOT)
export NRFXLIB_HOME := \$(SDK_NRFXLIB_ROOT)
export NRF5_SDK_HOME := \$(NRF5_SDK_ROOT)
export NRF5_SDK_MESH_HOME := \$(NRF5_SDK_MESH_ROOT)
export BSEC_HOME := \$(BSEC_ROOT)
EOF

echo "✅ makefile_path.mk created at: $MAKEFILE_PATH_MK"
echo "   Projects can include it with: include \$(IOSONATA_ROOT)/makefile_path.mk"

# ---------------------------------------------------------
# Sign the configured app before the headless builder starts.
sudo xattr -cr "$ECLIPSE_APP"
sudo codesign --force --deep --sign - "$ECLIPSE_APP"

# IOsonata Library Build (run exactly once)
# ---------------------------------------------------------
# If the clone script already invoked the builder, do NOT run it again.
# If it did NOT invoke the builder (e.g., clone supports --no-build), run it here.
if [[ "${CLONE_INVOKED_BUILD:-0}" -eq 0 ]]; then
  BUILD_SCRIPT="$ROOT/IOsonata/Installer/build_iosonata_lib_macos.sh"
  if [[ -f "$BUILD_SCRIPT" ]]; then
    echo ""
    echo "========================================================="
    echo "  IOsonata Library Build"
    echo "========================================================="
    echo ""
    bash "$BUILD_SCRIPT" --home "$ROOT" --iocomposer "$ECLIPSE_APP"
  else
    echo ""
    echo "ℹ️  IOsonata library builder not found:"
    echo "   $BUILD_SCRIPT"
    echo "   (Skipping library build)"
    echo ""
  fi
else
  echo ""
  echo "ℹ️  IOsonata libraries were already handled by the clone step (skipping rebuild)."
  echo ""
fi

# ---------------------------------------------------------
build_iocomposer_index
# Sign after every installer write to the application bundle, including builds.
sudo xattr -cr "$ECLIPSE_APP"
sudo codesign --force --deep --sign - "$ECLIPSE_APP"

# Summary

# ---------------------------------------------------------
echo
echo "=============================================="
echo " IOcomposer MCU Dev Tools Installation Summary"
echo "=============================================="

ECLIPSE_VER="Not installed"
if [[ -f "$ECLIPSE_APP/Contents/Info.plist" ]]; then
  ECLIPSE_VER=$(/usr/libexec/PlistBuddy -c "Print :CFBundleShortVersionString" "$ECLIPSE_APP/Contents/Info.plist" 2>/dev/null || echo "Unknown")
fi

if [[ -n "${ARM_DIR:-}" && -x "$ARM_DIR/bin/arm-none-eabi-gcc" ]]; then
  ARM_VER=$("$ARM_DIR/bin/arm-none-eabi-gcc" --version | head -n1 | grep -oE '[0-9]+\.[0-9]+\.[0-9]+' | head -n1)
else
  ARM_VER="Not found"
fi

if [[ -n "${RISCV_DIR:-}" && -x "$RISCV_DIR/bin/riscv-none-elf-gcc" ]]; then
  RISCV_VER=$("$RISCV_DIR/bin/riscv-none-elf-gcc" --version | head -n1 | grep -oE '[0-9]+\.[0-9]+\.[0-9]+' | head -n1)
else
  RISCV_VER="Not found"
fi

if [[ -n "${OPENOCD_DIR:-}" && -x "$OPENOCD_DIR/bin/openocd" ]]; then
#  OPENOCD_VER=$("$OPENOCD_DIR/bin/openocd" --version | head -n1 | grep -oE '[0-9]+\.[0-9]+\.[0-9]+' | head -n1)
  OPENOCD_VER=$("$OPENOCD_DIR/bin/openocd" --version 2>&1 | head -n1 | grep -oE '[0-9]+\.[0-9]+\.[0-9]+' | head -n1)
else
  OPENOCD_VER="Not found"
fi

printf "%-25s %s\n" "IOcomposer runtime:" "$ECLIPSE_VER"
printf "%-25s %s\n" "ARM GCC:" "$ARM_VER"
printf "%-25s %s\n" "RISC-V GCC:" "$RISCV_VER"
printf "%-25s %s\n" "OpenOCD:" "$OPENOCD_VER"
printf "%-25s %s\n" "pyOCD:" "$("$PYOCD" --version)"
printf "%-25s %s\n" "pyOCD executable:" "$PYOCD"
echo ""
printf "%-25s %s\n" "iocomposer_home:" "$ROOT"
printf "%-25s %s\n" "iosonata_loc:" "$ROOT"

echo "=============================================="
echo " Installation complete!"
echo "=============================================="
echo
echo "✅ iosonata_loc is configured in Eclipse installation"
echo
echo "Usage in .cproject files:"
echo "  \${system_property:iosonata_loc}/IOsonata/include"
echo "  \${system_property:iosonata_loc}/IOsonata/ARM/include"
echo "  \${system_property:iosonata_loc}/IOsonata/ARM/CMSIS/Core/Include"
echo
echo "The variable is available in ALL workspaces."
echo "No macOS reboot required!"
echo "=============================================="
echo

printf "SRecord: %s\nPython venv: %s\npyOCD: %s\nIDAP-Link: %s\n" "$SRECORD_DIR" "$PYOCD_DIR" "$PYOCD" "$ROOT/Tools/IDAP-Link"

printf "LLVM (Arm + RISC-V): %s\n" "$LLVM_STATUS"
