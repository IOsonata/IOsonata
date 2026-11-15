#!/bin/sh
# Compile SAM D11/D21 MCU sources for Cortex-M0+ with Arm GNU or Clang.
# Run from any directory. No generated files are committed.
set -eu

ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
OUT=${TMPDIR:-/tmp}/iosonata-samd-compile-$$
trap 'rm -rf "$OUT"' EXIT HUP INT TERM
mkdir -p "$OUT"

# Prefer the xPack Arm GNU toolchain installed by IOsonata.
# Also find it when its bin directory is not exported in PATH on macOS.
ARM_GCC=${ARM_GCC:-arm-none-eabi-gcc}
ARM_GXX=${ARM_GXX:-arm-none-eabi-g++}
if ! command -v "$ARM_GCC" >/dev/null 2>&1 || ! command -v "$ARM_GXX" >/dev/null 2>&1
then
	for candidate in /opt/xPacks/xpack-arm-none-eabi-*/bin/arm-none-eabi-gcc
	do
		if [ -x "$candidate" ]
		then
			ARM_GCC=$candidate
			ARM_GXX=${candidate%gcc}g++
		fi
	done
fi

if command -v "$ARM_GCC" >/dev/null 2>&1 && command -v "$ARM_GXX" >/dev/null 2>&1
then
	CC=$ARM_GCC
	CXX=$ARM_GXX
	COMMON="-mcpu=cortex-m0plus -mthumb -D__PROGRAM_START -ffreestanding -fno-builtin -Wall -Wextra -Werror"
	echo "Using Arm GNU: $CC"
else
	CLANG=${CLANG:-clang}
	CLANGXX=${CLANGXX:-clang++}
	if ! command -v "$CLANG" >/dev/null 2>&1 || ! command -v "$CLANGXX" >/dev/null 2>&1
	then
		echo "Install the xPack Arm GNU toolchain or Clang with an Arm sysroot" >&2
		exit 1
	fi
	SYSROOT_ARGS=""
	if [ -n "${ARM_SYSROOT:-}" ]
	then
		SYSROOT_ARGS="--sysroot=$ARM_SYSROOT"
	fi
	if ! printf '#include <stdint.h>\n' | "$CLANG" --target=arm-none-eabi -mcpu=cortex-m0plus $SYSROOT_ARGS -x c -fsyntax-only - >/dev/null 2>&1
	then
		echo "Clang cannot find ARM target headers. Install xPack Arm GNU or set ARM_SYSROOT." >&2
		exit 1
	fi
	CC=$CLANG
	CXX=$CLANGXX
	COMMON="-target arm-none-eabi -mcpu=cortex-m0plus -mthumb $SYSROOT_ARGS -D__PROGRAM_START -ffreestanding -fno-builtin -Wall -Wextra -Werror"
	echo "Using Clang: $CC"
fi

CMSIS="-I$ROOT/ARM/CMSIS/Core/Include"
GENERIC="-I$ROOT/include -I$ROOT/ARM/include"
FAMILY="-I$ROOT/ARM/Microchip/SAMD/include"

for DEVICE in SAMD11 SAMD21
do
	case "$DEVICE" in
		SAMD11)
			PART=__SAMD11D14AM__
			PART_INC="$ROOT/ARM/Microchip/SAMD/SAMD11/include"
			VECTORS="$ROOT/ARM/Microchip/SAMD/SAMD11/src/vectors_samd11.c"
			;;
		SAMD21)
			PART=__SAMD21J18A__
			PART_INC="$ROOT/ARM/Microchip/SAMD/SAMD21/include"
			VECTORS="$ROOT/ARM/Microchip/SAMD/SAMD21/src/vectors_samd21.c"
			;;
	esac

	echo "Checking $DEVICE"
	"$CC" $COMMON -std=c11 -D"$PART" $CMSIS $GENERIC $FAMILY -I"$PART_INC" \
		-c "$VECTORS" -o "$OUT/$DEVICE-vectors.o"
	"$CC" $COMMON -std=c11 -D"$PART" $CMSIS $GENERIC $FAMILY -I"$PART_INC" \
		-c "$ROOT/ARM/Microchip/SAMD/src/system_samd.c" -o "$OUT/$DEVICE-system.o"
	"$CXX" $COMMON -std=gnu++17 -fno-exceptions -fno-rtti -D"$PART" \
		$CMSIS $GENERIC $FAMILY -I"$PART_INC" \
		-c "$ROOT/ARM/Microchip/SAMD/src/system_power_samd.cpp" \
		-o "$OUT/$DEVICE-power.o"
	"$CC" $COMMON -std=c11 -D"$PART" $CMSIS $GENERIC $FAMILY -I"$PART_INC" \
		-c "$ROOT/ARM/Microchip/SAMD/src/iopincfg_samd.c" -o "$OUT/$DEVICE-gpio.o"

	"$CC" $COMMON -std=c11 -D"$PART" $CMSIS $GENERIC $FAMILY -I"$PART_INC" \
		-c "$ROOT/ARM/Microchip/SAMD/src/sercom_samd.c" \
		-o "$OUT/$DEVICE-sercom.o"
	# UART tables initialize hardware fields; remaining static fields are zero.
	"$CXX" $COMMON -Wno-missing-field-initializers -std=gnu++23 -fno-exceptions -fno-rtti -D"$PART" \
		$CMSIS $GENERIC $FAMILY -I"$PART_INC" \
		-c "$ROOT/ARM/Microchip/SAMD/src/uart_samd.cpp" \
		-o "$OUT/$DEVICE-uart.o"
done
echo "SAMD C and C++ source compile completed"
