#!/bin/sh
# Compile SAM D11/D21 MCU sources with Clang for Cortex-M0+.
# Run from any directory. No generated files are committed.
set -eu

ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
CLANG=${CLANG:-clang}
CLANGXX=${CLANGXX:-clang++}
OUT=${TMPDIR:-/tmp}/iosonata-samd-clang-$$
trap 'rm -rf "$OUT"' EXIT HUP INT TERM
mkdir -p "$OUT"

# The native host sysroot must not be used for an ARM target.
# ARM_SYSROOT points to the Arm GNU toolchain C library/sysroot.
SYSROOT_ARGS=""
if [ -n "${ARM_SYSROOT:-}" ]
then
	SYSROOT_ARGS="--sysroot=$ARM_SYSROOT"
fi

if ! command -v "$CLANG" >/dev/null 2>&1 || ! command -v "$CLANGXX" >/dev/null 2>&1
then
	echo "Clang C/C++ compilers are required" >&2
	exit 1
fi

if ! printf "#include <stdio.h>\n" | "$CLANG" --target=arm-none-eabi -mcpu=cortex-m0plus $SYSROOT_ARGS -x c -fsyntax-only - >/dev/null 2>&1
then
	echo "ARM target C headers are unavailable. Set ARM_SYSROOT to an Arm GNU sysroot." >&2
	exit 1
fi

COMMON="-target arm-none-eabi -mcpu=cortex-m0plus -mthumb $SYSROOT_ARGS -ffreestanding -fno-builtin -Wall -Wextra -Werror"
CMSIS="-I$ROOT/ARM/CMSIS/Core/Include"
GENERIC="-I$ROOT/include"
FAMILY="-I$ROOT/ARM/Microchip/SAMD/include"

for DEVICE in SAMD11 SAMD21
do
	case "$DEVICE" in
		SAMD11)
			PART=__SAMD11D14A__
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
	"$CLANG" $COMMON -std=c11 -D"$PART" $CMSIS $GENERIC $FAMILY -I"$PART_INC" \
		-c "$VECTORS" -o "$OUT/$DEVICE-vectors.o"
	"$CLANG" $COMMON -std=c11 -D"$PART" $CMSIS $GENERIC $FAMILY -I"$PART_INC" \
		-c "$ROOT/ARM/Microchip/SAMD/src/system_samd.c" -o "$OUT/$DEVICE-system.o"
	"$CLANG" $COMMON -std=c11 -D"$PART" $CMSIS $GENERIC $FAMILY -I"$PART_INC" \
		-c "$ROOT/ARM/Microchip/SAMD/src/iopincfg_samd.c" -o "$OUT/$DEVICE-gpio.o"

	if [ "$DEVICE" = SAMD21 ]
	then
		"$CLANG" $COMMON -std=c11 -D"$PART" $CMSIS $GENERIC $FAMILY -I"$PART_INC" \
			-c "$ROOT/ARM/Microchip/SAMD/SAMD21/src/sercom_clock_samd21.c" \
			-o "$OUT/$DEVICE-sercom-clock.o"
	fi
done
"$CLANGXX" $COMMON -std=c++17 -fno-exceptions -fno-rtti -D__SAMD21J18A__ \
	$CMSIS $GENERIC $FAMILY -I"$ROOT/ARM/Microchip/SAMD/SAMD21/include" \
	-c "$ROOT/ARM/Microchip/SAMD/SAMD21/src/uart_samd21.cpp" \
	-o "$OUT/SAMD21-uart.o"
echo "SAMD C and C++ source compile completed"
