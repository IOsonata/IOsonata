#!/bin/sh
# Check SAM D11/D21 Standby sources and generic PowerOffDemo for Cortex-M0+.
# Use the IOsonata-installed xPack Arm toolchain, not the macOS host headers.
set -eu

ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
OUT=${TMPDIR:-/tmp}/iosonata-samd-standby-$$
trap 'rm -rf "$OUT"' EXIT HUP INT TERM
mkdir -p "$OUT"

GCC=${ARM_GCC:-arm-none-eabi-gcc}
GXX=${ARM_GXX:-arm-none-eabi-g++}
if ! command -v "$GCC" >/dev/null 2>&1 ||
	! command -v "$GXX" >/dev/null 2>&1
then
	for item in /opt/xPacks/xpack-arm-none-eabi-*/bin/arm-none-eabi-gcc
	do
		if [ -x "$item" ]
		then
			GCC=$item
			GXX=${item%gcc}g++
		fi
	done
fi
if ! command -v "$GCC" >/dev/null 2>&1 ||
	! command -v "$GXX" >/dev/null 2>&1
then
	echo "xPack Arm GNU compiler not found" >&2
	exit 1
fi

COMMON="-mcpu=cortex-m0plus -mthumb -D__PROGRAM_START -ffreestanding -fno-builtin -Wall -Wextra -Werror"
CMSIS="-I$ROOT/ARM/CMSIS/Core/Include"
INC="-I$ROOT/include -I$ROOT/ARM/include -I$ROOT/ARM/Microchip/SAMD/include"

for DEVICE in SAMD11 SAMD21
do
	case "$DEVICE" in
		SAMD11)
			PART=__SAMD11D14AM__
			;;
		SAMD21)
			PART=__SAMD21J18A__
			;;
	esac
	PARTDIR="$ROOT/ARM/Microchip/SAMD/$DEVICE"
	BOARD="$PARTDIR/exemples/PowerOffDemo/src"
	echo "Checking $DEVICE Standby and PowerOffDemo"
	"$GCC" $COMMON -std=c11 -D"$PART" $CMSIS $INC -I"$PARTDIR/include" \
		-c "$ROOT/ARM/Microchip/SAMD/src/iopincfg_samd.c" \
		-o "$OUT/$DEVICE-gpio.o"
	"$GCC" $COMMON -std=c11 -D"$PART" \
		$CMSIS $INC -I"$PARTDIR/include" -I"$BOARD" \
		-c "$ROOT/ARM/Microchip/SAMD/src/system_sleep_samd.c" \
		-o "$OUT/$DEVICE-sleep.o"
	"$GXX" $COMMON -std=gnu++23 -Wno-missing-field-initializers \
		-fno-exceptions -fno-rtti -D"$PART" \
		$CMSIS $INC -I"$PARTDIR/include" -I"$BOARD" \
		-c "$ROOT/exemples/misc/power_off_demo.cpp" \
		-o "$OUT/$DEVICE-demo.o"
done

echo "SAM D11/D21 Standby source compilation PASS"
