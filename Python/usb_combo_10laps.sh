#!/bin/bash

CMD=(
    ./.venv/bin/python3
    ../IOsonata/Python/usb_combo_stress.py
    --loop-port /dev/cu.usbmodem142301
    --prbs-port /dev/cu.usbmodem142303
    --duration 200
)

for lap in {1..10}; do
    echo
    echo "============================================================"
    echo "  LAP $lap / 10"
    echo "============================================================"

    "${CMD[@]}"
    rc=$?

    if [ $rc -ne 0 ]; then
        echo
        echo "============================================================"
        echo "  FAIL at LAP $lap / 10"
        echo "============================================================"
        exit $rc
    fi

    echo
    echo "-------------------- LAP $lap / 10 PASS --------------------"
done

echo
echo "============================================================"
echo "  ALL 10 LAPS PASS"
echo "============================================================"
