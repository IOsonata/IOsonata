#!/usr/bin/env python3
"""Guard the nRF52 host-suspend policy that is not built by host USB tests."""

from pathlib import Path


SOURCE = Path(__file__).parents[2] / "ARM/Nordic/nRF52/src/usb_ctrlr_nrf52.cpp"


def function_body(source: str, signature: str) -> str:
    start = source.index(signature)
    return brace_body(source, source.index("{", start))


def brace_body(source: str, brace: int) -> str:
    depth = 0

    for pos in range(brace, len(source)):
        if source[pos] == "{":
            depth += 1
        elif source[pos] == "}":
            depth -= 1
            if depth == 0:
                return source[brace + 1 : pos]

    raise AssertionError("unterminated brace block")


source = SOURCE.read_text(encoding="utf-8")
enter_low_power = function_body(source, "static void nRFUsbdTryEnterLowPower(void)")
bus_event = function_body(source, "static void nRFUsbdHandleBusEvent(")
sof = function_body(source, "static void nRFUsbdHandleSof(void)")

assert "if (!s_Usbd.LowPowerSuspend ||" in enter_low_power, (
    "USBD low-power entry must be disabled when bLowPowerSuspend is false"
)
assert "USBD_FLAG_SUSPENDED" in bus_event and "NRF_USBD->LOWPOWER" not in bus_event, (
    "bus suspend must only mark suspend; peripheral low-power entry stays deferred"
)
assert "USBD_FLAG_SUSPEND_PEND" not in source, (
    "suspend draining must be derived from LowPowerSuspend/MAC_AWAKE, not a shadow flag"
)
assert "nRFUsbdHostResumeDetected();" in sof, (
    "SOF handling must retain the anomaly-211 host-resume recovery path"
)

print("nrf_usb_suspend_policy_test: PASS")
