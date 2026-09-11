#!/usr/bin/env python3
"""Guard the nRF52 host-suspend policy that is not built by host USB tests."""

from pathlib import Path


SOURCE = Path(__file__).parents[2] / "ARM/Nordic/src/usb_ctrlr_nrfx.cpp"


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
interrupt = function_body(source, "extern \"C\" void USBD_IRQHandler(void)")
sof_start = interrupt.index("if ((intStatus & USBD_INTEN_SOF_Msk) != 0)")
sof = brace_body(interrupt, interrupt.index("{", sof_start))

assert "if (!s_UsbdCfg.bLowPowerSuspend ||" in enter_low_power, (
    "USBD low-power entry must be disabled when bLowPowerSuspend is false"
)
assert (
    "atomic_store(&s_SuspendPending, s_UsbdCfg.bLowPowerSuspend);" in interrupt
), "bus suspend must not request peripheral low-power unconditionally"
assert "nRFUsbdHostResumeDetected();" in sof, (
    "SOF handling must retain the anomaly-211 host-resume recovery path"
)

print("nrf_usb_suspend_policy_test: PASS")
