#!/usr/bin/env python3
"""Guard the nRF52 direct EasyDMA completion policy."""

from pathlib import Path


SOURCE = Path(__file__).parents[2] / "ARM/Nordic/src/usb_ctrlr_nrfx.cpp"


def function_body(source: str, signature: str) -> str:
    start = source.index(signature)
    brace = source.index("{", start)
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
dma_start = function_body(source, "static void nRFUsbdDmaStart(")
dma_release = function_body(source, "static void nRFUsbdDmaRelease(void)")
service = function_body(source, "static void nRFUsbdServicePending(void)")
interrupt = function_body(source, 'extern "C" void USBD_IRQHandler(void)')

assert "nRFUsbdDmaReclaim" not in source
assert "s_LazyInMask" not in source
assert "NRF_USBD->INTENSET = nRFUsbdDmaEndMask(EpAddr);" in dma_start
assert dma_start.index("atomic_store(&s_DmaEpAddr, EpAddr)") < dma_start.index(
    "*pTask = 1"
)
assert "atomic_store(&s_DmaEpAddr, NRFX_USBD_DMA_EP_NONE)" in dma_release
assert "atomic_flag_clear(&s_DmaRunning)" in dma_release
assert "nRFUsbdDmaRelease();" in interrupt
assert interrupt.rindex("nRFUsbdServicePending();") > interrupt.index(
    "nRFUsbdDmaRelease();"
)
assert "nRFUsbdDmaEndIntEnable" not in service

print("nrf_usb_dma_policy_test: PASS")
