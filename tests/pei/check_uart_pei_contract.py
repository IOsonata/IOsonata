#!/usr/bin/env python3
"""nRF52 UART/UARTE PEI source and ownership regression checks.

These tests are static source/API contracts; the target must still be
built with its installed CMSIS header and validated on MCU hardware.
"""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
DRIVER = (ROOT / "ARM/Nordic/src/uart_nrfx.cpp").read_text()
HEADER = (ROOT / "include/coredev/uart.h").read_text()
DEMO = (ROOT / "exemples/pei/pei_uart_demo.cpp").read_text()


def section(begin, end):
    assert DRIVER.count(begin) == 1, begin
    assert DRIVER.count(end) == 1, end
    return DRIVER.split(begin, 1)[1].split(end, 1)[0]


def main():
    events = section("static PeiEndpoint_t UARTPeiEventEndpoint(",
                     "static PeiEndpoint_t UARTPeiTaskEndpoint(")
    tasks = section("static PeiEndpoint_t UARTPeiTaskEndpoint(",
                    'bool UARTPreparePeiTx(')
    ready = section("static bool UARTPeiReady(",
                    "static PeiEndpoint_t UARTPeiEventEndpoint(")
    prep = section('bool UARTPreparePeiTx(',
                   'bool UARTReleasePeiTx(')
    release = section('bool UARTReleasePeiTx(',
                      'bool UARTEnablePeiEvent(')
    connect = section('bool UARTConnectPeiTask(',
                      'bool UARTDisconnectPeiTask(')

    for event in ("CTS", "NCTS", "RXDRDY", "TXDRDY", "ERROR", "RXTO"):
        assert "&dev->pReg->EVENTS_" + event in events
        assert "&dev->pDmaReg->EVENTS_" + event in events
    for event in ("ENDRX", "ENDTX", "RXSTARTED", "TXSTARTED",
                  "TXSTOPPED"):
        assert "&dev->pDmaReg->EVENTS_" + event in events
        assert "&dev->pReg->EVENTS_" + event not in events

    for task in ("STARTTX", "STOPTX"):
        assert "&dev->pDmaReg->TASKS_" + task in tasks
    for task in ("STARTRX", "STOPRX", "FLUSHRX"):
        assert "TASKS_" + task not in tasks
    assert "dev->pUartDev->DevIntrf.bDma" in events
    assert "dev->DevNo < UARTE_COUNT" in ready
    assert "dev->DevNo < UART_COUNT" in ready

    assert "pDev->DevIntrf.bIntEn" in prep
    assert "pDev->FlowControl != UART_FLWCTRL_NONE" in prep
    assert "sizeof(dev->TxDmaMem)" in prep
    assert "CFifoUsed(pDev->hTxFifo)" in prep
    assert "atomic_flag_test_and_set(&pDev->DevIntrf.bBusy)" in prep
    assert "memcpy(dev->TxDmaMem, pTx, TxLen);" in prep
    assert "NRFX_UARTE_TXD_PTR(dev->pDmaReg) =" in prep
    assert "NRFX_UARTE_TXD_MAXCNT(dev->pDmaReg) = TxLen;" in prep
    assert "dev->bPeiTxPrepared = true;" in prep
    assert "TASKS_STARTTX" not in prep
    assert "EVENTS_TXSTARTED" in release
    assert "EVENTS_TXSTOPPED" in release
    assert "NRFX_UARTE_TASKS_STOPTX(dev->pDmaReg) = 1U;" in release
    assert "dev->bPeiTxPrepared = false;" in release
    assert "dev->bPeiTxPrepared" in connect

    for name in ("PreparePeiTx", "ReleasePeiTx", "EnablePeiEvent",
                 "DisablePeiEvent", "ConnectPeiTask", "DisconnectPeiTask"):
        assert "bool UART" + name + "(" in HEADER
        assert 'bool UART' + name + "(" in DRIVER
        assert 'extern "C" bool UART' + name + "(" not in DRIVER
    for name in ("PeiBindEvent", "PeiGetEventEndpoint",
                 "PeiUnbindEvent", "PeiConnectTask", "PeiDisconnectTask"):
        assert "#pragma weak " + name in DRIVER
    assert "dev->bPeiTxPrepared" in DRIVER
    assert "return !dev->bPeiTxPrepared;" in DRIVER
    rate = section("static uint32_t nRFUARTSetRate(",
                   "static bool nRFUARTStartRx(")
    tx = section("static int nRFUARTTxData(",
                 "static void nRFUARTStopTx(")
    for body, declarations in (
            (rate, ("uint32_t regval;", "int rate;")),
            (tx, ("int cnt = 0;", "int rtry = pDev->MaxRetry;"))):
        entry = body.split("if (dev->bPeiTxPrepared)", 1)[0]
        for declaration in declarations:
            assert declaration in entry, declaration


    assert "UART_PEI_EVENT_ENDTX" in DEMO
    assert "UART_PEI_EVENT_TXSTOPPED" in DEMO
    assert "UART_PEI_TASK_STARTTX" in DEMO
    assert "UART_PEI_TASK_STOPTX" in DEMO
    assert "g_PeiUartStatus = 1;" in DEMO
    assert 'extern "C"' not in DEMO
    assert "NRF_UARTE" not in DEMO
    assert "printf(" not in DEMO
    print("nRF52 UART/UARTE PEI source contracts: PASS")


if __name__ == "__main__":
    main()
