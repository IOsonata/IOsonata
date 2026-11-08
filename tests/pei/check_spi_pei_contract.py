#!/usr/bin/env python3
"""nRF52 legacy SPI / SPIM / SPIS PEI source contracts.

This does not replace target builds, real SPI transfer testing or
external SPIS master communication.
"""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
DRIVER = (ROOT / "ARM/Nordic/src/spi_nrfx.cpp").read_text()
PRIVATE = (ROOT / "ARM/Nordic/include/spi_nrfx.h").read_text()
HEADER = (ROOT / "include/coredev/spi.h").read_text()
APP = (ROOT / "exemples/pei/pei_spi_demo.cpp").read_text()


def section(start, end):
    assert DRIVER.count(start) == 1, start
    assert DRIVER.count(end) == 1, end
    return DRIVER.split(start, 1)[1].split(end, 1)[0]


def main():
    evts = section("static PeiEndpoint_t SPIPeiEventEndpoint(",
                   "static PeiEndpoint_t SPIPeiTaskEndpoint(")
    tasks = section("static PeiEndpoint_t SPIPeiTaskEndpoint(",
                    "static bool SPIPeiReady(")
    ready = section("static bool SPIPeiReady(",
                    'extern "C" bool SPIPreparePeiTransfer(')
    prep = section('extern "C" bool SPIPreparePeiTransfer(',
                   'extern "C" bool SPIReleasePeiTransfer(')
    connect = section('extern "C" bool SPIConnectPeiTask(',
                      'extern "C" bool SPIDisconnectPeiTask(')

    assert "&dev->pReg->EVENTS_READY" in evts
    for event in ("STARTED", "ENDRX", "ENDTX", "END", "STOPPED"):
        assert "&dev->pDmaReg->EVENTS_" + event in evts
    for event in ("ENDRX", "END", "ACQUIRED"):
        assert "&dev->pDmaSReg->EVENTS_" + event in evts
    for task in ("START", "STOP", "SUSPEND", "RESUME"):
        assert "&dev->pDmaReg->TASKS_" + task in tasks
    assert "TASKS_ACQUIRE" not in tasks
    assert "TASKS_RELEASE" not in tasks
    assert "pDev->Cfg.DevNo < SPIM_COUNT" in ready
    assert "pDev->Cfg.DevNo < SPIS_COUNT" in ready
    assert "pDev->Cfg.DevNo < SPI_COUNT" in ready
    assert "dev->bPeiPrepared" in PRIVATE
    assert "dev->bPeiPrepared" in connect
    assert "SPI_PEI_TASK_START" in APP
    assert "SPI_PEI_TASK_STOP" in APP
    assert "SPI_PEI_EVENT_END" in APP
    assert "SPI_PEI_EVENT_STOPPED" in APP
    assert "g_PeiSpiStatus = 1;" in APP
    assert "SPI_PEI_TASK_START" in HEADER
    assert "SPI_PEI_EVENT_ACQUIRED" in HEADER
    assert "pDev->Cfg.ChipSel != SPICSEL_MAN" in prep
    assert "pDev->Cfg.DataSize != 8U" in prep
    assert "pDev->Cfg.bIntEn" in prep
    assert "RxLen > NRFX_SPI_DMA_MAXCNT" in prep
    assert "TxLen > NRFX_SPI_TXBUF_SIZE" in prep
    assert "memcpy(dev->TxBuf, pTx, TxLen);" in prep
    assert "dev->pDmaReg->RXD.PTR =" in prep
    assert "dev->pDmaReg->TXD.PTR =" in prep
    assert "dev->pDmaReg->TASKS_START" not in prep
    assert "nrf52_errata_58()" in prep
    assert "dev->bPeiPrepared = true;" in prep
    assert "dev->bPeiPrepared = false;" in DRIVER
    for api in ("PeiBindEvent", "PeiGetEventEndpoint",
                "PeiUnbindEvent", "PeiConnectTask",
                "PeiDisconnectTask"):
        assert "#pragma weak " + api in DRIVER
    for name in ("PreparePeiTransfer", "ReleasePeiTransfer",
                 "EnablePeiEvent", "DisablePeiEvent",
                 "ConnectPeiTask", "DisconnectPeiTask"):
        assert "bool SPI" + name + "(" in HEADER
        assert 'extern "C" bool SPI' + name + "(" in DRIVER
    assert 'extern "C"' not in APP
    assert "NRF_SPIM" not in APP
    assert "NRF_SPIS" not in APP
    assert "printf(" not in APP
    print("nRF52 SPI/SPIM/SPIS PEI source contracts: PASS")


if __name__ == "__main__":
    main()
