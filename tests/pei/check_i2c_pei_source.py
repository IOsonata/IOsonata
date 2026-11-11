#!/usr/bin/env python3
"""nRF52 TWI/TWIM/TWIS PEI source and DMA ownership regression checks.

These checks do not simulate bus signals, ACK/NACK, IRQ callbacks or
replace a build against the installed Nordic CMSIS header version.
"""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
DRIVER = (ROOT / "ARM/Nordic/src/i2c_nrfx.cpp").read_text()
HEADER = (ROOT / "include/coredev/i2c.h").read_text()
APP = (ROOT / "exemples/pei/pei_twi_demo.cpp").read_text()


def section(start, end):
    assert DRIVER.count(start) == 1, start
    assert DRIVER.count(end) == 1, end
    return DRIVER.split(start, 1)[1].split(end, 1)[0]


def main():
    events = section("static PeiEndpoint_t I2CPeiEventEndpoint(",
                     "static PeiEndpoint_t I2CPeiTaskEndpoint(")
    tasks = section("static PeiEndpoint_t I2CPeiTaskEndpoint(",
                    "static bool I2CPeiReady(")
    ready = section("static bool I2CPeiReady(",
                    'bool I2CPreparePeiTransfer(')
    prep = section('bool I2CPreparePeiTransfer(',
                   'bool I2CReleasePeiTransfer(')
    release = section('bool I2CReleasePeiTransfer(',
                      'bool I2CEnablePeiEvent(')
    connect = section('bool I2CConnectTask(',
                      'bool I2CDisconnectTask(')

    for event in ("STOPPED", "ERROR", "SUSPENDED", "RXDREADY",
                  "TXDSENT", "BB"):
        assert "&dev->pReg->EVENTS_" + event in events
    for event in ("STOPPED", "ERROR", "SUSPENDED", "RXSTARTED",
                  "TXSTARTED", "LASTRX", "LASTTX"):
        assert "&dev->pDmaReg->EVENTS_" + event in events
    for event in ("STOPPED", "ERROR", "RXSTARTED", "TXSTARTED",
                  "WRITE", "READ"):
        assert "&dev->pDmaSReg->EVENTS_" + event in events
    for task in ("STARTRX", "STARTTX", "STOP", "SUSPEND", "RESUME"):
        assert "&dev->pDmaReg->TASKS_" + task in tasks
    assert "TASKS_PREPARERX" not in tasks
    assert "TASKS_PREPARETX" not in tasks
    assert "pDev->Cfg.DevNo < TWIM_COUNT" in ready
    assert "pDev->Cfg.DevNo < TWIS_COUNT" in ready
    assert "pDev->Cfg.DevNo < TWI_COUNT" in ready

    assert "pDev->Cfg.Mode != I2CMODE_MASTER" in prep
    assert "pDev->Cfg.AddrType != I2CADDR_TYPE_NORMAL" in prep
    assert "pDev->Cfg.bIntEn" in prep
    assert "TxLen > NRFX_I2C_TXBUF_SIZE" in prep
    assert "RxLen > NRFX_I2C_DMA_MAXCNT" in prep
    assert "((TxLen == 0U) == (RxLen == 0U))" in prep
    assert "atomic_flag_test_and_set(&pDev->DevIntrf.bBusy)" in prep
    assert "memcpy(dev->TxBuf, pTx, TxLen);" in prep
    assert "dev->pDmaReg->TXD.PTR =" in prep
    assert "dev->pDmaReg->RXD.PTR =" in prep
    assert "TWIM_SHORTS_LASTTX_STOP_Msk" in prep
    assert "TWIM_SHORTS_LASTRX_STOP_Msk" in prep
    assert "dev->pDmaReg->TASKS_STARTTX" not in prep
    assert "dev->pDmaReg->TASKS_STARTRX" not in prep
    assert "dev->bPeiPrepared = true;" in prep
    assert "dev->bPeiPrepared = false;" in release
    assert "atomic_flag_clear(&pDev->DevIntrf.bBusy)" in release
    assert "TASKS_RESUME = 1U" in release
    assert "TASKS_STOP = 1U" in release
    assert "I2C_PEI_TASK_STARTRX" in connect
    assert "I2C_PEI_TASK_STARTTX" in connect
    assert "dev->bPeiPrepared" in connect
    assert "#if defined(NRF52_SERIES) && defined(TWIM_PRESENT)" in connect
    assert "NRFX_TWI_TX(dev->pDmaReg).MAXCNT" in connect
    assert "NRFX_TWI_RX(dev->pDmaReg).MAXCNT" in connect
    assert "dev->pDmaReg->TXD.MAXCNT" not in connect
    assert "dev->pDmaReg->RXD.MAXCNT" not in connect

    for name in ("PreparePeiTransfer", "ReleasePeiTransfer",
                 "EnablePeiEvent", "DisablePeiEvent",
                 "ConnectTask", "DisconnectTask"):
        assert "bool I2C" + name + "(" in HEADER
        assert "bool I2C" + name + "(" in DRIVER
        assert 'extern "C" bool I2C' + name + "(" not in DRIVER
    for name in ("PeiBindEvent", "PeiGetEventEndpoint",
                 "PeiUnbindEvent", "PeiConnectTask",
                 "PeiDisconnectTask"):
        assert "#pragma weak " + name in DRIVER

    rate = section("uint32_t nRFxI2CSetRate(", "bool nRFxI2CStartRx(")
    declarations = rate.split("if (dev->bPeiPrepared)", 1)[0]
    assert "uint32_t best = 0;" in declarations
    assert "uint32_t error = UINT32_MAX;" in declarations
    assert "uint32_t regval = 0;" in declarations

    assert "dev->bPeiPrepared" in DRIVER
    assert "if (dev->bPeiPrepared)" in DRIVER
    assert "if (owner->bPeiPrepared)" in DRIVER
    assert "I2C_PEI_EVENT_ERROR" in APP
    assert "I2C_PEI_EVENT_STOPPED" in APP
    assert "I2C_PEI_TASK_STARTTX" in APP
    assert "I2C_PEI_TASK_STOP" in APP
    assert "g_PeiTwiStatus = 1;" in APP
    assert 'extern "C"' not in APP
    assert "NRF_TWI" not in APP
    assert "NRF_TWIM" not in APP
    assert "printf(" not in APP
    print("nRF52 TWI/TWIM/TWIS PEI source checks: PASS")


if __name__ == "__main__":
    main()
