/**-------------------------------------------------------------------------
@file	uart_r9a02.cpp

@brief	R9A02G021 SCI UART driver for IOsonata.

R9A02G021 has four populated SCI channels (SCI0, SCI1, SCI2, SCI9 -- the
RA2 family numbers them sparsely).  This driver maps the IOsonata DevNo
field as a logical index:

    DevNo = 0  ->  SCI0
    DevNo = 1  ->  SCI1
    DevNo = 2  ->  SCI2
    DevNo = 3  ->  SCI9

The driver currently provides polling-mode Tx/Rx good for printf-style
debug output.  Interrupt + CFIFO mode is sketched but the four ISR
hookups (RXI/TXI/TEI/ERI per channel via IELSR routing) are marked TODO
and need wiring into IELSR<n>_IRQHandler overrides in Vectors_R9A02.c
before they can fire.  See the InitInterrupts() helper for the routing
point.

Baud-rate calculation uses the standard RA SCI formula:

    BRR = PCLKB / (32 * 2^(2*CKS-1) * baud) - 1

i.e. BRR = PCLKB / (n_div * baud) - 1, where n_div is 32, 128, 512, or
2048 for CKS = 0, 1, 2, 3 respectively.  We pick the smallest CKS that
gives a BRR in [0..255], which minimises rate error.

Register sequence on bring-up follows the R9A02G021 manual section on
"SCI Module Bring-up": clear MSTP, reset SCR, program SMR/SEMR/BRR,
re-enable TE/RE/TIE/RIE in SCR.

@author	Nguyen Hoan Hoang
@date	May 12, 2026

@license

MIT License

Copyright (c) 2026 I-SYST inc. All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------*/
#include <stdint.h>
#include <stddef.h>

#include "coredev/uart.h"
#include "coredev/iopincfg.h"
#include "coredev/system_core_clock.h"
#include "r9a02g021.h"
#include "mstpcr_r9a02.h"

/*===========================================================================
 * Internal channel table -- maps DevNo (0..3) to chip resources
 *==========================================================================*/
typedef struct {
    R9A02_SCI_Type *    pReg;
    R9A02_Module_t      Module;
    R9A02_Event_t       EventRXI;
    R9A02_Event_t       EventTXI;
    R9A02_Event_t       EventTEI;
    R9A02_Event_t       EventERI;
} SciInfo_t;

static const SciInfo_t s_SciInfo[4] = {
    /* DevNo 0 -> SCI0 */
    { R9A02_SCI0, R9A02_MOD_SCI0,
      R9A02_EVENT_SCI0_RXI, R9A02_EVENT_SCI0_TXI,
      R9A02_EVENT_SCI0_TEI, R9A02_EVENT_SCI0_ERI },
    /* DevNo 1 -> SCI1 */
    { R9A02_SCI1, R9A02_MOD_SCI1,
      R9A02_EVENT_SCI1_RXI, R9A02_EVENT_SCI1_TXI,
      R9A02_EVENT_SCI1_TEI, R9A02_EVENT_SCI1_ERI },
    /* DevNo 2 -> SCI2 */
    { R9A02_SCI2, R9A02_MOD_SCI2,
      R9A02_EVENT_SCI2_RXI, R9A02_EVENT_SCI2_TXI,
      R9A02_EVENT_SCI2_TEI, R9A02_EVENT_SCI2_ERI },
    /* DevNo 3 -> SCI9 */
    { R9A02_SCI9, R9A02_MOD_SCI9,
      R9A02_EVENT_SCI9_RXI, R9A02_EVENT_SCI9_TXI,
      R9A02_EVENT_SCI9_TEI, R9A02_EVENT_SCI9_ERI }
};

/* One pDev per channel, for ISR look-up later. */
static UARTDev_t *s_pDevByCh[4] = { nullptr, nullptr, nullptr, nullptr };

/*===========================================================================
 * Baud-rate solver
 *
 * Find the (CKS, BRR) pair giving the smallest rate error for the
 * requested baud at the current PCLKB.  Returns false on failure.
 *==========================================================================*/
static bool ComputeBrr(uint32_t Pclkb, uint32_t Baud, uint8_t *pCks, uint8_t *pBrr)
{
    /* n_div for CKS = 0, 1, 2, 3 is 32, 128, 512, 2048. */
    static const uint16_t n_div[4] = { 32U, 128U, 512U, 2048U };

    for (uint8_t cks = 0U; cks < 4U; cks++)
    {
        uint32_t denom = (uint32_t)n_div[cks] * Baud;
        if (denom == 0U)
        {
            continue;
        }
        uint32_t brr = (Pclkb + (denom / 2U)) / denom;      /* rounded */
        if (brr == 0U)
        {
            continue;
        }
        brr -= 1U;
        if (brr <= 255U)
        {
            *pCks = cks;
            *pBrr = (uint8_t)brr;
            return true;
        }
    }
    return false;
}

/*===========================================================================
 * Pin mux -- caller provides pin map; we just route to peripheral mode.
 *
 * R9A02G021 SCI PSEL codes: SCI0=0x0B, SCI1=0x0C, SCI2=0x0D, SCI9=0x0E.
 * We pick the right PSEL from DevNo.
 *==========================================================================*/
static const uint32_t s_SciPsel[4] = {
    PFS_PSEL_SCI0, PFS_PSEL_SCI1, PFS_PSEL_SCI2, PFS_PSEL_SCI9
};

extern "C" void IOPinSetSense(int PortNo, int PinNo, IOPINSENSE Sense);   /* declared elsewhere */
extern "C" void IOPinConfig(int PortNo, int PinNo, int PinOp, IOPINDIR Dir,
                            IOPINRES Resistor, IOPINTYPE Type);

static void ConfigSciPin(int DevNo, const IOPinCfg_t *pPin)
{
    if (pPin == nullptr)
    {
        return;
    }
    /* Delegate the PFS unlock + PSEL programming to iopincfg_r9a02; pass
     * PinOp = SCI PSEL code so the existing IOPinConfig() routes the pin
     * to the alternate function.  See iopincfg_r9a02.c implementation. */
    IOPinConfig(pPin->PortNo, pPin->PinNo,
                (int)(s_SciPsel[DevNo] >> PFS_PSEL_Pos),
                IOPINDIR_INPUT,
                pPin->Res,
                pPin->Type);
}

/*===========================================================================
 * Init helpers
 *==========================================================================*/
static void Reset(R9A02_SCI_Type *p)
{
    p->SCR = 0;                 /* disable TE/RE/TIE/RIE/TEIE first */
    (void)p->SSR;
    p->SSR = 0;                 /* clear status flags */
}

static void ProgramFraming(R9A02_SCI_Type *p, const UARTCfg_t *pCfg)
{
    uint8_t smr = 0;

    /* CM=0 async, CHR=0 8-bit, PE/PM per parity, STOP = 1 or 2, CKS programmed
     * via separate path below to keep this function simple. */
    if (pCfg->Parity != UART_PARITY_NONE)
    {
        smr |= (1U << 6);                                 /* PE = parity enable */
        if (pCfg->Parity == UART_PARITY_ODD)
        {
            smr |= (1U << 4);                             /* PM = odd */
        }
    }
    if (pCfg->StopBits == 2)
    {
        smr |= (1U << 3);                                 /* STOP = 2 */
    }
    if (pCfg->DataBits == 7)
    {
        smr |= (1U << 6);                                 /* CHR1 set, CHR=01 -> 7-bit */
    }
    /* CKS bits [1:0] -- programmed by SetBaud below. */
    p->SMR = smr;

    p->SCMR = 0xF2;             /* default per RA manual -- SMIF=0, async */
    p->SEMR = 0x00;             /* ABCS=0, BGDM=0 -- standard 32x clock per bit */
    p->SNFR = 0x00;
    p->SIMR1 = 0x00;
    p->SIMR2 = 0x00;
    p->SIMR3 = 0x00;
    p->SPMR  = 0x00;
}

static bool SetBaud(R9A02_SCI_Type *p, uint32_t Baud)
{
    uint8_t cks, brr;
    uint32_t pclkb = SystemCoreClockGet();      /* PCLKB == ICLK in default config */

    if (!ComputeBrr(pclkb, Baud, &cks, &brr))
    {
        return false;
    }
    p->SMR = (uint8_t)((p->SMR & ~0x03U) | cks);
    p->BRR = brr;
    return true;
}

/*===========================================================================
 * DevIntrf entry points
 *==========================================================================*/

static int  UARTRxData (DevIntrf_t * const pDevIntrf, uint8_t *pBuff, int Bufflen);
static int  UARTTxData (DevIntrf_t * const pDevIntrf, const uint8_t *pData, int Datalen);
static bool UARTStartRx(DevIntrf_t * const pDevIntrf, uint32_t DevAddr);
static void UARTStopRx (DevIntrf_t * const pDevIntrf);
static bool UARTStartTx(DevIntrf_t * const pDevIntrf, uint32_t DevAddr);
static void UARTStopTx (DevIntrf_t * const pDevIntrf);
static void UARTDisable(DevIntrf_t * const pDevIntrf);
static void UARTEnable (DevIntrf_t * const pDevIntrf);

static int UARTRxData(DevIntrf_t * const pDevIntrf, uint8_t *pBuff, int Bufflen)
{
    UARTDev_t *pDev = (UARTDev_t *)pDevIntrf->pDevData;
    R9A02_SCI_Type *p = ((SciInfo_t *)pDev->pObj)->pReg;
    int n = 0;

    while (n < Bufflen)
    {
        if (!(p->SSR & SCI_SSR_RDRF))
        {
            break;                  /* no more bytes available right now */
        }
        pBuff[n++] = p->RDR;
        /* RDRF is cleared by reading RDR. */
    }
    return n;
}

static int UARTTxData(DevIntrf_t * const pDevIntrf, const uint8_t *pData, int Datalen)
{
    UARTDev_t *pDev = (UARTDev_t *)pDevIntrf->pDevData;
    R9A02_SCI_Type *p = ((SciInfo_t *)pDev->pObj)->pReg;
    int n = 0;

    while (n < Datalen)
    {
        /* Wait for TDR empty.  Blocking: acceptable for printf path. */
        while (!(p->SSR & SCI_SSR_TDRE))
        {
            /* spin */
        }
        p->TDR = pData[n++];
    }
    /* Wait until last bit clocked out so the caller can safely tear down. */
    while (!(p->SSR & SCI_SSR_TEND))
    {
        /* spin */
    }
    return n;
}

static bool UARTStartRx(DevIntrf_t * const pDevIntrf, uint32_t /*DevAddr*/)
{
    (void)pDevIntrf;
    return true;                /* always ready in polling mode */
}

static void UARTStopRx(DevIntrf_t * const /*pDevIntrf*/)
{
    /* nothing to do in polling mode */
}

static bool UARTStartTx(DevIntrf_t * const pDevIntrf, uint32_t /*DevAddr*/)
{
    (void)pDevIntrf;
    return true;
}

static void UARTStopTx(DevIntrf_t * const /*pDevIntrf*/)
{
    /* nothing to do */
}

static void UARTDisable(DevIntrf_t * const pDevIntrf)
{
    UARTDev_t *pDev = (UARTDev_t *)pDevIntrf->pDevData;
    SciInfo_t *info = (SciInfo_t *)pDev->pObj;
    info->pReg->SCR = 0;
    R9A02_ModuleStop(info->Module);
}

static void UARTEnable(DevIntrf_t * const pDevIntrf)
{
    UARTDev_t *pDev = (UARTDev_t *)pDevIntrf->pDevData;
    SciInfo_t *info = (SciInfo_t *)pDev->pObj;
    R9A02_ModuleStart(info->Module);
    info->pReg->SCR = SCI_SCR_TE | SCI_SCR_RE;
}

/*===========================================================================
 * Optional: interrupt + CFIFO mode hooks (sketch)
 *
 * To activate, the application must:
 *   1. Allocate four IELSR slots (e.g. 0..3 for SCI0)
 *   2. Route them by writing the event IDs into ICU->IELSR[n]
 *   3. Override IELSR<n>_IRQHandler in user code OR via a dispatch
 *      function installed here
 *   4. Set SCR.TIE / SCR.RIE / SCR.TEIE bits
 *
 * Left commented-out below as a structural reference; not yet plumbed
 * into the polling Init path.  Wiring this up needs the IELSR slot
 * allocator that should live in a future irq_r9a02.c / irq_r9a02.h --
 * outside the scope of this initial driver.
 *==========================================================================*/

#if 0
static void InitInterrupts(int DevNo, int FirstSlot)
{
    const SciInfo_t *info = &s_SciInfo[DevNo];
    R9A02_ICU->IELSR[FirstSlot + 0] = info->EventRXI;
    R9A02_ICU->IELSR[FirstSlot + 1] = info->EventTXI;
    R9A02_ICU->IELSR[FirstSlot + 2] = info->EventTEI;
    R9A02_ICU->IELSR[FirstSlot + 3] = info->EventERI;
    /* TODO: install per-slot handler that reads pDev from s_pDevByCh[DevNo]
     *       and dispatches RxData / TxReady / TxEnd / Error to EvtCallback. */
}
#endif

/*===========================================================================
 * Public init
 *==========================================================================*/

extern "C" bool UARTInit(UARTDev_t * const pDev, const UARTCfg_t *pCfgData)
{
    if (pDev == nullptr || pCfgData == nullptr)
    {
        return false;
    }
    if (pCfgData->DevNo < 0 || pCfgData->DevNo > 3)
    {
        return false;
    }

    SciInfo_t *info = (SciInfo_t *)&s_SciInfo[pCfgData->DevNo];
    R9A02_SCI_Type *p = info->pReg;

    /* 1. Bring the SCI clock up. */
    R9A02_ModuleStart(info->Module);

    /* 2. Pin mux. */
    const IOPinCfg_t *pins = (const IOPinCfg_t *)pCfgData->pIOPinMap;
    if (pins != nullptr && pCfgData->NbIOPins >= 2)
    {
        ConfigSciPin(pCfgData->DevNo, &pins[UARTPIN_RX_IDX]);
        ConfigSciPin(pCfgData->DevNo, &pins[UARTPIN_TX_IDX]);
    }

    /* 3. Reset SCI, then program framing and baud. */
    Reset(p);
    ProgramFraming(p, pCfgData);
    if (!SetBaud(p, (uint32_t)pCfgData->Rate))
    {
        R9A02_ModuleStop(info->Module);
        return false;
    }

    /* 4. Enable TX and RX. */
    p->SCR = SCI_SCR_TE | SCI_SCR_RE;

    /* 5. Populate UARTDev_t bookkeeping. */
    pDev->Mode        = pCfgData->Mode;
    pDev->Duplex      = pCfgData->Duplex;
    pDev->Rate        = pCfgData->Rate;
    pDev->DataBits    = pCfgData->DataBits;
    pDev->Parity      = pCfgData->Parity;
    pDev->StopBits    = pCfgData->StopBits;
    pDev->FlowControl = pCfgData->FlowControl;
    pDev->EvtCallback = pCfgData->EvtCallback;
    pDev->pObj        = (void *)info;
    pDev->bRxReady    = false;
    pDev->bTxReady    = true;
    pDev->LineState   = 0;
    pDev->RxOvrErrCnt = pDev->ParErrCnt = pDev->FramErrCnt = 0;
    pDev->RxDropCnt   = pDev->TxDropCnt = 0;

    pDev->DevIntrf.pDevData       = pDev;
    pDev->DevIntrf.Type           = DEVINTRF_TYPE_UART;
    pDev->DevIntrf.Disable        = UARTDisable;
    pDev->DevIntrf.Enable         = UARTEnable;
    pDev->DevIntrf.GetRate        = nullptr;
    pDev->DevIntrf.SetRate        = nullptr;
    pDev->DevIntrf.StartRx        = UARTStartRx;
    pDev->DevIntrf.RxData         = UARTRxData;
    pDev->DevIntrf.StopRx         = UARTStopRx;
    pDev->DevIntrf.StartTx        = UARTStartTx;
    pDev->DevIntrf.TxData         = UARTTxData;
    pDev->DevIntrf.StopTx         = UARTStopTx;
    pDev->DevIntrf.MaxRetry       = 5;
    pDev->DevIntrf.bDma           = false;
    pDev->DevIntrf.PowerOff       = nullptr;
    pDev->DevIntrf.EnCnt          = 1;

    s_pDevByCh[pCfgData->DevNo] = pDev;

    /* TODO: if pCfgData->bIntMode is set, allocate IELSR slots and route
     * RXI/TXI/TEI/ERI events to them, then enable RIE/TIE/TEIE in SCR.
     * Polling Tx/Rx is the current default. */

    return true;
}

extern "C" void UARTSetCtrlLineState(UARTDev_t * const /*pDev*/, uint32_t /*LineState*/)
{
    /* SCI does not have modem-control lines in async mode -- DTR/RTS handled
     * separately via GPIO when flow control is wired. */
}
