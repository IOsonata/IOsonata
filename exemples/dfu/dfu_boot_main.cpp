/**-------------------------------------------------------------------------
@example	dfu_boot_main.cpp

@brief	Stage 0 DFU boot, with recovery over UART or USB CDC.

Installs a pending image from slot 1 when the layout has one, then starts
the application in slot 0. When there is no application to start, when the
recovery button is held at reset, or when the application asked for it
(DfuRecoveryRequest), it serves recovery instead: a signed image goes
straight into slot 0 and its record.

The same source builds for every target. The flash layout is the one the
project's linker script includes, see docs/architecture/dfu.md; the target
part is in the library, dfu_<mcu>.cpp. What the target picks comes from the
project's board.h:

	DFU_PROTO		DFU_PROTO_WIRE (default): the wire protocol,
					docs/architecture/dfu_wire.md, host tool
					Python/dfu_wire.py. The signature is checked before
					anything is erased or written.
					DFU_PROTO_SMP: SMP, as MCUboot serial recovery, for the
					mcumgr tools. Larger, and it writes before the signature
					can be checked.
	DFU_TRANSPORT	DFU_TRANSPORT_CDC on a part with USB and an IOsonata USB
					device driver (nRF52840 today), DFU_TRANSPORT_UART
					(default) on every other. One of the two, never both.
	UART_PINS		UART pin map, as every UART example uses it
	UART_DEVNO		UART instance, 0 when not defined
	USB_DEVNO		USB instance, 0 when not defined
	DFU_BUT_PORT	recovery button, active low, optional
	DFU_BUT_PIN
	DFU_BOOT_BUFSIZE	largest WRITE data (wire, a power of 2) or SMP request,
					512 when not defined
	DFU_BOOT_FIFODEPTH	link FIFO depth each way, 256 when not defined
	DFU_BOOT_ERASE_MS	worst case one erase unit takes, 2000 when not
					defined, for host timeouts
	DFU_BOOT_ALLOW_DOWNGRADE	1 to take images older than slot 0

The key is the P-256 public key imgtool prints for the signing key:

	imgtool keygen -k my_p256.pem -t ecdsa-p256
	imgtool getpub -k my_p256.pem > dfu_boot_key.c

dfu_boot_key.c goes in the project, next to board.h. Images are then signed
with the private key, -S being the slot the target uploads to: slot 1 where
the layout has one, less its trailer, else slot 0 plus the header room:

	imgtool sign -k my_p256.pem --header-size 0x20 --pad-header --align 4 \
		-v 1.0.0 -S <slot size> app.bin app_signed.bin

and sent:

	python3 dfu_wire.py --port /dev/cu.usbmodem1101 app_signed.bin

@author	Hoang Nguyen Hoan
@date	Sep. 21, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

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

#include "board.h"
#include "cfifo.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "idelay.h"
#include "dfu/dfu_boot.h"
#include "dfu/dfu_mgr.h"
#include "crypto/crypto_softsha256.h"
#include "crypto/crypto_uecc.h"

// Earlier board.h files asked for USB this way.
#if !defined(DFU_TRANSPORT) && defined(DFU_BOOT_USB) && DFU_BOOT_USB
#define DFU_TRANSPORT		DFU_TRANSPORT_CDC
#endif

#ifndef DFU_PROTO
#define DFU_PROTO			DFU_PROTO_WIRE
#endif

#ifndef DFU_TRANSPORT
#define DFU_TRANSPORT		DFU_TRANSPORT_UART
#endif

#if DFU_TRANSPORT == DFU_TRANSPORT_CDC
#include "usb/usb.h"
#include "usb/usbd_cdc.h"
#elif DFU_TRANSPORT == DFU_TRANSPORT_UART
#include "coredev/uart.h"
#else
#error "Stage 0 recovery runs over DFU_TRANSPORT_UART or DFU_TRANSPORT_CDC"
#endif

#if DFU_PROTO == DFU_PROTO_WIRE
#include "dfu/dfu_wire.h"
#elif DFU_PROTO == DFU_PROTO_SMP
#include "dfu/dfu_smp.h"
#include "dfu/dfu_serial.h"
#else
#error "Stage 0 recovery speaks DFU_PROTO_WIRE or DFU_PROTO_SMP"
#endif

// From dfu_boot_key.c, as imgtool getpub writes it.
extern "C" const unsigned char ecdsa_pub_key[];
extern "C" const unsigned int ecdsa_pub_key_len;

#ifdef DFU_BOOT_ALLOW_UNSIGNED
#define DFU_BOOT_NO_REC		true
#else
#define DFU_BOOT_NO_REC		false
#endif

#ifndef DFU_BOOT_ALLOW_DOWNGRADE
#define DFU_BOOT_ALLOW_DOWNGRADE	0
#endif

#ifndef UART_DEVNO
#define UART_DEVNO			0
#endif

// Largest WRITE data (wire) or SMP request. A board.h for a part with little
// RAM may set it lower; hosts read it and send smaller pieces.
#ifndef DFU_BOOT_BUFSIZE
#define DFU_BOOT_BUFSIZE	512
#endif

// Link FIFO depth, each way.
#ifndef DFU_BOOT_FIFODEPTH
#define DFU_BOOT_FIFODEPTH	256
#endif

#define DFU_BOOT_FIFOSIZE	CFIFO_MEMSIZE(DFU_BOOT_FIFODEPTH)

// Worst case one erase unit takes, told to hosts for their timeouts.
#ifndef DFU_BOOT_ERASE_MS
#define DFU_BOOT_ERASE_MS	2000
#endif

#ifndef DFU_BOOT_VERSION
#define DFU_BOOT_VERSION	0x00010000
#endif

// The boot never returns, so nothing static is ever destroyed: no exit
// handlers to register, which also keeps the heap out of the image.
extern "C" int __cxa_atexit(void (*)(void *), void *, void *)
{
	return 0;
}

extern "C" int __aeabi_atexit(void *, void (*)(void *), void *)
{
	return 0;
}

extern "C" int atexit(void (*)(void))
{
	return 0;
}

// Nothing is ever deleted either, but a class with a virtual destructor
// names operator delete. Defined here with the rest of what the C++ runtime
// file of the library gives, so that file and the heap stay out.
void operator delete(void *) noexcept
{
}

void operator delete(void *, unsigned int) noexcept
{
}

extern "C" void __cxa_pure_virtual()
{
	for (;;)
	{
	}
}

// main never returns; the C start code still names exit.
extern "C" void exit(int)
{
	for (;;)
	{
	}
}

// The engines are built in place at run time: the digest and the signature
// check alone, nothing of HMAC, key generation or signing.
alignas(16) static uint8_t s_ShaMem[CRYPTO_SOFTSHA256_HASH_MEMSIZE];
alignas(16) static uint8_t s_EccMem[CRYPTO_UECC_VERIFY_MEMSIZE];

#if DFU_TRANSPORT == DFU_TRANSPORT_CDC

#ifndef USB_DEVNO
#define USB_DEVNO			0
#endif

#define DFU_BOOT_CDC_RXPKT	4
#define DFU_BOOT_CDC_RXMEM	USB_INTRF_RXMEM_SIZE(DFU_BOOT_CDC_RXPKT, \
								USB_CTRLR_PKT_LEN_MAX(USB_DEVNO, BULK))

alignas(4) static uint8_t s_CdcRxMem[DFU_BOOT_CDC_RXMEM];
alignas(4) static uint8_t s_CdcTxMem[DFU_BOOT_FIFOSIZE];

static const UsbdCdcCfg_t s_CdcCfg = {
	.DevNo = USB_DEVNO,
	.bBlocking = true,
	.RxFifoMemSize = DFU_BOOT_CDC_RXMEM,
	.pRxFifoMem = s_CdcRxMem,
	.TxFifoMemSize = DFU_BOOT_FIFOSIZE,
	.pTxFifoMem = s_CdcTxMem,
	.EvtCB = nullptr,
};

// pid.codes test id, as the USB examples use. A product puts its own
// vendor and product id here.
static const UsbCfg_t s_UsbCfg = {
	.DevNo = USB_DEVNO,
	.Mode = USB_MODE_DEVICE,
	.Vid = 0x1209,
	.Pid = 0x0001,
	.DevVer = 0x0100,
	.pManufacturer = "I-SYST",
	.pProduct = "IOsonata DFU",
	.pSerial = nullptr,
	.pFuncName = "IOsonata DFU",
	.IntPrio = 6,
	.DeviceClass = USB_DEVCLASS_MISC,
	.DeviceSubClass = 2U,
	.DeviceProtocol = 1U,
	.bSelfPowered = false,
	.bRemoteWakeup = false,
	.bLowPowerSuspend = false,
	.MaxPower = 100,
	.EvtHandler = nullptr,
};

static UsbdCdc s_Link;

static bool DfuBootLinkInit(void)
{
	if (UsbInit(&s_UsbCfg) == false || s_Link.Init(s_CdcCfg) == false)
	{
		return false;
	}

	// No cable yet is not an error: UsbProcess picks up the attach.
	UsbEnable(USB_DEVNO);

	return true;
}

static void DfuBootLinkRun(void)
{
	UsbProcess(USB_DEVNO);
}

#else

static IOPinCfg_t s_UartPins[] = UART_PINS;
static uint8_t s_UartRxFifo[DFU_BOOT_FIFOSIZE];
static uint8_t s_UartTxFifo[DFU_BOOT_FIFOSIZE];

static const UARTCfg_t s_UartCfg = {
	.DevNo = UART_DEVNO,
	.pIOPinMap = s_UartPins,
	.NbIOPins = sizeof(s_UartPins) / sizeof(IOPinCfg_t),
	.Rate = 115200,
	.DataBits = 8,
	.Parity = UART_PARITY_NONE,
	.StopBits = 1,
	.FlowControl = UART_FLWCTRL_NONE,
	.bIntMode = true,
	.IntPrio = 6,
	.EvtCallback = nullptr,
	.bFifoBlocking = true,
	.RxMemSize = DFU_BOOT_FIFOSIZE,
	.pRxMem = s_UartRxFifo,
	.TxMemSize = DFU_BOOT_FIFOSIZE,
	.pTxMem = s_UartTxFifo,
};

static UART s_Link;

static bool DfuBootLinkInit(void)
{
	return s_Link.Init(s_UartCfg);
}

static void DfuBootLinkRun(void)
{
}

#endif

static DfuStore_t s_Slot0;
static DfuMgr s_Mgr;

static bool DfuBootButton(void)
{
#if defined(DFU_BUT_PORT) && defined(DFU_BUT_PIN)
	IOPinConfig(DFU_BUT_PORT, DFU_BUT_PIN, 0, IOPINDIR_INPUT, IOPINRES_PULLUP,
				IOPINTYPE_NORMAL);
	for (int i = 0; i < 1000; i++)
	{
		__asm volatile("nop");
	}
	return IOPinRead(DFU_BUT_PORT, DFU_BUT_PIN) == 0;
#else
	return false;
#endif
}

// Time for the reset response to leave the FIFO: the frame is under 40
// bytes, 3.5 ms at 115200 baud.
#define DFU_BOOT_RESET_DELAY_MS		20

#if DFU_PROTO == DFU_PROTO_WIRE

static DfuWire s_Proto;
static uint8_t s_ProtoRx[DFU_WIRE_RXBUF_SIZE(DFU_BOOT_BUFSIZE)];

static void DfuBootReset(bool bRecovery)
{
	msDelay(DFU_BOOT_RESET_DELAY_MS);
	s_Mgr.Reset(bRecovery);
}

static bool DfuBootProtoInit(void)
{
	DfuWireCfg_t c = {
		.pMgr = &s_Mgr,
		.pIntrf = &s_Link,
		.pRxBuf = s_ProtoRx,
		.RxBufSize = sizeof(s_ProtoRx),
		.MaxBody = DFU_BOOT_BUFSIZE,
		.EraseMaxMs = DFU_BOOT_ERASE_MS,
		.BootVer = DFU_BOOT_VERSION,
		.pDevId = nullptr,
		.Reset = DfuBootReset,
	};

	return s_Proto.Init(c);
}

#else

static DfuSmp s_Smp;
static DfuSerial s_Proto;
static uint8_t s_SerRx[DFU_SERIAL_RXBUF_SIZE(DFU_BOOT_BUFSIZE)];
static uint8_t s_SerTx[DFU_SERIAL_TXBUF_SIZE];

static void DfuBootReset(void)
{
	msDelay(DFU_BOOT_RESET_DELAY_MS);
	DfuTgtReset();
}

static bool DfuBootProtoInit(void)
{
	DfuSmpCfg_t m = {
		.pMgr = &s_Mgr,
		.BufSize = DFU_BOOT_BUFSIZE,
		.ResetCB = DfuSerial::ResetCB,
		.pCtx = &s_Proto,
	};
	DfuSerialCfg_t c = {
		.pMgr = &s_Smp,
		.pIntrf = &s_Link,
		.pRxBuf = s_SerRx,
		.RxBufSize = sizeof(s_SerRx),
		.pTxBuf = s_SerTx,
		.TxBufSize = sizeof(s_SerTx),
		.Reset = DfuBootReset,
		.TxRetry = 0,
	};

	return s_Smp.Init(m) && s_Proto.Init(c);
}

#endif

int main()
{
	DfuImgKey_t key = { ecdsa_pub_key, ecdsa_pub_key_len };
	DfuBootCfg_t cfg = {
		.pHash = CryptoSoftSha256HashCreate(s_ShaMem, sizeof(s_ShaMem)),
		.pSign = CryptoUeccVerifyCreate(s_EccMem, sizeof(s_EccMem)),
		.pKey = &key,
		.NbKey = 1,
		.bVerifySlot0 = true,
		.bAllowNoRec = DFU_BOOT_NO_REC,
	};

	if (DfuBootButton() == false)
	{
		// Returns only when there is nothing to start, or the application
		// asked for recovery.
		(void)DfuBootRun(cfg);
	}

	// Recovery: straight into slot 0, the record last.
	DfuLayout_t lay;
	DfuMgrCfg_t mcfg = {
		.pStore = &s_Slot0,
		.bDirect = true,
		.bManifestOnly = DFU_PROTO == DFU_PROTO_WIRE,
		.bAllowDowngrade = DFU_BOOT_ALLOW_DOWNGRADE != 0,
		.pHash = cfg.pHash,
		.pBoot = &cfg,
	};

	if (DfuLayoutGet(&lay) &&
		DfuStoreTgt(&s_Slot0, lay.Slot0, lay.Slot0Size) &&
		s_Mgr.Init(mcfg) && DfuBootLinkInit() && DfuBootProtoInit())
	{
		for (;;)
		{
			DfuBootLinkRun();
			if (s_Proto.Poll() == false)
			{
				// The link interrupt wakes the core.
				__asm volatile("wfi");
			}
		}
	}

	for (;;)
	{
		__asm volatile("wfi");
	}
}
