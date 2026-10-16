// RA4M1 failure diagnostics for I2CMasterSlave.
// Prints the IIC registers of both units, the PFS value of the four bus pins
// and the driver trace. Runs a GPIO SCL isolation test after failure.
// Copyright (c) 2026, I-SYST inc. MIT License.
#include <stdio.h>
#include <stdint.h>

#include "ra4m1xxx.h"
#include "coredev/system_core_clock.h"
#include "iopinctrl.h"
#include "i2c_ra4m1.h"
#include "board.h"

static const char *TraceName(uint8_t Code)
{
	switch (Code)
	{
		case RA4M1_I2C_TR_CONFIG: return "config done";
		case RA4M1_I2C_TR_START_REFUSED: return "StartTx/Rx refused";
		case RA4M1_I2C_TR_IDLE_BUSY: return "bus busy, reset next";
		case RA4M1_I2C_TR_IDLE_FAIL: return "bus busy after reset";
		case RA4M1_I2C_TR_TX_REFUSED: return "TxData refused";
		case RA4M1_I2C_TR_RX_REFUSED: return "RxData refused";
		case RA4M1_I2C_TR_BEGIN: return "START requested";
		case RA4M1_I2C_TR_NACK: return "NACK";
		case RA4M1_I2C_TR_ABORT: return "abort";
		case RA4M1_I2C_TR_ADDR: return "address written";
		case RA4M1_I2C_TR_SAMPLE: return "stuck sample";
		case RA4M1_I2C_TR_START_POST: return "START register set";
		default: return "?";
	}
}

static void DumpUnit(const char *pName, int DevNo)
{
	const R_IIC0_Type *reg = DevNo == 0 ? R_IIC0 : R_IIC1;
	Ra4m1I2cTrace_t rec;
	uint32_t cnt = Ra4m1I2cTraceCount(DevNo);

	// MSTPCRB bit 9 is IIC0 and bit 8 is IIC1, a stopped unit is not read
	if (R_MSTP->MSTPCRB & (1UL << (DevNo == 0 ? 9 : 8)))
	{
		printf("%s IIC%d: module stopped\r\n", pName, DevNo);
		return;
	}

	// ICSR2 is read once here, a read does not clear its flags
	printf("%s IIC%d: ICCR1=%02x ICCR2=%02x ICSR2=%02x ICFER=%02x ICMR1=%02x ICMR2=%02x ICMR3=%02x ICIER=%02x ICSER=%02x SAR0=%02x ICBR=%02x/%02x\r\n",
		pName, DevNo, reg->ICCR1, reg->ICCR2, reg->ICSR2, reg->ICFER, reg->ICMR1,
		reg->ICMR2, reg->ICMR3, reg->ICIER, reg->ICSER, reg->SAR[0].L, reg->ICBRL, reg->ICBRH);
	printf("  ICCR1 SDAI=%d SCLI=%d ICE=%d IICRST=%d, ICCR2 BBSY=%d\r\n",
		reg->ICCR1 & 1U, (reg->ICCR1 >> 1) & 1U, (reg->ICCR1 >> 7) & 1U,
		(reg->ICCR1 >> 6) & 1U, (reg->ICCR2 >> 7) & 1U);

	for (uint32_t i = 0; Ra4m1I2cTraceGet(DevNo, i, &rec); ++i)
	{
		printf("  #%lu %s info=%02x mstate=%u CR1=%02x CR2=%02x SR2=%02x FER=%02x MR3=%02x IER=%02x SER=%02x peer CR1=%02x SR1=%02x SR2=%02x IER=%02x\r\n",
			(unsigned long)rec.Seq, TraceName(rec.Code), rec.Info, rec.MState, rec.ICCR1,
			rec.ICCR2, rec.ICSR2, rec.ICFER, rec.ICMR3, rec.ICIER, rec.ICSER,
			rec.PeerCR1, rec.PeerSR1, rec.PeerSR2, rec.PeerIER);

		if (rec.Code == RA4M1_I2C_TR_ABORT ||
			rec.Code == RA4M1_I2C_TR_BEGIN ||
			rec.Code == RA4M1_I2C_TR_START_POST)
		{
			printf("    PFS master SDA=%08lx SCL=%08lx slave SDA=%08lx SCL=%08lx\r\n",
				(unsigned long)rec.SdaPfs, (unsigned long)rec.SclPfs,
				(unsigned long)rec.PeerSdaPfs, (unsigned long)rec.PeerSclPfs);
		}
	}
	if (cnt == 0U)
		printf("  no trace events\r\n");
}

// Toggle SCL as open-drain GPIO after IIC has been disabled.
// The slow transitions are visible with a single oscilloscope probe.
static void GpioSclDelay(void)
{
	volatile uint32_t count = SystemCoreClockGet() / 25U;

	while (count > 0U)
	{
		__NOP();
		count--;
	}
}

static void GpioSclTest(int PortNo, int PinNo)
{
	IOPinSet(PortNo, PinNo);
	IOPinConfig(PortNo, PinNo, IOPINOP_GPIO, IOPINDIR_OUTPUT,
		IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN);

	printf("  GPIO SCL start: PFS=%08lx input=%d\r\n",
		(unsigned long)R_PFS->PORT[PortNo].PIN[PinNo].PmnPFS,
		IOPinRead(PortNo, PinNo));

	for (int i = 0; i < 8; i++)
	{
		IOPinClear(PortNo, PinNo);
		GpioSclDelay();
		IOPinSet(PortNo, PinNo);
		GpioSclDelay();
	}

	printf("  GPIO SCL finish: PFS=%08lx input=%d\r\n",
		(unsigned long)R_PFS->PORT[PortNo].PIN[PinNo].PmnPFS,
		IOPinRead(PortNo, PinNo));

	// Isolated P400 test only: exercise push-pull HIGH/LOW after open-drain.
	// The SCL jumper must remain disconnected during this diagnostic.
	IOPinSet(PortNo, PinNo);
	IOPinConfig(PortNo, PinNo, IOPINOP_GPIO, IOPINDIR_OUTPUT,
		IOPINRES_NONE, IOPINTYPE_NORMAL);
	GpioSclDelay();
	printf("  GPIO push-pull HIGH: PFS=%08lx input=%d\r\n",
		(unsigned long)R_PFS->PORT[PortNo].PIN[PinNo].PmnPFS,
		IOPinRead(PortNo, PinNo));

	IOPinClear(PortNo, PinNo);
	GpioSclDelay();
	printf("  GPIO push-pull LOW:  PFS=%08lx input=%d\r\n",
		(unsigned long)R_PFS->PORT[PortNo].PIN[PinNo].PmnPFS,
		IOPinRead(PortNo, PinNo));

	IOPinSet(PortNo, PinNo);
	GpioSclDelay();
	printf("  GPIO push-pull HIGH again: PFS=%08lx input=%d\r\n",
		(unsigned long)R_PFS->PORT[PortNo].PIN[PinNo].PmnPFS,
		IOPinRead(PortNo, PinNo));
}

void I2cMasterSlaveDiagnostics(const char *pWhen)
{
	printf("--- I2C diagnostics %s ---\r\n", pWhen);
	printf("  MSTPCRB=%08lx SCKSCR=%02x SCKDIVCR=%08lx OPCCR=%02x SOPCCR=%02x\r\n",
		(unsigned long)R_MSTP->MSTPCRB, R_SYSTEM->SCKSCR,
		(unsigned long)R_SYSTEM->SCKDIVCR, R_SYSTEM->OPCCR, R_SYSTEM->SOPCCR);
	printf("  ICLK=%lu PCLKB=%lu Hz\r\n",
		(unsigned long)SystemCoreClockGet(),
		(unsigned long)SystemPeriphClockGet(1));
	DumpUnit("master", I2C_MASTER_DEVNO);
	DumpUnit("slave", I2C_SLAVE_DEVNO);

	// After all transfers have failed, disable the master peripheral.
	// Hold this state in the application's final loop for a single-probe
	// measurement. This is diagnostic-only and does not alter bus wiring.
	R_IIC0_Type *master = I2C_MASTER_DEVNO == 0 ? R_IIC0 : R_IIC1;
	const int port = I2C_MASTER_SCL_PORT;
	const int pin = I2C_MASTER_SCL_PIN;

	printf("  ICE test before: CR1=%02x CR2=%02x SR2=%02x SCLPFS=%08lx\r\n",
		master->ICCR1, master->ICCR2, master->ICSR2,
		(unsigned long)R_PFS->PORT[port].PIN[pin].PmnPFS);
	master->ICIER = 0U;
	master->ICCR1 &= (uint8_t)~0x80U;
	printf("  ICE test after:  CR1=%02x CR2=%02x SR2=%02x SCLPFS=%08lx\r\n",
		master->ICCR1, master->ICCR2, master->ICSR2,
		(unsigned long)R_PFS->PORT[port].PIN[pin].PmnPFS);
	GpioSclTest(port, pin);
	printf("--- end ---\r\n");
}
