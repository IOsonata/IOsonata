// RA4M1 failure diagnostics for I2CMasterSlave.
// Prints the IIC registers of both units, the PFS value of the four bus pins
// and the driver trace. Reads only, nothing is changed.
// Copyright (c) 2026, I-SYST inc. MIT License.
#include <stdio.h>
#include <stdint.h>

#include "ra4m1xxx.h"
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
	}
	if (cnt == 0U)
		printf("  no trace events\r\n");
}

static void DumpPin(const char *pName, int Port, int Pin)
{
	uint32_t pfs = R_PFS->PORT[Port].PIN[Pin].PmnPFS;

	// PSEL bits 28..24, PMR 16, NCODR 6, PCR 4, PDR 2
	printf("  %s P%d%02d PFS=%08lx PSEL=%lu PMR=%lu NCODR=%lu PCR=%lu PDR=%lu DSCR=%lu\r\n",
		pName, Port, Pin, (unsigned long)pfs, (unsigned long)((pfs >> 24) & 0x1FU),
		(unsigned long)((pfs >> 16) & 1U), (unsigned long)((pfs >> 6) & 1U),
		(unsigned long)((pfs >> 4) & 1U), (unsigned long)((pfs >> 2) & 1U),
		(unsigned long)((pfs >> 10) & 1U));
}

void I2cMasterSlaveDiagnostics(const char *pWhen)
{
	printf("--- I2C diagnostics %s ---\r\n", pWhen);
	DumpUnit("master", I2C_MASTER_DEVNO);
	DumpUnit("slave", I2C_SLAVE_DEVNO);
	DumpPin("master SDA", I2C_MASTER_SDA_PORT, I2C_MASTER_SDA_PIN);
	DumpPin("master SCL", I2C_MASTER_SCL_PORT, I2C_MASTER_SCL_PIN);
	DumpPin("slave SDA", I2C_SLAVE_SDA_PORT, I2C_SLAVE_SDA_PIN);
	DumpPin("slave SCL", I2C_SLAVE_SCL_PORT, I2C_SLAVE_SCL_PIN);
	printf("--- end ---\r\n");
}
