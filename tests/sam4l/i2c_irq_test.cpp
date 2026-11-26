// Host check of the SAM4L I2C master interrupt transfers.
//
// The production driver is compiled against register blocks in host memory.
// The test plays the TWIM: it sets the status flags, calls the interrupt
// handler and looks at what the driver wrote. IER, IDR and SCR are write only
// on the MCU; Sync() applies them to IMR and SR between two steps.
//
// Not hardware validation: bus timing and the PDCA are not modelled.
#include <cassert>
#include <cstdio>
#include <cstring>
#include "sam4lxxx.h"

unsigned g_PowerTestIrqEnabled, g_PowerTestIrqPending, g_PowerTestMask;
void PowerTestWfi(void) {}
static inline void NVIC_SetPriority(IRQn_Type, uint32_t) {}

static Twim s_Twim[4];
static Twis s_Twis[2];
static Pm s_Pm;
static Pdca s_Pdca;

#undef SAM4L_TWIM0
#undef SAM4L_TWIM1
#undef SAM4L_TWIM2
#undef SAM4L_TWIM3
#undef SAM4L_TWIS0
#undef SAM4L_TWIS1
#undef SAM4L_PM
#undef SAM4L_PDCA
#define SAM4L_TWIM0	(&s_Twim[0])
#define SAM4L_TWIM1	(&s_Twim[1])
#define SAM4L_TWIM2	(&s_Twim[2])
#define SAM4L_TWIM3	(&s_Twim[3])
#define SAM4L_TWIS0	(&s_Twis[0])
#define SAM4L_TWIS1	(&s_Twis[1])
#define SAM4L_PM	(&s_Pm)
#define SAM4L_PDCA	(&s_Pdca)

#include "../../ARM/Microchip/SAM4L/src/i2c_sam4l.cpp"

uint32_t SystemPeriphClockGet(int) { return 48000000U; }
void IOPinDisable(int, int) {}
void IOPinConfig(int, int, int, IOPINDIR, IOPINRES, IOPINTYPE) {}
void I2CBusReset(I2CDev_t * const) {}

static int s_Completed, s_TxReady, s_Count;

static int Event(DevIntrf_t *, DEVINTRF_EVT Evt, uint8_t *, int Len)
{
	if (Evt == DEVINTRF_EVT_COMPLETED)
	{
		s_Completed++;
		s_Count = Len;
	}
	if (Evt == DEVINTRF_EVT_TX_READY)
		s_TxReady++;
	return 0;
}

static I2CDev_t s_I2c;
static Twim &R = s_Twim[0];
static const IOPinCfg_t s_Pins[2] = {};

// Apply the write only registers the driver used since the last step
static void Sync(void)
{
	R.TWIM_IMR = (R.TWIM_IMR | R.TWIM_IER) & ~R.TWIM_IDR;
	R.TWIM_IER = R.TWIM_IDR = 0;
	R.TWIM_SR &= ~R.TWIM_SCR;
	R.TWIM_SCR = 0;
}

static void Init(bool b10Bit)
{
	I2CCfg_t cfg = {};
	cfg.DevNo = 0;
	cfg.Type = I2CTYPE_STANDARD;
	cfg.Mode = I2CMODE_MASTER;
	cfg.pIOPinMap = s_Pins;
	cfg.NbIOPins = 2;
	cfg.Rate = 100000;
	cfg.MaxRetry = 0;
	cfg.AddrType = b10Bit ? I2CADDR_TYPE_EXT : I2CADDR_TYPE_NORMAL;
	cfg.bDmaEn = false;
	cfg.bIntEn = true;
	cfg.EvtCB = Event;
	memset(s_Twim, 0, sizeof(s_Twim));
	assert(I2CInit(&s_I2c, &cfg));
	// The reset in the initialisation masked every interrupt
	Sync();
	s_Completed = s_TxReady = s_Count = 0;
}

static void Irq(uint32_t Status)
{
	Sync();
	R.TWIM_SR = Status;
	TWIM0_Handler();
	Sync();
}

static bool Busy(void)
{
	bool b = atomic_flag_test_and_set(&s_I2c.DevIntrf.bBusy);
	if (!b)
		atomic_flag_clear(&s_I2c.DevIntrf.bBusy);
	return b;
}

static uint32_t NBytes(void)
{
	return (R.TWIM_CMDR & TWIM_CMDR_NBYTES_Msk) >> TWIM_CMDR_NBYTES_Pos;
}

static uint8_t s_Buf[700];

// Transmit of Len bytes, sent from the caller's buffer command after command
static void TestTx(int Len)
{
	Init(false);
	for (int i = 0; i < Len; i++)
		s_Buf[i] = (uint8_t)(i * 7 + 1);
	assert(DeviceIntrfTx(&s_I2c.DevIntrf, 0x22, s_Buf, Len) == -1);

	int done = 0;
	while (done < Len)
	{
		const int chunk = Len - done > 255 ? 255 : Len - done;
		const bool last = done + chunk == Len;

		Sync();
		assert((R.TWIM_CMDR & TWIM_CMDR_VALID) && !(R.TWIM_CMDR & TWIM_CMDR_READ));
		assert((int)NBytes() == chunk);
		assert(((R.TWIM_CMDR & TWIM_CMDR_START) != 0) == (done == 0));
		assert(((R.TWIM_CMDR & TWIM_CMDR_STOP) != 0) == last);
		assert(R.TWIM_IMR & TWIM_IMR_TXRDY);
		for (int i = 0; i < chunk; i++)
		{
			assert(s_Completed == 0 && Busy());
			Irq(TWIM_SR_TXRDY);
			assert(R.TWIM_THR == s_Buf[done + i]);
		}
		// The byte interrupt is off once the command has all its bytes
		assert(!(R.TWIM_IMR & TWIM_IMR_TXRDY));
		assert(s_Completed == 0 && s_TxReady == 0);
		R.TWIM_CMDR = 0;
		Irq(TWIM_SR_CCOMP | TWIM_SR_TXRDY);
		done += chunk;
	}
	assert(s_TxReady == 1 && s_Completed == 1 && s_Count == Len);
	assert(!Busy() && s_I2c.DevIntrf.bTxReady && R.TWIM_IMR == 0);
}

// Receive of Len bytes into the caller's buffer
static void TestRx(int Len, bool b10Bit)
{
	Init(b10Bit);
	memset(s_Buf, 0, sizeof(s_Buf));
	assert(DeviceIntrfRx(&s_I2c.DevIntrf, b10Bit ? 0x322 : 0x22, s_Buf, Len) == -1);

	if (b10Bit)
	{
		// Address command first: write direction, no data
		Sync();
		assert(!(R.TWIM_CMDR & TWIM_CMDR_READ) && NBytes() == 0);
		assert(R.TWIM_CMDR & TWIM_CMDR_TENBIT);
		assert(s_Completed == 0);
		Irq(TWIM_SR_CCOMP);
		assert(s_Completed == 0 && Busy());
		assert(R.TWIM_CMDR & TWIM_CMDR_REPSAME);
	}

	int done = 0;
	while (done < Len)
	{
		const int chunk = Len - done > 255 ? 255 : Len - done;
		const bool last = done + chunk == Len;

		Sync();
		assert((R.TWIM_CMDR & TWIM_CMDR_VALID) && (R.TWIM_CMDR & TWIM_CMDR_READ));
		assert((int)NBytes() == chunk);
		assert(((R.TWIM_CMDR & TWIM_CMDR_STOP) != 0) == last);
		assert(((R.TWIM_CMDR & TWIM_CMDR_ACKLAST) != 0) == !last);
		for (int i = 0; i < chunk; i++)
		{
			R.TWIM_RHR = (uint8_t)(done + i + 3);
			assert(s_Completed == 0 && Busy());
			Irq(TWIM_SR_RXRDY);
		}
		Irq(TWIM_SR_CCOMP);
		done += chunk;
	}
	assert(s_Completed == 1 && s_TxReady == 0 && s_Count == Len && !Busy());
	for (int i = 0; i < Len; i++)
		assert(s_Buf[i] == (uint8_t)(i + 3));
}

// Address not acknowledged, then the bus is used again
static void TestError(void)
{
	Init(false);
	s_Buf[0] = 1;
	assert(DeviceIntrfTx(&s_I2c.DevIntrf, 0x22, s_Buf, 4) == -1);
	Irq(TWIM_SR_ANAK);
	assert(s_Completed == 1 && s_Count == 0 && !Busy());
	R.TWIM_SR = 0;
	assert(DeviceIntrfTx(&s_I2c.DevIntrf, 0x22, s_Buf, 1) == -1);
	Irq(TWIM_SR_TXRDY);
	Irq(TWIM_SR_CCOMP);
	assert(s_Completed == 2 && s_Count == 1 && !Busy());

	Init(false);
	assert(DeviceIntrfRx(&s_I2c.DevIntrf, 0x22, s_Buf, 4) == -1);
	R.TWIM_RHR = 9;
	Irq(TWIM_SR_RXRDY);
	Irq(TWIM_SR_DNAK);
	assert(s_Completed == 1 && s_Count == 1 && !Busy());
}

// Register read: the command bytes are taken from the caller's pointer when
// the receive is programmed. This path is polled, the flags stay set.
static void TestRead(void)
{
	Init(false);
	uint8_t cmd[2] = { 0xA7, 0x5C };
	R.TWIM_SR = TWIM_SR_TXRDY | TWIM_SR_RXRDY | TWIM_SR_CCOMP | TWIM_SR_IDLE;
	R.TWIM_RHR = 0x42;
	int n = DeviceIntrfRead(&s_I2c.DevIntrf, 0x22, cmd, 2, s_Buf, 5);
	assert(n == 5 && R.TWIM_THR == 0x5C && s_Buf[4] == 0x42);
	assert((R.TWIM_NCMDR & TWIM_NCMDR_READ) && !Busy());
}

int main(void)
{
	TestTx(1);
	TestTx(3);
	TestTx(255);
	TestTx(300);
	TestTx(600);
	TestRx(1, false);
	TestRx(4, false);
	TestRx(300, false);
	TestRx(4, true);
	TestRx(300, true);
	TestError();
	TestRead();
	printf("SAM4L I2C master interrupt transfers: PASS\n");
	return 0;
}
