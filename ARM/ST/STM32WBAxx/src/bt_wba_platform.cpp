/**-------------------------------------------------------------------------
@file	bt_wba_platform.cpp

@brief	STM32WBA BLE stack platform layer on IOsonata.

The STM32CubeWBA BLE host stack and its link layer library call a small set of
platform functions. A Cube project supplies them with its HAL, the sequencer,
the timer server, the RTC, the memory manager and the flash manager. This file
supplies them with IOsonata code instead, so the Cube project files that remain
in a build are bleplat.c, linklayer_plat.c, power_table.c, and the WPAN
ll_sys_startup.c, ll_sys_intf.c, ll_sys_cs.c, ll_sys_dp_slp.c:

- HAL_NVIC_* and the radio baseband clock: direct NVIC and RCC access.
- Sequencer (UTIL_SEQ_RegTask, UTIL_SEQ_SetTask): the tasks are run from the
  Bluetooth event queue, BtEvtQue.
- Link layer system hooks (ll_sys_config_params, ll_sys_reset, the background
  process scheduling and the temperature request), in place of the Cube
  ll_sys_if.c.
- BLE_TIMER_Start/Stop: fixed host timer slots over the ST controller timer service.
- HW_RNG_Get and the RNG clock requests: RNG peripheral, polled.
- BAES_*: the STM32WBA AES block through CryptoWbaAes (ECB, CMAC and CCM).
  A call that arrives while another one holds the block, an interrupt that
  preempts a task for example, runs on the CryptoSoftAes engine instead.
- The P-256 operations are in bt_wba_pka.cpp.
- Link layer interrupts (RADIO and HASH vectors), HSE32 wait, system clock
  manager entries, debug signals: small functions.
- Radio calibration temperature: TempSTM32WBA (temp_stm32wba.cpp) reads the
  internal temperature sensor through ADC4. If the first reading fails, the
  link layer keeps its periodic calibration.

Known limits:
- The AES block is polled. The secure AES (SAES) is not used.

@author	Hoang Nguyen Hoan
@date	Oct. 14, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is furnished
to do so, subject to the following conditions:

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
#include <string.h>

#include "stm32wbaxx.h"

#include "app_conf.h"
#include "bt_wba_stack.h"

#include "istddef.h"
#include "crypto/crypto_softaes.h"
#include "crypto/crypto_stm32wba.h"
#include "bluetooth/bt_app.h"

#include "coredev/system_core_clock.h"
#include "bt_wba_platform.h"
#include "syslog.h"
#include "temp_stm32wba.h"

// Defined by the port in bt_app_stm32wba.cpp: runs the host stack
extern "C" void BleStackCB_Process(void);

// Offset the link layer adds to the temperature (TEMPMEAS_MIN_TEMP_LIMIT)
#define BT_WBA_TEMPERATURE_OFS	40

#define BT_WBA_TIMER_SLOTS		8U

// Busy loop limits, in loop passes. They only stop a hang when the hardware
// does not answer.
#define BT_WBA_HSE_POLL_MAX		2000000UL
#define BT_WBA_RNG_POLL_MAX		200000UL

#define BT_WBA_RNG_RETRY		4

static inline uint32_t BtWbaIrqSave(void)
{
	uint32_t pm = __get_PRIMASK();

	__disable_irq();

	return pm;
}

static inline void BtWbaIrqRestore(uint32_t Pm)
{
	__set_PRIMASK(Pm);
}

//----------------------------------------------------------------------------
// Interrupt controller and radio clock
//----------------------------------------------------------------------------

extern "C" __attribute__((weak)) void HAL_NVIC_SetPriority(IRQn_Type IRQn,
														   uint32_t PreemptPriority,
														   uint32_t SubPriority)
{
	(void)SubPriority;

	NVIC_SetPriority(IRQn, PreemptPriority);
}

extern "C" __attribute__((weak)) void HAL_NVIC_EnableIRQ(IRQn_Type IRQn)
{
	NVIC_EnableIRQ(IRQn);
}

extern "C" __attribute__((weak)) void HAL_NVIC_DisableIRQ(IRQn_Type IRQn)
{
	NVIC_DisableIRQ(IRQn);
}

extern "C" __attribute__((weak)) void HAL_NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
	NVIC_SetPendingIRQ(IRQn);
}

extern "C" __attribute__((weak)) void HAL_RCCEx_EnableRadioBBClock(void)
{
	SET_BIT(RCC->RADIOENR, RCC_RADIOENR_BBCLKEN);
}

extern "C" __attribute__((weak)) void HAL_RCCEx_DisableRadioBBClock(void)
{
	CLEAR_BIT(RCC->RADIOENR, RCC_RADIOENR_BBCLKEN);
}

// Called by the link layer when something fails that
// cannot be recovered.
extern "C" __attribute__((weak)) void Error_Handler(void)
{
	__disable_irq();
	while (1)
	{
	}
}

// Link layer debug pins are not used.
extern "C" void LINKLAYER_DEBUG_SIGNAL_SET(uint32_t Signal)
{
	(void)Signal;
}

extern "C" void LINKLAYER_DEBUG_SIGNAL_RESET(uint32_t Signal)
{
	(void)Signal;
}

extern "C" void LINKLAYER_DEBUG_SIGNAL_TOGGLE(uint32_t Signal)
{
	(void)Signal;
}

//----------------------------------------------------------------------------
// System clock manager entries. The clock is set by SystemInit, see
// system_stm32wbaxx.c, so only the HSE32 wait does work.
//----------------------------------------------------------------------------

extern "C" void SCM_HSE_WaitUntilReady(void)
{
	uint32_t n = BT_WBA_HSE_POLL_MAX;

	SET_BIT(RCC->CR, RCC_CR_HSEON);
	while ((RCC->CR & RCC_CR_HSERDY) == 0U && n > 0U)
	{
		n--;
	}
}

// CubeWBA otp.c searches 16-byte records backwards for index zero.
// Byte 14 is HSE load-capacitance trim; byte 15 is the record index.
static uint32_t BtWbaHseTrimGet(void)
{
	const volatile uint8_t *otp = (const volatile uint8_t *)FLASH_OTP_BASE;
	size_t offset = FLASH_OTP_SIZE;

	while (offset >= 16U)
	{
		offset -= 16U;
		if (otp[offset + 15U] == 0)
		{
			return otp[offset + 14U] <= 0x3FU ? otp[offset + 14U] : 0x0CU;
		}
	}
	return 0x0CU;
}

/**
 * @brief	Start HSE32 and select the radio sleep timer clock.
 *
 * The link layer reads the sleep timer clock source and stops when it is not
 * set. LSE is used when the project asks for a crystal low power oscillator
 * and it starts,
 * otherwise LSI1 is used. LSI1 is less accurate, which limits connections.
 */
extern "C" void BtWbaPlatformInit(void)
{
	uint32_t n;
	uint32_t src;
	uint32_t ready = RCC_BDCR1_LSERDY | RCC_BDCR1_LSESYSRDY;
	bool bLse = false;

	MODIFY_REG(RCC->ECSCR1, RCC_ECSCR1_HSETRIM,
		BtWbaHseTrimGet() << RCC_ECSCR1_HSETRIM_Pos);
	SCM_HSE_WaitUntilReady();

	SET_BIT(PWR->DBPR, PWR_DBPR_DBP);

	if (g_McuOsc.LowPwrOsc.Type != OSC_TYPE_RC)
	{
		if ((RCC->BDCR1 & RCC_BDCR1_LSEON) == 0U)
		{
			// Match Cube WBA crystal startup. Preserve an already running LSE.
			MODIFY_REG(RCC->BDCR1, RCC_BDCR1_LSEDRV, RCC_BDCR1_LSEDRV_0);
			if (g_McuOsc.LowPwrOsc.Type == OSC_TYPE_TCXO)
			{
				SET_BIT(RCC->BDCR1, RCC_BDCR1_LSEBYP);
			}
			else
			{
				CLEAR_BIT(RCC->BDCR1, RCC_BDCR1_LSEBYP);
			}
		}
		// LSE starts much more slowly than HSE. Use the LPTIM driver's
		// clock-scaled allowance (at least 2 s at four cycles per poll).
		n = SystemCoreClock / 2U;
		// Cube RCC_LSE_ON enables both the oscillator and its system output.
		// Complete both before entering BleStack_Init.
		SET_BIT(RCC->BDCR1, RCC_BDCR1_LSEON | RCC_BDCR1_LSESYSEN);
		while ((RCC->BDCR1 & ready) != ready && n > 0U)
		{
			n--;
		}
		bLse = (RCC->BDCR1 & ready) == ready;
		if (bLse == false)
		{
			// No crystal answered, use the internal oscillator.
			CLEAR_BIT(RCC->BDCR1, RCC_BDCR1_LSEON | RCC_BDCR1_LSESYSEN);
		}
	}

	if (bLse)
	{
		src = RCC_BDCR1_RADIOSTSEL_0;
	}
	else
	{
		n = BT_WBA_HSE_POLL_MAX;
		SET_BIT(RCC->BDCR1, RCC_BDCR1_LSI1ON);
		while ((RCC->BDCR1 & RCC_BDCR1_LSI1RDY) == 0U && n > 0U)
		{
			n--;
		}
		src = RCC_BDCR1_RADIOSTSEL_1;
	}

	MODIFY_REG(RCC->BDCR1, RCC_BDCR1_RADIOSTSEL, src);
	SysLogPrintf(SysLogGet(), "WBA radio sleep clock: %s\r\n",
				 bLse ? "LSE 32768 Hz" : "LSI1");
	SysLogPrintf(SysLogGet(), "WBA core clock: %lu Hz; ICACHE: %s\r\n",
				 (unsigned long)SystemCoreClock,
				 (ICACHE->CR & ICACHE_CR_EN) != 0U ? "enabled" : "disabled");
}

extern "C" void scm_notifyradiostate(uint32_t State)
{
	(void)State;
}

extern "C" void scm_setsystemclock(uint32_t User, uint32_t Clock)
{
	(void)User;
	(void)Clock;
}

//----------------------------------------------------------------------------
// Sequencer: the stack and the link layer register tasks and set them. The
// tasks run from the Bluetooth event queue.
//----------------------------------------------------------------------------

static void (*s_SeqTask[32])(void);
static volatile uint32_t s_SeqPend;
static volatile bool s_SeqQueued;
static volatile bool s_SeqOwed;

static void BtWbaSeqEvt(uint32_t Evt, void *pCtx);

// Mark tasks pending and queue one run for all of them. Can be called from
// interrupt context, the link layer sets its background task from its ISR.
static void BtWbaSeqSet(uint32_t TaskBm)
{
	uint32_t pm = BtWbaIrqSave();
	bool bQue = s_SeqQueued == false;

	s_SeqPend |= TaskBm;
	s_SeqQueued = true;
	BtWbaIrqRestore(pm);

	if (bQue)
	{
		if (BtEvtQue(0, nullptr, BtWbaSeqEvt) == false)
		{
			s_SeqQueued = false;
			s_SeqOwed = true;
		}
	}
}

static void BtWbaSeqQue(void)
{
	BtWbaSeqSet(0);
}

static void BtWbaSeqEvt(uint32_t Evt, void *pCtx)
{
	uint32_t pm;
	uint32_t pend;
	uint32_t i;

	(void)Evt;
	(void)pCtx;

	pm = BtWbaIrqSave();
	pend = s_SeqPend;
	s_SeqPend = 0;
	s_SeqQueued = false;
	s_SeqOwed = false;
	BtWbaIrqRestore(pm);

	// Run one snapshot. A task can request another pass, but that pass must
	// yield to queued host work and application notification processing.
	for (i = 0; i < 32U; i++)
	{
		if ((pend & (1UL << i)) != 0U && s_SeqTask[i] != nullptr)
		{
			s_SeqTask[i]();
		}
	}
}

extern "C" void UTIL_SEQ_RegTask(uint32_t TaskBm, uint32_t Flags, void (*Task)(void))
{
	(void)Flags;

	for (uint32_t i = 0; i < 32U; i++)
	{
		if ((TaskBm & (1UL << i)) != 0U)
		{
			s_SeqTask[i] = Task;
		}
	}
}

extern "C" void UTIL_SEQ_SetTask(uint32_t TaskBm, uint32_t Prio)
{
	(void)Prio;

	BtWbaSeqSet(TaskBm);
}

//----------------------------------------------------------------------------
// Link layer system hooks, in place of the Cube ll_sys_if.c. They call the
// link layer configuration functions; the only register read is the radio
// sleep timer clock selection in RCC_BDCR1.
// - ll_sys_startup.c calls ll_sys_bg_process_init and ll_sys_config_params
//   when the link layer starts.
// - The link layer calls ll_sys_schedule_bg_process and its _isr version to
//   run ll_sys_bg_process.
// - The host stack calls ll_sys_reset on an HCI reset (le_ctrl.o).
// - linklayer_plat.c calls ll_sys_bg_temperature_measurement.
//----------------------------------------------------------------------------

// Link layer scheduler margins (ll_sys_config_BLE_schldr_timings units), Cube
// values. The link layer starts with the defaults.
#define BT_WBA_LL_DRIFT_TIME			14U
#define BT_WBA_LL_EXEC_TIME				28U
#define BT_WBA_LL_DRIFT_TIME_LSI		9U
#define BT_WBA_LL_EXEC_TIME_LSI			3U
#define BT_WBA_LL_DRIFT_TIME_DEBUG		6U
#define BT_WBA_LL_EXEC_TIME_DEBUG		4U

// LSI1 calibration: event length in sleep timer cycles, period in ms
#define BT_WBA_LL_RCO_CLBR_CYCLES		24U
#define BT_WBA_LL_RCO_CLBR_PERIOD_MS	15000U

// Give the link layer the sleep timer clock that BtWbaPlatformInit selected,
// and return the RCC_BDCR1 RADIOSTSEL field. The table is indexed by that
// field: none, LSE, LSI, HSE / 1000. With none, 0 is passed, as Cube does,
// which BtWbaPlatformInit never leaves.
static uint32_t BtWbaSleepClockSelect(void)
{
	static const uint8_t s_SlpTmrSrc[4] = {
		0, RTC_SLPTMR, RCO_SLPTMR, CRYSTAL_OSCILLATOR_SLPTMR
	};
	uint32_t sel = RCC->BDCR1 & RCC_BDCR1_RADIOSTSEL;
	uint16_t freq;

	ll_intf_cmn_le_select_slp_clk_src(s_SlpTmrSrc[sel >> RCC_BDCR1_RADIOSTSEL_Pos], &freq);

	return sel;
}

// Initial measurement is taken before declaring the sensor to the LL.
static TempSTM32WBA s_TemperatureSensor;
static int32_t s_Temperature;
static bool s_TemperatureInitial;

static void BtWbaTemperatureReport(void)
{
	TempSensorData_t data;

	if (s_TemperatureSensor.UpdateData())
	{
		s_TemperatureSensor.Read(data);
		ll_intf_cmn_set_temperature_value((uint32_t)(data.Temperature / 100 + BT_WBA_TEMPERATURE_OFS));
	}
}

extern "C" void ll_sys_bg_process_init(void)
{
	s_SeqTask[CFG_TASK_LINK_LAYER] = ll_sys_bg_process;
}

extern "C" void ll_sys_schedule_bg_process(void)
{
	BtWbaSeqSet(1UL << CFG_TASK_LINK_LAYER);
}

extern "C" void ll_sys_schedule_bg_process_isr(void)
{
	BtWbaSeqSet(1UL << CFG_TASK_LINK_LAYER);
}

extern "C" void ll_sys_config_params(void)
{
	static const TempSensorCfg_t cfg = {
		0, SENSOR_OPMODE_SINGLE, 0, 0, 0, DEVINTR_POL_LOW, 0, 0, nullptr
	};
	TempSensorData_t data;

	// Radio low ISR used, next event scheduled from the ISR. The link layer
	// is built for these two settings.
	ll_intf_cmn_config_ll_ctx_params(1, 1);
	BtWbaSleepClockSelect();
#if (USE_TEMPERATURE_BASED_RADIO_CALIBRATION == 1)
	// Recalibrate on temperature reports in place of the periodic calibration
	// every 10 s. The first temperature request comes from this call.
	s_SeqTask[CFG_TASK_TEMP_MEAS] = BtWbaTemperatureReport;
	s_TemperatureInitial = s_TemperatureSensor.Init(cfg) && s_TemperatureSensor.UpdateData();
	if (s_TemperatureInitial)
	{
		s_TemperatureSensor.Read(data);
		s_Temperature = data.Temperature;
		ll_intf_cmn_set_temperature_sensor_state();
	}
	// Without a valid initial measurement leave periodic calibration active.
#endif
	ll_intf_cmn_select_tx_power_table(CFG_RF_TX_POWER_TABLE_ID);
}

extern "C" void ll_sys_reset(void)
{
	uint32_t sel = BtWbaSleepClockSelect();
	uint8_t drift = BT_WBA_LL_DRIFT_TIME;
	uint8_t exec = BT_WBA_LL_EXEC_TIME;

	// Sleep clock accuracy 0 (251 to 500 ppm): the Cube value for every WBA5
	// revision and WBA6, with any sleep timer clock
	ll_intf_le_set_sleep_clock_accuracy(0);

	if (sel == RCC_BDCR1_RADIOSTSEL_1)
	{
		// LSI1
		ll_intf_le_set_rco_clbr_evnt_params(BT_WBA_LL_RCO_CLBR_CYCLES,
											BT_WBA_LL_RCO_CLBR_PERIOD_MS);
		drift += BT_WBA_LL_DRIFT_TIME_LSI;
		exec += BT_WBA_LL_EXEC_TIME_LSI;
	}
#ifdef DEBUG
	else
	{
		// Unoptimized platform code needs a larger margin
		drift += BT_WBA_LL_DRIFT_TIME_DEBUG;
		exec += BT_WBA_LL_EXEC_TIME_DEBUG;
	}
#endif

	if (drift != BT_WBA_LL_DRIFT_TIME || exec != BT_WBA_LL_EXEC_TIME)
	{
		ll_sys_config_BLE_schldr_timings(drift, exec);
	}
}

// Link layer temperature request. The first one comes from
// ll_intf_cmn_set_temperature_sensor_state in ll_sys_config_params and is
// answered at once. The next ones come from the event scheduler and are
// answered from the event queue, as the Cube code does.
extern "C" void ll_sys_bg_temperature_measurement(void)
{
	if (s_TemperatureInitial)
	{
		s_TemperatureInitial = false;
		ll_intf_cmn_set_temperature_value((uint32_t)(s_Temperature / 100 + BT_WBA_TEMPERATURE_OFS));
	}
	else
	{
		BtWbaSeqSet(1UL << CFG_TASK_TEMP_MEAS);
	}
}

//----------------------------------------------------------------------------
// BLE host deadlines and the application tick use ST controller software timers.
// os_timer_create reserves from the controller's fixed pool, once per adapter.
// Callbacks only queue work; host expiry processing stays outside the LL ISR.
//----------------------------------------------------------------------------

enum {
	BT_WBA_TMR_FREE,
	BT_WBA_TMR_RUN
};

typedef struct {
	uint16_t Id;
	uint8_t State;
	uint32_t Due;		//!< Millisecond tick of the expiry
} BtWbaTimerSlot_t;

static BtWbaTimerSlot_t s_BleTimer[BT_WBA_TIMER_SLOTS];
static os_timer_id s_BleTimerId;
static os_timer_id s_AppTickTimer;
static bool s_ClockReady;
static uint32_t s_ClockLast;
static uint64_t s_ClockSteps;
static uint32_t s_ClockMul;			//!< Sleep timer step to us, multiplier
static uint32_t s_ClockDiv;			//!< Sleep timer step to us, divider
static volatile bool s_BleTimerQueued;
static volatile bool s_BleTimerOwed;

static void BtWbaTimerEvt(uint32_t Evt, void *pCtx);

static void BtWbaTimerQue(void)
{
	uint32_t pm = BtWbaIrqSave();
	bool bQue = s_BleTimerQueued == false;

	if (bQue)
	{
		s_BleTimerQueued = true;
	}
	BtWbaIrqRestore(pm);

	if (bQue)
	{
		if (BtEvtQue(0, nullptr, BtWbaTimerEvt) == false)
		{
			s_BleTimerQueued = false;
			s_BleTimerOwed = true;
		}
	}
}

// Controller time in milliseconds. ll_intf_get_aligned_us_now spins until the
// next sleep timer edge, up to one step (30.5 us on LSE), so it only supplies
// the step scale. Each call here reads the sleep timer once with
// ll_intf_cmn_get_slptmr_value (ll_sys.h), the read Cube linklayer_plat.c
// uses, and extends its 32-bit count in a short critical section. The radio
// interrupt is never held off for a sleep timer edge.
uint32_t BtWbaMsTick(void)
{
	uint32_t pm;
	uint32_t steps;
	uint32_t multiplier;
	uint32_t divider;
	uint64_t total;

	if (s_ClockReady == false)
	{
		return 0;
	}
	pm = BtWbaIrqSave();
	steps = ll_intf_cmn_get_slptmr_value();
	// Only move forward. A read can be stale while the radio bus clock
	// resynchronizes after sleep (LINKLAYER_PLAT_WaitHclkRdy); a value behind
	// the last one must not become a 2^32 step jump.
	if ((int32_t)(steps - s_ClockLast) > 0)
	{
		s_ClockSteps += (uint32_t)(steps - s_ClockLast);
		s_ClockLast = steps;
	}
	total = s_ClockSteps;
	multiplier = s_ClockMul;
	divider = s_ClockDiv;
	BtWbaIrqRestore(pm);
	// Reduce the controller clock ratios before converting to milliseconds.
	// These exact forms avoid a software 64-bit divide on every tick read.
	if (multiplier == 125U && divider == 4U)
	{
		return (uint32_t)(total >> 5);
	}
	if (multiplier == 15625U && divider == 512U)
	{
		return (uint32_t)((total * 125U) >> 12);
	}
	return (uint32_t)(total * multiplier / ((uint64_t)divider * 1000U));
}

// Refresh the step scale. The link layer recalibrates it on LSI; it is fixed on
// LSE. Thread context only: the aligned read waits for a sleep timer edge.
void BtWbaClockScaleUpdate(void)
{
	uint32_t pm;
	uint32_t multiplier;
	uint32_t divider;

	if (s_ClockReady == false)
	{
		return;
	}
	(void)ll_intf_get_aligned_us_now(&multiplier, &divider);
	if (multiplier == 0U || divider == 0U)
	{
		return;
	}
	pm = BtWbaIrqSave();
	s_ClockMul = multiplier;
	s_ClockDiv = divider;
	BtWbaIrqRestore(pm);
}

static void BtWbaClockStart(void)
{
	uint32_t pm;
	uint32_t multiplier;
	uint32_t divider;

	if (s_ClockReady)
	{
		return;
	}
	// The edge wait stays outside the critical section
	(void)ll_intf_get_aligned_us_now(&multiplier, &divider);
	if (multiplier == 0U || divider == 0U)
	{
		return;
	}
	pm = BtWbaIrqSave();
	if (s_ClockReady == false)
	{
		s_ClockLast = ll_intf_cmn_get_slptmr_value();
		s_ClockSteps = 0;
		s_ClockMul = multiplier;
		s_ClockDiv = divider;
		s_ClockReady = true;
	}
	BtWbaIrqRestore(pm);
}

// Controller timer callback, interrupt context.
static void BtWbaTimerHandler(const void *pCtx)
{
	(void)pCtx;
	(void)BtWbaMsTick();
	BtWbaTimerQue();
}

bool BtWbaTickStart(void (*Callback)(const void *))
{
	if (s_AppTickTimer == nullptr)
	{
		s_AppTickTimer = os_timer_create(Callback, os_timer_periodic, nullptr);
		if (s_AppTickTimer == nullptr)
		{
			return false;
		}
	}
	BtWbaClockStart();
	// os_timer_start/set use controller steps of 31.25 us, including on LSE.
	return os_timer_set(s_AppTickTimer, 32000U) == 0;
}

// Set the trigger to the nearest due time, or stop it when no slot runs.
static void BtWbaTimerArm(void)
{
	uint32_t now = BtWbaMsTick();
	bool bAny = false;
	uint32_t delay = 0;
	uint32_t i;
	int32_t d;
	uint32_t pm = BtWbaIrqSave();

	for (i = 0; i < BT_WBA_TIMER_SLOTS; i++)
	{
		if (s_BleTimer[i].State == BT_WBA_TMR_RUN)
		{
			d = (int32_t)(s_BleTimer[i].Due - now);

			if (d < 1)
			{
				d = 1;
			}
			if (bAny == false || (uint32_t)d < delay)
			{
				delay = (uint32_t)d;
				bAny = true;
			}
		}
	}
	BtWbaIrqRestore(pm);

	(void)os_timer_stop(s_BleTimerId);
	if (bAny)
	{
		// Sample the controller counter at least once per second while a
		// host deadline is active, including before the application tick starts.
		// Keep Due unchanged across these intermediate wakeups.
		if (delay > 1000U)
		{
			delay = 1000U;
		}
		if (os_timer_start(s_BleTimerId, delay * 32U) != 0)
		{
			// A refused controller arm must not strand the software timers.
			// The existing status retry queues a deadline check and re-arm.
			s_BleTimerOwed = true;
		}
	}
}

static void BtWbaTimerEvt(uint32_t Evt, void *pCtx)
{
	uint32_t now;
	uint32_t i;
	uint16_t id;
	bool bFired = false;

	(void)Evt;
	(void)pCtx;

	s_BleTimerQueued = false;

	now = BtWbaMsTick();

	for (i = 0; i < BT_WBA_TIMER_SLOTS; i++)
	{
		if (s_BleTimer[i].State == BT_WBA_TMR_RUN &&
			(int32_t)(now - s_BleTimer[i].Due) >= 0)
		{
			id = s_BleTimer[i].Id;

			s_BleTimer[i].State = BT_WBA_TMR_FREE;
			BLEPLATCB_TimerExpiry(id);
			bFired = true;
		}
	}

	if (bFired)
	{
		BleStackCB_Process();
	}

	BtWbaTimerArm();
}

extern "C" uint8_t BLE_TIMER_Start(uint16_t id, uint32_t timeout)
{
	uint32_t now;
	uint32_t pm;
	uint32_t i;
	int slot = -1;

	if (s_BleTimerId == nullptr)
	{
		s_BleTimerId = os_timer_create(BtWbaTimerHandler, os_timer_once, nullptr);
		if (s_BleTimerId == nullptr)
		{
			return BLE_STATUS_INSUFFICIENT_RESOURCES;
		}
		BtWbaClockStart();
	}

	if (timeout == 0U)
	{
		timeout = 1;
	}

	now = BtWbaMsTick();
	pm = BtWbaIrqSave();

	// A timer with the same id is restarted
	for (i = 0; i < BT_WBA_TIMER_SLOTS; i++)
	{
		if (s_BleTimer[i].State == BT_WBA_TMR_RUN && s_BleTimer[i].Id == id)
		{
			slot = (int)i;
			break;
		}
	}
	if (slot < 0)
	{
		for (i = 0; i < BT_WBA_TIMER_SLOTS; i++)
		{
			if (s_BleTimer[i].State == BT_WBA_TMR_FREE)
			{
				slot = (int)i;
				break;
			}
		}
	}
	if (slot >= 0)
	{
		s_BleTimer[slot].Id = id;
		s_BleTimer[slot].Due = now + timeout;
		s_BleTimer[slot].State = BT_WBA_TMR_RUN;
	}
	BtWbaIrqRestore(pm);

	if (slot < 0)
	{
		return BLE_STATUS_INSUFFICIENT_RESOURCES;
	}

	BtWbaTimerArm();

	return BLE_STATUS_SUCCESS;
}

extern "C" void BLE_TIMER_Stop(uint16_t id)
{
	uint32_t pm;
	uint32_t i;

	if (s_BleTimerId == nullptr)
	{
		return;
	}

	pm = BtWbaIrqSave();

	for (i = 0; i < BT_WBA_TIMER_SLOTS; i++)
	{
		if (s_BleTimer[i].State == BT_WBA_TMR_RUN && s_BleTimer[i].Id == id)
		{
			s_BleTimer[i].State = BT_WBA_TMR_FREE;
		}
	}
	BtWbaIrqRestore(pm);

	BtWbaTimerArm();
}

//----------------------------------------------------------------------------
// Random number generator. The kernel clock is HSI16. The peripheral is
// started when a number is needed and stopped when no user is left.
//----------------------------------------------------------------------------

static uint8_t s_RngUsers;

// Start the generator. Returns false when it does not give data.
static bool BtWbaRngStart(void)
{
	uint32_t n;

	SET_BIT(RCC->CR, RCC_CR_HSION);
	n = BT_WBA_RNG_POLL_MAX;
	while ((RCC->CR & RCC_CR_HSIRDY) == 0U && n > 0U)
	{
		n--;
	}

	MODIFY_REG(RCC->CCIPR2, RCC_CCIPR2_RNGSEL, RCC_CCIPR2_RNGSEL_1);
	SET_BIT(RCC->AHB2ENR, RCC_AHB2ENR_RNGEN);
	(void)RCC->AHB2ENR;

	if ((RNG->CR & RNG_CR_RNGEN) != 0U)
	{
		return true;
	}

	// Clock error detection off, conditioning reset, configuration 2 as set
	// by the HAL for this series.
	MODIFY_REG(RNG->CR, RNG_CR_CED | RNG_CR_CONDRST | RNG_CR_RNG_CONFIG2,
			   RNG_CR_CED | RNG_CR_CONDRST | (1UL << RNG_CR_RNG_CONFIG2_Pos));
	CLEAR_BIT(RNG->CR, RNG_CR_CONDRST);
	n = BT_WBA_RNG_POLL_MAX;
	while ((RNG->CR & RNG_CR_CONDRST) != 0U && n > 0U)
	{
		n--;
	}

	SET_BIT(RNG->CR, RNG_CR_RNGEN);
	n = BT_WBA_RNG_POLL_MAX;
	while ((RNG->SR & RNG_SR_DRDY) == 0U && n > 0U)
	{
		n--;
	}

	return (RNG->SR & RNG_SR_DRDY) != 0U;
}

static void BtWbaRngStop(void)
{
	CLEAR_BIT(RNG->CR, RNG_CR_RNGEN);
	CLEAR_BIT(RCC->AHB2ENR, RCC_AHB2ENR_RNGEN);
}

extern "C" void HW_RNG_EnableClock(uint8_t UserMask)
{
	s_RngUsers |= UserMask;
	BtWbaRngStart();
}

extern "C" void HW_RNG_DisableClock(uint8_t UserMask)
{
	s_RngUsers &= (uint8_t)~UserMask;
	if (s_RngUsers == 0U)
	{
		BtWbaRngStop();
	}
}

extern "C" void HW_RNG_Get(uint8_t n, uint32_t *pVal)
{
	while (n-- > 0U)
	{
		bool bOk = false;

		for (int retry = 0; retry < BT_WBA_RNG_RETRY && bOk == false; retry++)
		{
			if (BtWbaRngStart() == false)
			{
				BtWbaRngStop();
				continue;
			}

			// Seed or clock error: clear the flags and restart the generator
			if ((RNG->SR & (RNG_SR_SEIS | RNG_SR_CEIS)) != 0U)
			{
				CLEAR_BIT(RNG->SR, RNG_SR_SEIS | RNG_SR_CEIS);
				BtWbaRngStop();
				continue;
			}

			if ((RNG->SR & RNG_SR_DRDY) != 0U)
			{
				*pVal = RNG->DR;
				bOk = true;
			}
		}

		if (bOk == false)
		{
			// The generator does not answer. Keys must not come from a
			// guess, so stop here.
			Error_Handler();
		}
		pVal++;
	}

	if (s_RngUsers == 0U)
	{
		BtWbaRngStop();
	}
}

// Entries of the Cube RNG module that are called by Cube application code
// only. They are kept so a project that includes the module header links.
extern "C" void HW_RNG_Init(void)
{
	BtWbaRngStart();
}

extern "C" void HW_RNG_Disable(void)
{
	BtWbaRngStop();
}

//----------------------------------------------------------------------------
// AES for the stack: ECB, CMAC and CCM through the IOsonata engines.
// ST ECB reverses the key, input and output relative to standard AES byte
// strings. ST CCM reverses only the key; CMAC uses standard byte order.
//----------------------------------------------------------------------------

// Every STM32WBA part has the AES block, so the hardware engine is the one the
// stack uses. The software engine takes a call only when the hardware one is
// busy.
alignas(CryptoWbaAes) static uint8_t s_HwAesMem[CRYPTO_WBA_AES_MEMSIZE];
alignas(CryptoSoftAes) static uint8_t s_AesMem[CRYPTO_SOFTAES_MEMSIZE];
static CryptoWbaAes *s_pHwAes;
static CryptoSoftAes *s_pAes;
static uint8_t s_CmacKey[16];
static uint8_t s_CmacIv[16];
static bool s_CmacFailed;

// Engine for one attempt of a call: pass 0 is the hardware engine, pass 1 the
// software engine. A call goes to pass 1 only after pass 0 answered BUSY. The
// engines are built on first use with interrupts off, as the calls can come
// from interrupt context.
static CryptoSoftAes *BtWbaAes(int Pass)
{
	if (Pass == 0)
	{
		if (s_pHwAes == nullptr)
		{
			uint32_t pm = BtWbaIrqSave();

			if (s_pHwAes == nullptr)
			{
				s_pHwAes = CryptoWbaAesCreate(s_HwAesMem, sizeof(s_HwAesMem));
			}
			BtWbaIrqRestore(pm);
		}

		return s_pHwAes;
	}

	if (s_pAes == nullptr)
	{
		uint32_t pm = BtWbaIrqSave();

		if (s_pAes == nullptr)
		{
			CryptoSoftAes *p = CryptoSoftAesCreate(s_AesMem, sizeof(s_AesMem));

			if (p != nullptr)
			{
				p->Enable();
			}
			s_pAes = p;
		}
		BtWbaIrqRestore(pm);
	}

	return s_pAes;
}

static void BtWbaAesKey(CryptoKey &Key, const uint8_t *pKey, uint32_t Usage)
{
	Key.Type = CRYPTO_KEY_AES_128;
	Key.Loc = CRYPTO_KEY_LOC_PLAIN;
	Key.Usage = Usage;
	Key.Plain.pData = pKey;
	Key.Plain.Len = 16;
}

extern "C" void BAES_Reset(void)
{
	CryptoSecureWipe(s_CmacKey, sizeof(s_CmacKey));
	CryptoSecureWipe(s_CmacIv, sizeof(s_CmacIv));
	s_CmacFailed = false;
}

// Match Cube baes_ecb.c / hw_aes.c: the BLE e() interface is little endian.
// Keep conversion here; CryptoSoftAes and the common SMP use standard AES order.
static void BtWbaAesReverse(uint8_t *pDst, const uint8_t *pSrc)
{
	for (size_t i = 0; i < 16; i++)
	{
		pDst[i] = pSrc[15 - i];
	}
}

extern "C" void BAES_EcbCrypt(const uint8_t *pKey, const uint8_t *pIn,
							  uint8_t *pOut, int bEnc)
{
	CryptoKey key;
	uint8_t aesKey[16];
	uint8_t input[16];
	uint8_t output[16] = {};
	CRYPTO_STATUS st = CRYPTO_STATUS_BUSY;

	BtWbaAesReverse(aesKey, pKey);
	BtWbaAesReverse(input, pIn);
	BtWbaAesKey(key, aesKey, CRYPTO_KEY_USE_ENCRYPT | CRYPTO_KEY_USE_DECRYPT);
	for (int pass = 0; pass < 2 && st == CRYPTO_STATUS_BUSY; pass++)
	{
		CryptoSoftAes *p = BtWbaAes(pass);

		if (p == nullptr)
		{
			continue;
		}
		st = p->Cipher(CRYPTO_CIPHER_ECB, bEnc, key, nullptr, 0,
					   input, sizeof(input), output);
	}
	if (st != CRYPTO_STATUS_OK)
	{
		memset(output, 0, sizeof(output));
	}
	BtWbaAesReverse(pOut, output);
	CryptoSecureWipe(aesKey, sizeof(aesKey));
	CryptoSecureWipe(input, sizeof(input));
	CryptoSecureWipe(output, sizeof(output));
}

extern "C" void BAES_CmacSetKey(const uint8_t *pKey)
{
	memcpy(s_CmacKey, pKey, sizeof(s_CmacKey));
	memset(s_CmacIv, 0, sizeof(s_CmacIv));
	s_CmacFailed = false;
}

// Standard byte order here; the ST ECB entry above has different byte order.
static bool BtWbaCmacEncrypt(const uint8_t *pIn, uint8_t *pOut)
{
	CryptoKey key;
	CRYPTO_STATUS st = CRYPTO_STATUS_BUSY;

	BtWbaAesKey(key, s_CmacKey, CRYPTO_KEY_USE_ENCRYPT);
	for (int pass = 0; pass < 2 && st == CRYPTO_STATUS_BUSY; pass++)
	{
		CryptoSoftAes *p = BtWbaAes(pass);

		if (p == nullptr)
		{
			continue;
		}
		st = p->Cipher(CRYPTO_CIPHER_ECB, 1, key, nullptr, 0, pIn, 16, pOut);
	}

	return st == CRYPTO_STATUS_OK;
}

static void BtWbaCmacSubkey(uint8_t *pKey)
{
	uint8_t carry = 0;
	uint8_t msb = pKey[0] >> 7;

	for (int i = 15; i >= 0; i--)
	{
		uint8_t value = pKey[i];

		pKey[i] = (uint8_t)((value << 1) | carry);
		carry = value >> 7;
	}
	pKey[15] ^= (uint8_t)(0x87U & (0U - msb));
}

// Cube permits whole-block appends with no output. Only the final call
// applies CMAC padding and subkeys; its input may have any length.
extern "C" void BAES_CmacCompute(const uint8_t *pIn, uint32_t Size, uint8_t *pOut)
{
	uint8_t block[16] = {};
	uint8_t subkey[16] = {};
	uint32_t last = 0;

	if ((Size != 0 && pIn == nullptr) || (pOut == nullptr && Size % 16 != 0))
	{
		s_CmacFailed = true;
	}
	if (pOut != nullptr)
	{
		last = Size % 16;
		if (Size != 0 && last == 0)
		{
			last = 16;
		}
	}

	uint32_t prefix = Size - last;

	while (prefix != 0 && s_CmacFailed == false)
	{
		for (size_t i = 0; i < 16; i++)
		{
			block[i] = pIn[i] ^ s_CmacIv[i];
		}
		s_CmacFailed = BtWbaCmacEncrypt(block, s_CmacIv) == false;
		pIn += 16;
		prefix -= 16;
	}

	if (pOut != nullptr)
	{
		if (s_CmacFailed == false)
		{
			memset(block, 0, sizeof(block));
			s_CmacFailed = BtWbaCmacEncrypt(block, subkey) == false;
			BtWbaCmacSubkey(subkey);
			if (last < 16)
			{
				BtWbaCmacSubkey(subkey);
				block[last] = 0x80;
			}
			if (last != 0)
			{
				memcpy(block, pIn, last);
			}
			for (size_t i = 0; i < 16; i++)
			{
				block[i] ^= s_CmacIv[i] ^ subkey[i];
			}
			if (s_CmacFailed == false)
			{
				s_CmacFailed = BtWbaCmacEncrypt(block, pOut) == false;
			}
		}
		if (s_CmacFailed)
		{
			memset(pOut, 0, 16);
		}
		CryptoSecureWipe(s_CmacIv, sizeof(s_CmacIv));
		CryptoSecureWipe(s_CmacKey, sizeof(s_CmacKey));
	}
	CryptoSecureWipe(block, sizeof(block));
	CryptoSecureWipe(subkey, sizeof(subkey));
}

// Mode 0 encrypts and writes the tag. Mode 1 decrypts and checks the tag, the
// result is not zero when the tag is wrong.
extern "C" int BAES_CcmCrypt(uint8_t Mode, const uint8_t *pKey,
							 uint8_t IvLen, const uint8_t *pIv,
							 uint16_t AddLen, const uint8_t *pAdd,
							 uint16_t InLen, const uint8_t *pIn,
							 uint8_t TagLen, uint8_t *pTag, uint8_t *pOut)
{
	CryptoKey key;
	uint8_t aesKey[16];
	CRYPTO_STATUS st = CRYPTO_STATUS_BUSY;

	// HW_AES_InitCcm loads KEYR0 first, unlike the standard-order CMAC key.
	BtWbaAesReverse(aesKey, pKey);
	BtWbaAesKey(key, aesKey, Mode == 0U ? CRYPTO_KEY_USE_ENCRYPT :
										   CRYPTO_KEY_USE_DECRYPT);
	for (int pass = 0; pass < 2 && st == CRYPTO_STATUS_BUSY; pass++)
	{
		CryptoSoftAes *p = BtWbaAes(pass);

		if (p == nullptr)
		{
			continue;
		}
		if (Mode == 0U)
		{
			st = p->Seal(CRYPTO_AEAD_AES_CCM, key, pIv, IvLen, pAdd, AddLen,
						 pIn, InLen, pOut, pTag, TagLen);
		}
		else
		{
			st = p->Open(CRYPTO_AEAD_AES_CCM, key, pIv, IvLen, pAdd, AddLen,
						 pIn, InLen, pOut, pTag, TagLen);
		}
	}

	CryptoSecureWipe(aesKey, sizeof(aesKey));
	return st == CRYPTO_STATUS_OK ? 0 : 1;
}

//----------------------------------------------------------------------------
// Link layer interrupts. The vector table of the IOsonata system module has
// weak entries for them. The callbacks are set by LINKLAYER_PLAT_SetupRadioIT
// and LINKLAYER_PLAT_SetupSwLowIT in linklayer_plat.c.
//----------------------------------------------------------------------------

extern "C" void (*radio_callback)(void);
extern "C" void (*low_isr_callback)(void);
extern "C" volatile uint8_t radio_sw_low_isr_is_running_high_prio;

extern "C" void RADIO_IRQHandler(void)
{
	if (radio_callback != nullptr)
	{
		radio_callback();
	}

	CLEAR_BIT(RCC->RADIOENR, RCC_RADIOENR_STRADIOCLKON);
	__ISB();
}

// Software low priority radio interrupt. It runs on the HASH vector.
extern "C" void HASH_IRQHandler(void)
{
	// Block nested calls
	NVIC_DisableIRQ((IRQn_Type)RADIO_SW_LOW_INTR_NUM);

	if (low_isr_callback != nullptr)
	{
		low_isr_callback();
	}

	// A request made at high priority is done, back to the low priority
	if (radio_sw_low_isr_is_running_high_prio != 0)
	{
		NVIC_SetPriority((IRQn_Type)RADIO_SW_LOW_INTR_NUM, RADIO_INTR_PRIO_LOW);
		radio_sw_low_isr_is_running_high_prio = 0;
	}

	NVIC_EnableIRQ((IRQn_Type)RADIO_SW_LOW_INTR_NUM);
}

//----------------------------------------------------------------------------
// Work that did not fit in the event queue is offered again from here.
//----------------------------------------------------------------------------

void BtWbaPlatformRetry(void)
{
	if (s_SeqOwed || s_SeqPend != 0U)
	{
		s_SeqOwed = false;
		BtWbaSeqQue();
	}

	if (s_BleTimerOwed)
	{
		s_BleTimerOwed = false;
		BtWbaTimerQue();
	}
}
