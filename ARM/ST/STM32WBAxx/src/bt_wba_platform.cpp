/**-------------------------------------------------------------------------
@file	bt_wba_platform.cpp

@brief	STM32WBA BLE stack platform layer on IOsonata.

The STM32CubeWBA BLE host stack and its link layer library call a small set of
platform functions. A Cube project supplies them with its HAL, the sequencer,
the timer server, the RTC, the memory manager and the flash manager. This file
supplies them with IOsonata code instead, so the Cube project files that remain
in a build are bleplat.c, linklayer_plat.c, ll_sys_if.c, ll_sys_startup.c,
power_table.c and the PKA controller:

- HAL_NVIC_* and the radio baseband clock: direct NVIC and RCC access.
- Sequencer (UTIL_SEQ_RegTask, UTIL_SEQ_SetTask): the tasks are run from the
  Bluetooth event queue, BtEvtQue.
- BLE_TIMER_Start/Stop: software timer slots over one IOsonata LPTIM trigger.
- HW_RNG_Get and the RNG clock requests: RNG peripheral, polled.
- BAES_*: the IOsonata CryptoSoftAes engine (ECB, CMAC and CCM).
- PKA controller hooks: queue, semaphore and interrupt handler.
- Link layer interrupts (RADIO and HASH vectors), HSE32 wait, system clock
  manager entries, debug signals: small functions.

Known limits:
- The temperature used by the link layer for radio calibration is fixed at 25
  degrees C. There is no ADC based measurement yet.
- AES is software. The AES peripheral is not used.
- Nothing here is tested on hardware.

@author	Hoang Nguyen Hoan
@date	Oct. 14, 2026

@license

Copyright (c) 2026, I-SYST inc., all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>

#include "stm32wbaxx.h"

#include "app_conf.h"
#include "bt_wba_stack.h"

extern "C" {
#include "ll_intf_cmn.h"
}

#include "istddef.h"
#include "coredev/timer.h"
#include "crypto/crypto_softaes.h"
#include "bluetooth/bt_app.h"

#include "coredev/system_core_clock.h"
#include "bt_wba_platform.h"

// Defined by the port in bt_app_stm32wba.cpp: runs the host stack
extern "C" void BleStackCB_Process(void);

// Bit of the PKA controller task in the sequencer mask. The link layer task
// uses the bit its Target file defines, which is below this one.
#define BT_WBA_SEQ_PKA			31U

// Temperature, in degrees C, given to the link layer for radio calibration
#define BT_WBA_TEMPERATURE_C	25
// Offset the link layer adds to the temperature (TEMPMEAS_MIN_TEMP_LIMIT)
#define BT_WBA_TEMPERATURE_OFS	40

#define BT_WBA_TIMER_SLOTS		8U

// Busy loop limits, in loop passes. They only stop a hang when the hardware
// does not answer.
#define BT_WBA_HSE_POLL_MAX		2000000UL
#define BT_WBA_RNG_POLL_MAX		200000UL
#define BT_WBA_PKA_POLL_MAX		50000000UL

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

extern "C" {

__attribute__((weak)) void HAL_NVIC_SetPriority(IRQn_Type IRQn,
												uint32_t PreemptPriority,
												uint32_t SubPriority)
{
	(void)SubPriority;

	NVIC_SetPriority(IRQn, PreemptPriority);
}

__attribute__((weak)) void HAL_NVIC_EnableIRQ(IRQn_Type IRQn)
{
	NVIC_EnableIRQ(IRQn);
}

__attribute__((weak)) void HAL_NVIC_DisableIRQ(IRQn_Type IRQn)
{
	NVIC_DisableIRQ(IRQn);
}

__attribute__((weak)) void HAL_NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
	NVIC_SetPendingIRQ(IRQn);
}

__attribute__((weak)) void HAL_RCCEx_EnableRadioBBClock(void)
{
	SET_BIT(RCC->RADIOENR, RCC_RADIOENR_BBCLKEN);
}

__attribute__((weak)) void HAL_RCCEx_DisableRadioBBClock(void)
{
	CLEAR_BIT(RCC->RADIOENR, RCC_RADIOENR_BBCLKEN);
}

// Called by the PKA controller and the link layer when something fails that
// cannot be recovered.
__attribute__((weak)) void Error_Handler(void)
{
	__disable_irq();
	while (1)
	{
	}
}

// Link layer debug pins are not used.
void LINKLAYER_DEBUG_SIGNAL_SET(uint32_t Signal)
{
	(void)Signal;
}

void LINKLAYER_DEBUG_SIGNAL_RESET(uint32_t Signal)
{
	(void)Signal;
}

void LINKLAYER_DEBUG_SIGNAL_TOGGLE(uint32_t Signal)
{
	(void)Signal;
}

//----------------------------------------------------------------------------
// System clock manager entries. The clock is set by SystemInit, see
// system_stm32wbaxx.c, so only the HSE32 wait does work.
//----------------------------------------------------------------------------

void SCM_HSE_WaitUntilReady(void)
{
	uint32_t n = BT_WBA_HSE_POLL_MAX;

	SET_BIT(RCC->CR, RCC_CR_HSEON);
	while ((RCC->CR & RCC_CR_HSERDY) == 0U && n > 0U)
	{
		n--;
	}
}

/**
 * @brief	Start HSE32 and select the radio sleep timer clock.
 *
 * The link layer reads the sleep timer clock source and stops when it is not
 * set. LSE is used when the project asks for a crystal low power oscillator,
 * otherwise LSI1 is used.
 */
void BtWbaPlatformInit(void)
{
	uint32_t n = BT_WBA_HSE_POLL_MAX;
	uint32_t src;

	SCM_HSE_WaitUntilReady();

	SET_BIT(PWR->DBPR, PWR_DBPR_DBP);

	if (g_McuOsc.LowPwrOsc.Type != OSC_TYPE_RC)
	{
		SET_BIT(RCC->BDCR1, RCC_BDCR1_LSEON);
		while ((RCC->BDCR1 & RCC_BDCR1_LSERDY) == 0U && n > 0U)
		{
			n--;
		}
		src = RCC_BDCR1_RADIOSTSEL_0;
	}
	else
	{
		SET_BIT(RCC->BDCR1, RCC_BDCR1_LSI1ON);
		while ((RCC->BDCR1 & RCC_BDCR1_LSI1RDY) == 0U && n > 0U)
		{
			n--;
		}
		src = RCC_BDCR1_RADIOSTSEL_1;
	}

	MODIFY_REG(RCC->BDCR1, RCC_BDCR1_RADIOSTSEL, src);
}

void scm_notifyradiostate(uint32_t State)
{
	(void)State;
}

void scm_setsystemclock(uint32_t User, uint32_t Clock)
{
	(void)User;
	(void)Clock;
}

//----------------------------------------------------------------------------
// Temperature for the radio calibration
//----------------------------------------------------------------------------

void TEMPMEAS_RequestTemperatureMeasurement(void)
{
	ll_intf_cmn_set_temperature_value((uint32_t)(BT_WBA_TEMPERATURE_C + BT_WBA_TEMPERATURE_OFS));
}

} // extern "C"

//----------------------------------------------------------------------------
// Sequencer: the stack and the link layer register tasks and set them. The
// tasks run from the Bluetooth event queue.
//----------------------------------------------------------------------------

static void (*s_SeqTask[32])(void);
static volatile uint32_t s_SeqPend;
static volatile bool s_SeqQueued;
static volatile bool s_SeqOwed;

static void BtWbaSeqEvt(uint32_t Evt, void *pCtx);

static void BtWbaSeqQue(void)
{
	uint32_t pm = BtWbaIrqSave();
	bool bQue = s_SeqQueued == false;

	if (bQue)
	{
		s_SeqQueued = true;
	}
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

static void BtWbaSeqEvt(uint32_t Evt, void *pCtx)
{
	(void)Evt;
	(void)pCtx;

	s_SeqQueued = false;

	while (true)
	{
		uint32_t pm = BtWbaIrqSave();
		uint32_t pend = s_SeqPend;

		s_SeqPend = 0;
		BtWbaIrqRestore(pm);

		if (pend == 0U)
		{
			break;
		}

		for (uint32_t i = 0; i < 32U; i++)
		{
			if ((pend & (1UL << i)) != 0U && s_SeqTask[i] != nullptr)
			{
				s_SeqTask[i]();
			}
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

	uint32_t pm = BtWbaIrqSave();

	s_SeqPend |= TaskBm;
	BtWbaIrqRestore(pm);

	BtWbaSeqQue();
}

//----------------------------------------------------------------------------
// BLE stack timers: slots over one LPTIM trigger. The trigger is set to the
// nearest due time. The expiry callbacks run from the event queue.
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
static TimerDev_t s_BleTimerDev;
static bool s_BleTimerInit;
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

// Timer trigger, interrupt context
static void BtWbaTimerHandler(TimerDev_t *pTimer, uint32_t Evt)
{
	(void)pTimer;

	if (Evt & TIMER_EVT_TRIGGER(0))
	{
		BtWbaTimerQue();
	}
}

static const TimerCfg_t s_BleTimerCfg = {
	.DevNo = 0,
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 0,			// 0 => Default frequency
	.IntPrio = 6,
	.EvtHandler = BtWbaTimerHandler
};

// Set the trigger to the nearest due time, or stop it when no slot runs.
static void BtWbaTimerArm(void)
{
	uint32_t now = TimerGetMilisecond(&s_BleTimerDev);
	bool bAny = false;
	uint32_t delay = 0;

	uint32_t pm = BtWbaIrqSave();

	for (uint32_t i = 0; i < BT_WBA_TIMER_SLOTS; i++)
	{
		if (s_BleTimer[i].State == BT_WBA_TMR_RUN)
		{
			int32_t d = (int32_t)(s_BleTimer[i].Due - now);

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

	TimerDisableTrigger(&s_BleTimerDev, 0);
	if (bAny)
	{
		msTimerEnableTrigger(&s_BleTimerDev, 0, delay, TIMER_TRIG_TYPE_SINGLE,
							 nullptr, nullptr);
	}
}

static void BtWbaTimerEvt(uint32_t Evt, void *pCtx)
{
	(void)Evt;
	(void)pCtx;

	s_BleTimerQueued = false;

	uint32_t now = TimerGetMilisecond(&s_BleTimerDev);
	bool bFired = false;

	for (uint32_t i = 0; i < BT_WBA_TIMER_SLOTS; i++)
	{
		if (s_BleTimer[i].State == BT_WBA_TMR_RUN &&
			(int32_t)(now - s_BleTimer[i].Due) >= 0)
		{
			uint16_t id = s_BleTimer[i].Id;

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
	if (s_BleTimerInit == false)
	{
		if (TimerInit(&s_BleTimerDev, &s_BleTimerCfg) == false)
		{
			return BLE_STATUS_FAILED;
		}
		s_BleTimerInit = true;
	}

	if (timeout == 0U)
	{
		timeout = 1;
	}

	uint32_t now = TimerGetMilisecond(&s_BleTimerDev);
	int slot = -1;

	uint32_t pm = BtWbaIrqSave();

	// A timer with the same id is restarted
	for (uint32_t i = 0; i < BT_WBA_TIMER_SLOTS; i++)
	{
		if (s_BleTimer[i].State == BT_WBA_TMR_RUN && s_BleTimer[i].Id == id)
		{
			slot = (int)i;
			break;
		}
	}
	if (slot < 0)
	{
		for (uint32_t i = 0; i < BT_WBA_TIMER_SLOTS; i++)
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
	if (s_BleTimerInit == false)
	{
		return;
	}

	uint32_t pm = BtWbaIrqSave();

	for (uint32_t i = 0; i < BT_WBA_TIMER_SLOTS; i++)
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
// started when a number or the PKA needs it and stopped when no user is left.
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
// AES for the stack: ECB, CMAC and CCM on the IOsonata software engine.
// Byte arrays are in the natural AES order, as the Cube BasicAES module.
//----------------------------------------------------------------------------

static uint64_t s_AesMem[(CRYPTO_SOFTAES_MEMSIZE + 7U) / 8U];
static CryptoSoftAes *s_pAes;
static uint8_t s_CmacKey[16];

static CryptoSoftAes *BtWbaAes(void)
{
	if (s_pAes == nullptr)
	{
		s_pAes = CryptoSoftAesCreate(s_AesMem, sizeof(s_AesMem));
		if (s_pAes != nullptr)
		{
			s_pAes->Enable();
		}
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
}

// The stack encrypts only (BLEPLAT_AesEcbEncrypt). The engine has no ECB
// decrypt, so a decrypt request answers zeros.
extern "C" void BAES_EcbCrypt(const uint8_t *pKey, const uint8_t *pIn,
							  uint8_t *pOut, int bEnc)
{
	CryptoSoftAes *p = BtWbaAes();
	CryptoKey key;

	if (p == nullptr)
	{
		memset(pOut, 0, 16);
		return;
	}

	BtWbaAesKey(key, pKey, CRYPTO_KEY_USE_ENCRYPT | CRYPTO_KEY_USE_DECRYPT);
	if (p->Cipher(CRYPTO_CIPHER_ECB, bEnc, key, nullptr, 0, pIn, 16, pOut) != CRYPTO_STATUS_OK)
	{
		memset(pOut, 0, 16);
	}
}

extern "C" void BAES_CmacSetKey(const uint8_t *pKey)
{
	memcpy(s_CmacKey, pKey, sizeof(s_CmacKey));
}

// One call computes the whole tag. The stack never calls it in pieces. The
// Cube module accepts a call with no output to continue with more data, which
// is not supported here, so such a call does nothing.
extern "C" void BAES_CmacCompute(const uint8_t *pIn, uint32_t Size, uint8_t *pOut)
{
	if (pOut == nullptr)
	{
		return;
	}

	CryptoSoftAes *p = BtWbaAes();
	CryptoKey key;

	if (p == nullptr)
	{
		memset(pOut, 0, 16);
		return;
	}

	BtWbaAesKey(key, s_CmacKey, CRYPTO_KEY_USE_SIGN);
	if (p->Mac(CRYPTO_MAC_CMAC, key, pIn, Size, pOut, 16) != CRYPTO_STATUS_OK)
	{
		memset(pOut, 0, 16);
	}
}

// Mode 0 encrypts and writes the tag. Mode 1 decrypts and checks the tag, the
// result is not zero when the tag is wrong.
extern "C" int BAES_CcmCrypt(uint8_t Mode, const uint8_t *pKey,
							 uint8_t IvLen, const uint8_t *pIv,
							 uint16_t AddLen, const uint8_t *pAdd,
							 uint16_t InLen, const uint8_t *pIn,
							 uint8_t TagLen, uint8_t *pTag, uint8_t *pOut)
{
	CryptoSoftAes *p = BtWbaAes();
	CryptoKey key;
	CRYPTO_STATUS st;

	if (p == nullptr)
	{
		return 1;
	}

	if (Mode == 0U)
	{
		BtWbaAesKey(key, pKey, CRYPTO_KEY_USE_ENCRYPT);
		st = p->Seal(CRYPTO_AEAD_AES_CCM, key, pIv, IvLen, pAdd, AddLen,
					 pIn, InLen, pOut, pTag, TagLen);
	}
	else
	{
		BtWbaAesKey(key, pKey, CRYPTO_KEY_USE_DECRYPT);
		st = p->Open(CRYPTO_AEAD_AES_CCM, key, pIv, IvLen, pAdd, AddLen,
					 pIn, InLen, pOut, pTag, TagLen);
	}

	return st == CRYPTO_STATUS_OK ? 0 : 1;
}

//----------------------------------------------------------------------------
// Hooks of the Cube PKA controller (pka_ctrl.c)
//----------------------------------------------------------------------------

static volatile uint8_t s_PkaSem;
static volatile uint8_t s_PkaMut;
static volatile bool s_PkaTaskReg;

extern "C" void PKACTRL_BG_Process(void);

extern "C" void PKACTRL_CB_Process(void)
{
	if (s_PkaTaskReg == false)
	{
		UTIL_SEQ_RegTask(1UL << BT_WBA_SEQ_PKA, 0, PKACTRL_BG_Process);
		s_PkaTaskReg = true;
	}
	UTIL_SEQ_SetTask(1UL << BT_WBA_SEQ_PKA, 0);
}

// The stack is the only user of the controller and it runs in one context,
// so the mutex has nothing to wait for.
extern "C" int PKACTRL_MutexTake(void)
{
	s_PkaMut++;

	return 0;
}

extern "C" int PKACTRL_MutexRelease(void)
{
	s_PkaMut = 0;

	return 0;
}

// The semaphore is taken when a PKA operation starts and released by the end
// of operation interrupt. A second take waits for that release.
extern "C" int PKACTRL_TakeSemEndOfOperation(void)
{
	uint32_t n = BT_WBA_PKA_POLL_MAX;

	while (s_PkaSem != 0U && n > 0U)
	{
		n--;
	}
	s_PkaSem = 1;

	return 0;
}

extern "C" int PKACTRL_ReleaseSemEndOfOperation(void)
{
	s_PkaSem = 0;

	return 0;
}

extern "C" void PKACTRL_EndOfProcessCb(void);

extern "C" void PKA_IRQHandler(void)
{
	if ((PKA->SR & PKA_SR_PROCENDF) != 0U)
	{
		PKA->CLRFR = PKA_CLRFR_PROCENDFC;
		PKACTRL_EndOfProcessCb();
	}
}

//----------------------------------------------------------------------------
// Link layer interrupts. The vector table of the IOsonata system module has
// weak entries for them. The callbacks are set by LINKLAYER_PLAT_SetupRadioIT
// and LINKLAYER_PLAT_SetupSwLowIT in linklayer_plat.c.
//----------------------------------------------------------------------------

extern "C" {

extern void (*radio_callback)(void);
extern void (*low_isr_callback)(void);
extern volatile uint8_t radio_sw_low_isr_is_running_high_prio;

void RADIO_IRQHandler(void)
{
	if (radio_callback != nullptr)
	{
		radio_callback();
	}

	CLEAR_BIT(RCC->RADIOENR, RCC_RADIOENR_STRADIOCLKON);
	__ISB();
}

// Software low priority radio interrupt. It runs on the HASH vector.
void HASH_IRQHandler(void)
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

} // extern "C"

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
