/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
#include <stdint.h>
#include <nrf_sdm.h>
#include <nrf_soc.h>
#include <bm/softdevice_handler/nrf_sdh.h>
#include <bm/bm_scheduler.h>

#include "coredev/system_core_clock.h"
#include "idelay.h"

/**
 * @defgroup nrf_sd_isr_vectors SoftDevice Interrupt Vector Table Offsets
 * @{
 *
 *  @brief SoftDevice interrupt vector table offsets.
 *         The SoftDevice interrupt vector table contains only the addresses of the interrupt handlers
 *         required by the SoftDevice. The table is located at the SoftDevice base address. When the SoftDevice
 *         is enabled, the application must forward the interrupts corresponding to the defined offsets
 *         to the SoftDevice. The address of the interrupt handler is located at the SoftDevice base address plus the offset.
 *
 *         An example of how to forward an interrupt to the SoftDevice is shown below:
 *
 *         @code
 *         SVC_Handler:
 *           LDR   R0, =NRF_SD_ISR_OFFSET_SVC
 *           LDR   R1, =SOFTDEVICE_BASE_ADDRESS
 *           LDR   R1, [R1, R0]
 *           BX    R1
 *         @endcode
 */
#define NRF_SD_ISR_OFFSET_RESET          (0x0000) /**< SoftDevice Reset Handler address offset */
#define NRF_SD_ISR_OFFSET_HARDFAULT      (0x0004) /**< SoftDevice HardFault Handler address offset */
#define NRF_SD_ISR_OFFSET_SVC            (0x0008) /**< SoftDevice SVC Handler address offset */
#define NRF_SD_ISR_OFFSET_SWI00          (0x000c) /**< SoftDevice SWI00 Handler address offset */
#define NRF_SD_ISR_OFFSET_AAR00_CCM00    (0x0010) /**< SoftDevice AAR00_CCM00 Handler address offset */
#define NRF_SD_ISR_OFFSET_ECB00          (0x0014) /**< SoftDevice ECB00 Handler address offset */
#define NRF_SD_ISR_OFFSET_TIMER10        (0x0018) /**< SoftDevice TIMER10 Handler address offset */
#define NRF_SD_ISR_OFFSET_RADIO_0        (0x001c) /**< SoftDevice RADIO_0 Handler address offset */
#define NRF_SD_ISR_OFFSET_GRTC_3         (0x0020) /**< SoftDevice GRTC_3 Handler address offset */
#define NRF_SD_ISR_OFFSET_CLOCK_POWER    (0x0024) /**< SoftDevice CLOCK_POWER Handler address offset */

/* Stringify helpers — expand macro value THEN stringify */
#define _SD_XSTR(x) #x
#define SD_XSTR(x)  _SD_XSTR(x)


uint32_t softdevice_vector_forward_address;


static atomic_t sdh_is_suspended;	/* Whether the SoftDevice event interrupts are disabled. */
static atomic_t sdh_transition;		/* Whether enable/disable process was started. */

static char *state_to_str(enum nrf_sdh_state_evt s)
{
	switch (s) {
	case NRF_SDH_STATE_EVT_ENABLE_PREPARE:
		return "enabling";
	case NRF_SDH_STATE_EVT_ENABLED:
		return "enabled";
	case NRF_SDH_STATE_EVT_DISABLE_PREPARE:
		return "disabling";
	case NRF_SDH_STATE_EVT_DISABLED:
		return "disabled";
	default:
		return "unknown";
	};
}

/**
 * @brief Notify a state change to state observers.
 *
 * Extern in nrf_sdh_ble.c
 *
 * @param state The state to be notified.
 * @return true If any observers are busy.
 * @return false If no observers are busy.
 */
bool sdh_state_evt_observer_notify(enum nrf_sdh_state_evt state)
{
	bool busy;
	bool all_ready;
	bool busy_is_allowed;

	if (IS_ENABLED(CONFIG_NRF_SDH_STR_TABLES)) {
		LOG_DBG("State change: %s", state_to_str(state));
	} else {
		LOG_DBG("State change: %#x", state);
	}

	all_ready = true;
	busy_is_allowed = (state == NRF_SDH_STATE_EVT_ENABLE_PREPARE) ||
			  (state == NRF_SDH_STATE_EVT_DISABLE_PREPARE);

	TYPE_SECTION_FOREACH(struct nrf_sdh_state_evt_observer, nrf_sdh_state_evt_observers, obs) {
		/* If it's a _PREPARE event, dispatch only to busy observers, and update
		 * their busy state in RAM. Otherwise dispatch unconditionally to all observers.
		 */
		if (busy_is_allowed && obs->is_busy) {
			obs->is_busy = !!obs->handler(state, obs->context);
			if (obs->is_busy) {
				LOG_DBG("SoftDevice observer %p is busy", obs);
			}
			all_ready &= !obs->is_busy;
		} else {
			busy = obs->handler(state, obs->context);
			(void) busy;
			__ASSERT(!busy, "Returning non-zero from these events is ignored");
		}
	}

	return !all_ready;
}

__weak void softdevice_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
	LOG_ERR("SoftDevice fault! ID %#x, PC %#x, Info %#x", id, pc, info);

	switch (id) {
	case NRF_FAULT_ID_SD_ASSERT:
		LOG_ERR("NRF_FAULT_ID_SD_ASSERT: SoftDevice assert");
		break;
	case NRF_FAULT_ID_APP_MEMACC:
		LOG_ERR("NRF_FAULT_ID_APP_MEMACC: Application bad memory access");
		if (info == 0x00) {
			LOG_ERR("Application tried to access SoftDevice RAM");
		} else {
			LOG_ERR("Application tried to access SoftDevice peripheral at %#x", info);
		}
		break;
	}

	for (;;) {
		/* loop */
	}
}

uint8_t nrf_get_lfclk_accuracy(uint32_t ppm)
{
	if (ppm < 2)
	{
		return NRF_CLOCK_LF_ACCURACY_1_PPM;
	}
	else if (ppm < 5)
	{
		return NRF_CLOCK_LF_ACCURACY_2_PPM;
	}
	else if (ppm < 10)
	{
		return NRF_CLOCK_LF_ACCURACY_5_PPM;
	}
	else if (ppm < 20)
	{
		return NRF_CLOCK_LF_ACCURACY_10_PPM;
	}
	else if (ppm < 30)
	{
		return NRF_CLOCK_LF_ACCURACY_20_PPM;
	}
	else if (ppm < 50)
	{
		return NRF_CLOCK_LF_ACCURACY_30_PPM;
	}
	else if (ppm < 75)
	{
		return NRF_CLOCK_LF_ACCURACY_50_PPM;
	}
	else if (ppm < 100)
	{
		return NRF_CLOCK_LF_ACCURACY_75_PPM;
	}
	else if (ppm < 150)
	{
		return NRF_CLOCK_LF_ACCURACY_100_PPM;
	}
	else if (ppm < 250)
	{
		return NRF_CLOCK_LF_ACCURACY_150_PPM;
	}
	else if (ppm < 500)
	{
		return NRF_CLOCK_LF_ACCURACY_250_PPM;
	}
	else
	{
		return NRF_CLOCK_LF_ACCURACY_500_PPM;
	}
}

static int nrf_sdh_enable(void)
{
	int err;
	OscDesc_t const *lfosc = GetLowFreqOscDesc();

	nrf_clock_lf_cfg_t clock_lf_cfg = {
		.source = lfosc->Type, //CONFIG_NRF_SDH_CLOCK_LF_SRC,
		.rc_ctiv = lfosc->Type != OSC_TYPE_RC ? 0 : CONFIG_NRF_SDH_CLOCK_LF_RC_CTIV,
		.rc_temp_ctiv = lfosc->Type != OSC_TYPE_RC ? 0 : CONFIG_NRF_SDH_CLOCK_LF_RC_TEMP_CTIV,
		.accuracy = nrf_get_lfclk_accuracy(lfosc->Accuracy), //CONFIG_NRF_SDH_CLOCK_LF_ACCURACY,
		.hfclk_latency = CONFIG_NRF_SDH_CLOCK_HFCLK_LATENCY,
		.hfint_ctiv = CONFIG_NRF_SDH_CLOCK_HFINT_CALIBRATION_INTERVAL,
	};

	err = sd_softdevice_enable(&clock_lf_cfg, softdevice_fault_handler);
	if (err) {
		LOG_ERR("Failed to enable SoftDevice, nrf_error %#x", err);
		return -EINVAL;
	}

	atomic_set(&sdh_is_suspended, false);
	atomic_set(&sdh_transition, false);

#if 0
#if defined(CONFIG_NRF_SDH_DISPATCH_MODEL_SCHED)
	/* Upon enabling the SoftDevice events IRQ, the SoftDevice will request a rand seed.
	 * When the events are dispatched by the scheduler, it won't be possible to
	 * enable Bluetooth until that event has been processed (in the main() loop).
	 * To avoid this, we seed the SoftDevice before enabling the interrupt so
	 * that no event is generated, and the application can complete the BLE
	 * initialization before reaching the main() loop.
	 */
	BUILD_ASSERT(IS_ENABLED(CONFIG_NRF_SDH_SOC_RAND_SEED));
	extern void sdh_soc_rand_seed(uint32_t evt, void *ctx);
	(void) sdh_soc_rand_seed(NRF_EVT_RAND_SEED_REQUEST, NULL);
#endif /* CONFIG_NRF_SDH_DISPATCH_MODEL_SCHED */
#endif

	/* Enable event interrupt, the priority has already been set by the stack. */
	NVIC_EnableIRQ((IRQn_Type)SD_EVT_IRQn);

	(void)sdh_state_evt_observer_notify(NRF_SDH_STATE_EVT_ENABLED);

	return 0;
}

static int nrf_sdh_disable(void)
{
	int err;

	err = sd_softdevice_disable();
	if (err) {
		LOG_ERR("Failed to disable SoftDevice, nrf_error %#x", err);
		return -EINVAL;
	}

	atomic_set(&sdh_transition, false);

	NVIC_DisableIRQ((IRQn_Type)SD_EVT_IRQn);

	(void)sdh_state_evt_observer_notify(NRF_SDH_STATE_EVT_DISABLED);

	return 0;
}

int nrf_sdh_enable_request(void)
{
	bool busy;
	uint8_t enabled;

	/* Handle warm reset (debugger, watchdog): SRAM is retained and
	 * the SD's "enabled" flag from the previous session survives.
	 * Zero the SD's static+dynamic RAM region to clear stale state
	 * instead of calling sd_softdevice_disable() which tears down
	 * GRTC and other peripherals we can't fully reconstruct.
	 *
	 * SD RAM layout (from DTS):
	 *   Static:  0x20000000 - 0x2000177F  (6K)
	 *   Dynamic: 0x20001780 - 0x2000477F  (12K)
	 */
	memset((void *)0x20000000, 0, 0x4780);

	/* 2. Set SD base address so SVC forwarding knows where to jump.
	 *    Do NOT call CallSoftDeviceResetHandler() here — it makes
	 *    sd_softdevice_is_enabled() return true, which causes
	 *    nrf_sdh_enable_request() to skip sd_softdevice_enable().
	 *    The reset handler is called in nrf_sdh_enable() instead. */
	softdevice_vector_forward_address = FIXED_PARTITION_OFFSET(softdevice_partition);
#ifdef CONFIG_BOOTLOADER_MCUBOOT
	softdevice_vector_forward_address += CONFIG_ROM_START_OFFSET;
#endif


	(void)sd_softdevice_is_enabled(&enabled);
	if (enabled) {
		return -EALREADY;
	}

	if (sdh_transition) {
		return -EINPROGRESS;
	}

	atomic_set(&sdh_transition, true);
	/* Assume all observers to be busy */
	TYPE_SECTION_FOREACH(struct nrf_sdh_state_evt_observer,
			     nrf_sdh_state_evt_observers, obs) {
		obs->is_busy = true;
	}

	busy = sdh_state_evt_observer_notify(NRF_SDH_STATE_EVT_ENABLE_PREPARE);
	if (busy) {
		/* Leave sdh_transition to 1, so process can be continued */
		return -EBUSY;
	}

	return nrf_sdh_enable();
}

int nrf_sdh_disable_request(void)
{
	bool busy;
	uint8_t enabled;

	(void)sd_softdevice_is_enabled(&enabled);
	if (!enabled) {
		return -EALREADY;
	}

	if (sdh_transition) {
		return -EINPROGRESS;
	}

	atomic_set(&sdh_transition, true);

	/* Assume all observers to be busy */
	TYPE_SECTION_FOREACH(struct nrf_sdh_state_evt_observer,
			     nrf_sdh_state_evt_observers, obs) {
		obs->is_busy = true;
	}

	busy = sdh_state_evt_observer_notify(NRF_SDH_STATE_EVT_DISABLE_PREPARE);
	if (busy) {
		/* Leave sdh_transition to 1, so process can be continued */
		return -EBUSY;
	}

	return nrf_sdh_disable();
}

int nrf_sdh_observer_ready(struct nrf_sdh_state_evt_observer *obs)
{
	int err;
	bool busy;
	uint8_t enabled;

	if (!obs) {
		return -EFAULT;
	}
	if (!sdh_transition) {
		return -EPERM;
	}
	if (!obs->is_busy) {
		LOG_WRN("Observer %p is not busy", obs);
		return 0;
	}

	obs->is_busy = false;

	(void)sd_softdevice_is_enabled(&enabled);

	busy = sdh_state_evt_observer_notify(enabled ? NRF_SDH_STATE_EVT_DISABLE_PREPARE
						     : NRF_SDH_STATE_EVT_ENABLE_PREPARE);

	/* Another observer needs to ready itself */
	if (busy) {
		return 0;
	}

	if (enabled) {
		err = nrf_sdh_disable();
	} else {
		err = nrf_sdh_enable();
	}

	__ASSERT(!err, "Failed to change SoftDevice state");
	(void) err;

	return 0;
}

void nrf_sdh_suspend(void)
{
	uint8_t sd_is_enabled;

	(void)sd_softdevice_is_enabled(&sd_is_enabled);

	if (!sd_is_enabled) {
		LOG_WRN("Tried to suspend, but SoftDevice is disabled");
		return;
	}
	if (sdh_is_suspended) {
		LOG_WRN("Tried to suspend, but already is suspended");
		return;
	}

	NVIC_DisableIRQ((IRQn_Type)SD_EVT_IRQn);

	atomic_set(&sdh_is_suspended, true);
}

void nrf_sdh_resume(void)
{
	uint8_t sd_is_enabled;

	(void)sd_softdevice_is_enabled(&sd_is_enabled);

	if (!sd_is_enabled) {
		LOG_WRN("Tried to resume, but SoftDevice is disabled");
		return;
	}
	if (!sdh_is_suspended) {
		LOG_WRN("Tried to resume, but not suspended");
		return;
	}

	/* Force calling ISR again to make sure we dispatch pending events */
	NVIC_SetPendingIRQ((IRQn_Type)SD_EVT_IRQn);
	NVIC_EnableIRQ((IRQn_Type)SD_EVT_IRQn);

	atomic_set(&sdh_is_suspended, false);
}

bool nrf_sdh_is_suspended(void)
{
	return sdh_is_suspended;
}

void nrf_sdh_evts_poll(void)
{
	/* Notify observers about pending SoftDevice event. */
	TYPE_SECTION_FOREACH(struct nrf_sdh_stack_evt_observer, nrf_sdh_stack_evt_observers, obs) {
		obs->handler(obs->context);
	}
}

#if defined(CONFIG_NRF_SDH_DISPATCH_MODEL_IRQ)

void SD_EVT_IRQHandler(void)
{
	nrf_sdh_evts_poll();
}

#elif defined(CONFIG_NRF_SDH_DISPATCH_MODEL_SCHED)

static void sdh_events_poll(void *data, size_t len)
{
	(void)data;
	(void)len;

	nrf_sdh_evts_poll();
}

void SD_EVT_IRQHandler(void)
{
	int err;

	err = bm_scheduler_defer(sdh_events_poll, NULL, 0);
	if (err) {
		LOG_WRN("Unable to schedule SoftDevice event, err %d", err);
	}
}

#elif defined(CONFIG_NRF_SDH_DISPATCH_MODEL_POLL)

#endif /* NRF_SDH_DISPATCH_MODEL */

/* ------------------------------------------------------------------
 * SVC_Handler — always forwards to SoftDevice.
 * (SVCs with SD numbers only issued after sd_softdevice_enable)
 * ------------------------------------------------------------------ */
__attribute__((naked))
void SVC_Handler(void)
{
	__asm volatile(
		"LDR  R0, =" SD_XSTR(NRF_SD_ISR_OFFSET_SVC) "        \n"
		"LDR  R1, =softdevice_vector_forward_address          \n"
		"LDR  R1, [R1]                                        \n"
		"LDR  R1, [R1, R0]                                    \n"
		"BX   R1                                              \n"
	);
}

__attribute__((naked))
void CLOCK_POWER_IRQHandler(void)
{
	__asm volatile(
		"LDR  R0, =" SD_XSTR(NRF_SD_ISR_OFFSET_CLOCK_POWER) "        \n"
		"LDR  R1, =softdevice_vector_forward_address          \n"
		"LDR  R1, [R1]                                        \n"
		"LDR  R1, [R1, R0]                                    \n"
		"BX   R1                                              \n"
	);
}

__attribute__((naked))
void SWI00_IRQHandler(void)
{
	__asm volatile(
		"LDR  R0, =" SD_XSTR(NRF_SD_ISR_OFFSET_SWI00) "        \n"
		"LDR  R1, =softdevice_vector_forward_address          \n"
		"LDR  R1, [R1]                                        \n"
		"LDR  R1, [R1, R0]                                    \n"
		"BX   R1                                              \n"
	);
}

__attribute__((naked))
void RADIO_0_IRQHandler(void)
{
	__asm volatile(
		"LDR  R0, =" SD_XSTR(NRF_SD_ISR_OFFSET_RADIO_0) "        \n"
		"LDR  R1, =softdevice_vector_forward_address          \n"
		"LDR  R1, [R1]                                        \n"
		"LDR  R1, [R1, R0]                                    \n"
		"BX   R1                                              \n"
	);
}

__attribute__((naked))
void TIMER10_IRQHandler(void)
{
	__asm volatile(
		"LDR  R0, =" SD_XSTR(NRF_SD_ISR_OFFSET_TIMER10) "        \n"
		"LDR  R1, =softdevice_vector_forward_address          \n"
		"LDR  R1, [R1]                                        \n"
		"LDR  R1, [R1, R0]                                    \n"
		"BX   R1                                              \n"
	);
}

__attribute__((naked))
void GRTC_3_IRQHandler(void)
{
	__asm volatile(
		"LDR  R0, =" SD_XSTR(NRF_SD_ISR_OFFSET_GRTC_3) "        \n"
		"LDR  R1, =softdevice_vector_forward_address          \n"
		"LDR  R1, [R1]                                        \n"
		"LDR  R1, [R1, R0]                                    \n"
		"BX   R1                                              \n"
	);
}

__attribute__((naked))
void ECB00_IRQHandler(void)
{
	__asm volatile(
		"LDR  R0, =" SD_XSTR(NRF_SD_ISR_OFFSET_ECB00) "        \n"
		"LDR  R1, =softdevice_vector_forward_address          \n"
		"LDR  R1, [R1]                                        \n"
		"LDR  R1, [R1, R0]                                    \n"
		"BX   R1                                              \n"
	);
}

__attribute__((naked))
void AAR00_CCM00_IRQHandler(void)
{
	__asm volatile(
		"LDR  R0, =" SD_XSTR(NRF_SD_ISR_OFFSET_AAR00_CCM00) "        \n"
		"LDR  R1, =softdevice_vector_forward_address          \n"
		"LDR  R1, [R1]                                        \n"
		"LDR  R1, [R1, R0]                                    \n"
		"BX   R1                                              \n"
	);
}

