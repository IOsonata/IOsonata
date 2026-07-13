from pathlib import Path

sdh_path = Path('ARM/Nordic/nRF54/src/nrf_sdh.c')
sys_path = Path('ARM/Nordic/nRF54/src/system_nrf54l.c')

sdh = sdh_path.read_text()
sys = sys_path.read_text()

old = '''static void CallSoftDeviceResetHandler(void)
{
	const uint32_t handler_addr = *(const uint32_t *)(
		softdevice_vector_forward_address + NRF_SD_ISR_OFFSET_RESET);
	void (*handler)(void) = (void (*)(void))handler_addr;

	handler();
}
'''
new = '''static void CallSoftDeviceResetHandler(void)
{
	const uint32_t handler_addr = *(const uint32_t *)(
		softdevice_vector_forward_address + NRF_SD_ISR_OFFSET_RESET);
	void (*handler)(void) = (void (*)(void))handler_addr;

	handler();
}

void NrfSdhEarlyInit(void)
{
	if (softdevice_reset_done)
	{
		return;
	}

	softdevice_vector_forward_address = FIXED_PARTITION_OFFSET(softdevice_partition);
#ifdef CONFIG_BOOTLOADER_MCUBOOT
	softdevice_vector_forward_address += CONFIG_ROM_START_OFFSET;
#endif

	CallSoftDeviceResetHandler();
	softdevice_reset_done = true;
}
'''
if old not in sdh:
    raise SystemExit('reset handler anchor not found')
sdh = sdh.replace(old, new, 1)

old = '''	/* Set the SoftDevice vector base before issuing any SoftDevice SVC.
	 * The nRF54 SoftDevice reset entry initializes its RAM state and must be
	 * called once after every application reset. The sdk-nrf-bm irq_connect
	 * implementation performs the same step during system initialization. */
	softdevice_vector_forward_address = FIXED_PARTITION_OFFSET(softdevice_partition);
	SDH_TRACE("sdh: vector base set");
#ifdef CONFIG_BOOTLOADER_MCUBOOT
	softdevice_vector_forward_address += CONFIG_ROM_START_OFFSET;
#endif

	if (!softdevice_reset_done) {
		SDH_TRACE("sdh: before reset entry");
		CallSoftDeviceResetHandler();
		SDH_TRACE("sdh: after reset entry");
		softdevice_reset_done = true;
	}

	SDH_TRACE("sdh: before is_enabled");
'''
new = '''	/* The reset entry must run during early system startup, before UART,
	 * timers and other application peripherals are initialized. */
	if (!softdevice_reset_done)
	{
		SDH_TRACE("sdh: early reset missing");
		return -EPERM;
	}

	SDH_TRACE("sdh: before is_enabled");
'''
if old not in sdh:
    raise SystemExit('late reset block not found')
sdh = sdh.replace(old, new, 1)

inc_anchor = '''#include "coredev/system_core_clock.h"
'''
inc_new = '''#include "coredev/system_core_clock.h"

#if defined(S145)
extern void NrfSdhEarlyInit(void);
#endif
'''
if inc_anchor not in sys:
    raise SystemExit('system include anchor not found')
sys = sys.replace(inc_anchor, inc_new, 1)

init_anchor = '''void SystemInit(void)
{
    #ifdef __CORTEX_M
'''
init_new = '''void SystemInit(void)
{
    #if defined(S145)
        /* Initialize the SoftDevice image before clocks, UART, timers or any
         * other application peripheral. The reset entry may change peripheral
         * and clock state, so it must not be called later from BtAppInit. */
        NrfSdhEarlyInit();
    #endif

    #ifdef __CORTEX_M
'''
if init_anchor not in sys:
    raise SystemExit('SystemInit anchor not found')
sys = sys.replace(init_anchor, init_new, 1)

sdh_path.write_text(sdh)
sys_path.write_text(sys)
