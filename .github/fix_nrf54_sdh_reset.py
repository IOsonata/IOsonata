from pathlib import Path

path = Path('ARM/Nordic/nRF54/src/nrf_sdh.c')
text = path.read_text()

old = '''uint32_t softdevice_vector_forward_address;


static atomic_t sdh_is_suspended;'''
new = '''uint32_t softdevice_vector_forward_address;
static bool softdevice_reset_done;

static void CallSoftDeviceResetHandler(void)
{
	const uint32_t handler_addr = *(const uint32_t *)(
		softdevice_vector_forward_address + NRF_SD_ISR_OFFSET_RESET);
	void (*handler)(void) = (void (*)(void))handler_addr;

	handler();
}

static atomic_t sdh_is_suspended;'''
if old not in text:
    raise SystemExit('global insertion anchor not found')
text = text.replace(old, new, 1)

old = '''	/* 2. Set SD base address so SVC forwarding knows where to jump.
	 *    Do NOT call CallSoftDeviceResetHandler() here — it makes
	 *    sd_softdevice_is_enabled() return true, which causes
	 *    nrf_sdh_enable_request() to skip sd_softdevice_enable().
	 *    The reset handler is called in nrf_sdh_enable() instead. */
	softdevice_vector_forward_address = FIXED_PARTITION_OFFSET(softdevice_partition);
#ifdef CONFIG_BOOTLOADER_MCUBOOT
	softdevice_vector_forward_address += CONFIG_ROM_START_OFFSET;
#endif


	(void)sd_softdevice_is_enabled(&enabled);'''
new = '''	/* Set the SoftDevice vector base before issuing any SoftDevice SVC.
	 * The nRF54 SoftDevice reset entry initializes its RAM state and must be
	 * called once after every application reset. The sdk-nrf-bm irq_connect
	 * implementation performs the same step during system initialization. */
	softdevice_vector_forward_address = FIXED_PARTITION_OFFSET(softdevice_partition);
#ifdef CONFIG_BOOTLOADER_MCUBOOT
	softdevice_vector_forward_address += CONFIG_ROM_START_OFFSET;
#endif

	if (!softdevice_reset_done) {
		CallSoftDeviceResetHandler();
		softdevice_reset_done = true;
	}

	(void)sd_softdevice_is_enabled(&enabled);'''
if old not in text:
    raise SystemExit('enable sequence anchor not found')
text = text.replace(old, new, 1)

path.write_text(text)
