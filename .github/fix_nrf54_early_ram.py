from pathlib import Path

sdh_path = Path('ARM/Nordic/nRF54/src/nrf_sdh.c')
sdh = sdh_path.read_text()

sdh = sdh.replace('#include <stdint.h>\n', '#include <stdint.h>\n#include <string.h>\n', 1)

old = '''extern void BtAppDebugMarker(const char *pMsg);
#define SDH_TRACE(Msg) BtAppDebugMarker(Msg)

'''
if old not in sdh:
    raise SystemExit('SDH trace declaration not found')
sdh = sdh.replace(old, '', 1)

for marker in [
    '\tSDH_TRACE("sdh: request enter");\n',
    '\tSDH_TRACE("sdh: before is_enabled");\n',
    '\tSDH_TRACE("sdh: after is_enabled");\n',
    '\tSDH_TRACE("sdh: before observer init");\n',
    '\tSDH_TRACE("sdh: before enable prepare");\n',
    '\tSDH_TRACE("sdh: after enable prepare");\n',
    '\tSDH_TRACE("sdh: before enable");\n',
    '\tSDH_TRACE("sdh: enable enter");\n',
    '\tSDH_TRACE("sdh: before sd_softdevice_enable");\n',
    '\tSDH_TRACE("sdh: after sd_softdevice_enable");\n',
    '\t\tSDH_TRACE("sdh: early reset missing");\n',
]:
    sdh = sdh.replace(marker, '')

old_early = '''void NrfSdhEarlyInit(void)
{
	if (softdevice_reset_done)
	{
		return;
	}

	softdevice_vector_forward_address = FIXED_PARTITION_OFFSET(softdevice_partition);
'''
new_early = '''void NrfSdhEarlyInit(void)
{
	if (softdevice_reset_done)
	{
		return;
	}

	/* The reset entry initializes the SoftDevice static and dynamic RAM.
	 * Clear that reserved range before calling it. Clearing the range later,
	 * immediately before the first SVC, destroys the state just initialized by
	 * the reset entry. */
	const uintptr_t app_ram_start = (uintptr_t)SystemRamStart();
	if (app_ram_start > 0x20000000UL)
	{
		memset((void *)0x20000000UL, 0,
			   app_ram_start - 0x20000000UL);
	}

	softdevice_vector_forward_address = FIXED_PARTITION_OFFSET(softdevice_partition);
'''
if old_early not in sdh:
    raise SystemExit('early init anchor not found')
sdh = sdh.replace(old_early, new_early, 1)

sdh_path.write_text(sdh)

app_path = Path('ARM/Nordic/nRF54/src/bt_app_bm.cpp')
app = app_path.read_text()

bridge = '''extern "C" void BtAppDebugMarker(const char *pMsg)
{
	g_Uart.printf("%s\\r\\n", pMsg);
}

'''
if bridge not in app:
    raise SystemExit('debug marker bridge not found')
app = app.replace(bridge, '', 1)

late_clear = '''	uint32_t ramstart = (uint32_t)SystemRamStart();

	memset((void *)0x20000000, 0, ramstart - 0x20000000);

	// GRTC3 must be enabled with interrupt disbled before calling nrf_sdh_enable_request
'''
replacement = '''	uint32_t ramstart = (uint32_t)SystemRamStart();

	/* NrfSdhEarlyInit already cleared the reserved SoftDevice RAM and then
	 * initialized it through the S145 reset entry. Do not erase it here. */

	// GRTC3 must be enabled with interrupt disbled before calling nrf_sdh_enable_request
'''
if late_clear not in app:
    raise SystemExit('late SoftDevice RAM clear not found')
app = app.replace(late_clear, replacement, 1)

app_path.write_text(app)
