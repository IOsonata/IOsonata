from pathlib import Path

app = Path('ARM/Nordic/nRF54/src/bt_app_bm.cpp')
text = app.read_text()
anchor = '''extern "C" void softdevice_fault_handler(uint32_t Id, uint32_t Pc, uint32_t Info)
'''
insert = '''extern "C" void BtAppDebugMarker(const char *pMsg)
{
	g_Uart.printf("%s\\r\\n", pMsg);
}

extern "C" void softdevice_fault_handler(uint32_t Id, uint32_t Pc, uint32_t Info)
'''
if anchor not in text:
    raise SystemExit('bt_app marker anchor missing')
text = text.replace(anchor, insert, 1)
app.write_text(text)

sdh = Path('ARM/Nordic/nRF54/src/nrf_sdh.c')
text = sdh.read_text()
anchor = '''#include "idelay.h"
'''
insert = '''#include "idelay.h"

extern void BtAppDebugMarker(const char *pMsg);
#define SDH_TRACE(Msg) BtAppDebugMarker(Msg)
'''
if anchor not in text:
    raise SystemExit('nrf_sdh include anchor missing')
text = text.replace(anchor, insert, 1)

repls = [
('''static int nrf_sdh_enable(void)
{
	int err;
''', '''static int nrf_sdh_enable(void)
{
	int err;
	SDH_TRACE("sdh: enable enter");
'''),
('''	err = sd_softdevice_enable(&clock_lf_cfg, softdevice_fault_handler);
	if (err) {
''', '''	SDH_TRACE("sdh: before sd_softdevice_enable");
	err = sd_softdevice_enable(&clock_lf_cfg, softdevice_fault_handler);
	SDH_TRACE("sdh: after sd_softdevice_enable");
	if (err) {
'''),
('''int nrf_sdh_enable_request(void)
{
	bool busy;
	uint8_t enabled;
''', '''int nrf_sdh_enable_request(void)
{
	bool busy;
	uint8_t enabled;
	SDH_TRACE("sdh: request enter");
'''),
('''	softdevice_vector_forward_address = FIXED_PARTITION_OFFSET(softdevice_partition);
#ifdef CONFIG_BOOTLOADER_MCUBOOT
''', '''	softdevice_vector_forward_address = FIXED_PARTITION_OFFSET(softdevice_partition);
	SDH_TRACE("sdh: vector base set");
#ifdef CONFIG_BOOTLOADER_MCUBOOT
'''),
('''	if (!softdevice_reset_done) {
		CallSoftDeviceResetHandler();
		softdevice_reset_done = true;
	}

	(void)sd_softdevice_is_enabled(&enabled);
''', '''	if (!softdevice_reset_done) {
		SDH_TRACE("sdh: before reset entry");
		CallSoftDeviceResetHandler();
		SDH_TRACE("sdh: after reset entry");
		softdevice_reset_done = true;
	}

	SDH_TRACE("sdh: before is_enabled");
	(void)sd_softdevice_is_enabled(&enabled);
	SDH_TRACE("sdh: after is_enabled");
'''),
('''	atomic_set(&sdh_transition, true);
	/* Assume all observers to be busy */
''', '''	atomic_set(&sdh_transition, true);
	SDH_TRACE("sdh: before observer init");
	/* Assume all observers to be busy */
'''),
('''	busy = sdh_state_evt_observer_notify(NRF_SDH_STATE_EVT_ENABLE_PREPARE);
	if (busy) {
''', '''	SDH_TRACE("sdh: before enable prepare");
	busy = sdh_state_evt_observer_notify(NRF_SDH_STATE_EVT_ENABLE_PREPARE);
	SDH_TRACE("sdh: after enable prepare");
	if (busy) {
'''),
('''	return nrf_sdh_enable();
}
''', '''	SDH_TRACE("sdh: before enable");
	return nrf_sdh_enable();
}
''')
]
for old, new in repls:
    if old not in text:
        raise SystemExit('nrf_sdh trace anchor missing: ' + old[:70])
    text = text.replace(old, new, 1)

sdh.write_text(text)
