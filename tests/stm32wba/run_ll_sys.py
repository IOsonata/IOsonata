#!/usr/bin/env python3
"""Compare the WBA link layer system hooks with the Cube ll_sys_if.c.

bt_wba_platform.cpp replaces the Cube BLE_p2pServer ll_sys_if.c. This test
builds both against the same link layer and sequencer mocks and compares the
calls they make, for each radio sleep timer clock selection (none, LSE, LSI,
HSE / 1000), with and without DEBUG. ST's file is read from the STM32CubeWBA
checkout and is not copied.

Set STM32CUBEWBA and STM32_WPAN to the checkouts when they are not in
../external.
"""
from pathlib import Path
import os
import re
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[2]


def checkout(env, name):
    for path in [os.environ.get(env), ROOT.parent / 'external' / name, ROOT.parent / name]:
        if path and Path(path).is_dir():
            return Path(path)
    sys.exit(name + ' checkout not found, set ' + env)


CUBE = checkout('STM32CUBEWBA', 'STM32CubeWBA')
WPAN = checkout('STM32_WPAN', 'stm32-mw-wpan')
TARGET = CUBE / 'Projects/NUCLEO-WBA65RI/Applications/BLE/BLE_p2pServer/STM32_WPAN/Target'
MODULES = CUBE / 'Projects/Common/WPAN/Modules'

platform = (ROOT / 'ARM/ST/STM32WBAxx/src/bt_wba_platform.cpp').read_text()
appconf = (ROOT / 'ARM/ST/STM32WBAxx/STM32WBA65/include/app_conf.h').read_text()
types = (WPAN / 'link_layer/ll_cmd_lib/inc/common_types.h').read_text()

# Task numbers from the IOsonata app_conf.h, sleep timer sources from the link
# layer header, temperature from the platform file
tasks = re.search(r'typedef enum \{(.*?)\} CFG_Task_Id_t;', appconf, re.S).group(1)
tasks = 'enum {' + tasks + '};\n'
slptmr = ''.join('#define %s %s\n' % m for m in
                 re.findall(r'\b(\w+_SLPTMR)\s*=\s*(0x[0-9a-fA-F]+)', types))
temp = ''.join(re.findall(r'#define BT_WBA_TEMPERATURE_\w+\s+\d+\n', platform))
assert 'RTC_SLPTMR' in slptmr and 'RCO_SLPTMR' in slptmr and temp.count('#define') == 1

start = platform.index('// Link layer scheduler margins')
end = platform.index('\n//----', start)
ours = platform[start:end]

conf = r'''
#ifndef APP_CONF_H
#define APP_CONF_H
#include <stdint.h>
#define STM32WBA65xx
#define USE_TEMPERATURE_BASED_RADIO_CALIBRATION 1
#define CFG_RF_TX_POWER_TABLE_ID 0
#define CFG_LPM_STANDBY_SUPPORTED 1
#define CFG_SEQ_PRIO_0 0
''' + tasks + slptmr + temp + '#endif\n'

mocks_h = r'''
#ifndef MOCKS_H
#define MOCKS_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
extern uint32_t g_Bdcr1;
void Log(const char *Fmt, ...);
uint8_t ll_intf_cmn_config_ll_ctx_params(uint8_t LowIsr, uint8_t FromIsr);
uint8_t ll_intf_cmn_le_select_slp_clk_src(uint8_t Src, uint16_t *pFreq);
void ll_intf_cmn_set_temperature_sensor_state(void);
uint8_t ll_intf_cmn_select_tx_power_table(uint8_t Id);
uint8_t ll_intf_cmn_set_temperature_value(uint32_t Temp);
uint8_t ll_intf_le_set_sleep_clock_accuracy(uint8_t Sca);
uint8_t ll_intf_le_set_rco_clbr_evnt_params(uint8_t Duration, uint32_t Interval);
uint32_t ll_sys_config_BLE_schldr_timings(uint8_t Drift, uint8_t Exec);
void ll_intf_apply_cte_degrad_change(void);
void ll_sys_bg_process(void);
void ll_sys_bg_process_init(void);
void ll_sys_schedule_bg_process(void);
void ll_sys_schedule_bg_process_isr(void);
void ll_sys_config_params(void);
void ll_sys_reset(void);
void ll_sys_bg_temperature_measurement(void);
void (*TestTask(unsigned Bit))(void);
#ifdef __cplusplus
}
#endif
#endif
'''

mocks_c = r'''
#include <stdarg.h>
#include <stdio.h>
#include "mocks.h"
uint32_t g_Bdcr1;
void Log(const char *Fmt, ...)
{
	va_list args;
	va_start(args, Fmt);
	vprintf(Fmt, args);
	va_end(args);
	putchar('\n');
}
uint8_t ll_intf_cmn_config_ll_ctx_params(uint8_t LowIsr, uint8_t FromIsr)
{ Log("config_ll_ctx_params %u %u", LowIsr, FromIsr); return 0; }
uint8_t ll_intf_cmn_le_select_slp_clk_src(uint8_t Src, uint16_t *pFreq)
{ Log("select_slp_clk_src %u", Src); *pFreq = 0; return 0; }
void ll_intf_cmn_set_temperature_sensor_state(void)
{
	// The link layer asks for a first temperature from here
	Log("set_temperature_sensor_state");
	ll_sys_bg_temperature_measurement();
}
uint8_t ll_intf_cmn_select_tx_power_table(uint8_t Id)
{ Log("select_tx_power_table %u", Id); return 0; }
uint8_t ll_intf_cmn_set_temperature_value(uint32_t Temp)
{ Log("set_temperature_value %u", (unsigned)Temp); return 0; }
uint8_t ll_intf_le_set_sleep_clock_accuracy(uint8_t Sca)
{ Log("set_sleep_clock_accuracy %u", Sca); return 0; }
uint8_t ll_intf_le_set_rco_clbr_evnt_params(uint8_t Duration, uint32_t Interval)
{ Log("set_rco_clbr_evnt_params %u %u", Duration, (unsigned)Interval); return 0; }
uint32_t ll_sys_config_BLE_schldr_timings(uint8_t Drift, uint8_t Exec)
{ Log("config_BLE_schldr_timings %u %u", Drift, Exec); return 0; }
void ll_intf_apply_cte_degrad_change(void) { Log("apply_cte_degrad_change"); }
void ll_sys_bg_process(void) { Log("bg_process"); }
'''

main_c = r'''
#include <stdlib.h>
#include "mocks.h"
int main(int argc, char **argv)
{
	unsigned i;
	g_Bdcr1 = (uint32_t)atoi(argv[1]) << 18;
	Log("init");
	ll_sys_bg_process_init();
	Log("config");
	ll_sys_config_params();
	Log("reset");
	ll_sys_reset();
	Log("schedule");
	ll_sys_schedule_bg_process();
	ll_sys_schedule_bg_process_isr();
	Log("temperature");
	ll_sys_bg_temperature_measurement();
	Log("tasks");
	for (i = 0; i < 32; i++)
	{
		if (TestTask(i) != 0)
		{
			Log("task %u", i);
			TestTask(i)();
		}
	}
	return 0;
}
'''

# Headers the Cube file includes, reduced to what it uses
st_headers = {
    'main.h': r'''
#include "app_conf.h"
#include "mocks.h"
#define LL_RCC_RADIOSLEEPSOURCE_NONE 0U
#define LL_RCC_RADIOSLEEPSOURCE_LSE (1UL << 18)
#define LL_RCC_RADIOSLEEPSOURCE_LSI (2UL << 18)
#define LL_RCC_RADIOSLEEPSOURCE_HSE_DIV1000 (3UL << 18)
#define LL_RCC_RADIO_GetSleepTimerClockSource() (g_Bdcr1 & (3UL << 18))
#define LL_DBGMCU_GetRevisionID() 0x1000U
#define assert_param(x) ((void)0)
#define UNUSED(x) ((void)(x))
''',
    'app_common.h': '#include "app_conf.h"\n',
    'log_module.h': '',
    'll_intf_cmn.h': '#include "mocks.h"\n',
    'll_sys.h': '#include "mocks.h"\n',
    'stm32_rtos.h': r'''
#define UTIL_SEQ_RFU 0
#define TASK_PRIO_LINK_LAYER 0
void UTIL_SEQ_RegTask(uint32_t TaskBm, uint32_t Flags, void (*Task)(void));
void UTIL_SEQ_SetTask(uint32_t TaskBm, uint32_t Prio);
''',
    'temp_measurement.h': 'void TEMPMEAS_RequestTemperatureMeasurement(void);\n',
}

# Sequencer and temperature as the port provided them to the Cube file
st_glue = r'''
#include "app_conf.h"
#include "mocks.h"
static void (*s_Task[32])(void);
void UTIL_SEQ_RegTask(uint32_t TaskBm, uint32_t Flags, void (*Task)(void))
{
	unsigned i;
	(void)Flags;
	for (i = 0; i < 32; i++)
		if (TaskBm & (1UL << i))
			s_Task[i] = Task;
}
void UTIL_SEQ_SetTask(uint32_t TaskBm, uint32_t Prio)
{
	(void)Prio;
	Log("SetTask 0x%x", (unsigned)TaskBm);
}
void TEMPMEAS_RequestTemperatureMeasurement(void)
{
	ll_intf_cmn_set_temperature_value(25 + BT_WBA_TEMPERATURE_OFS);
}
void (*TestTask(unsigned Bit))(void) { return s_Task[Bit]; }
'''

ours_cpp = r'''
#include <cstdlib>
#include "app_conf.h"
#include "mocks.h"
static struct { uint32_t BDCR1; } s_Rcc;
#define RCC (&s_Rcc)
#define RCC_BDCR1_RADIOSTSEL_Pos 18UL
#define RCC_BDCR1_RADIOSTSEL (3UL << 18)
#define RCC_BDCR1_RADIOSTSEL_1 (2UL << 18)
static void (*s_SeqTask[32])(void);
static void BtWbaSeqSet(uint32_t TaskBm)
{
	Log("SetTask 0x%x", (unsigned)TaskBm);
}
struct TempSensorCfg_t { int a, b, c, d, e, f, g, h; void *p; };
struct TempSensorData_t { int32_t Temperature; };
#define SENSOR_OPMODE_SINGLE 0
#define DEVINTR_POL_LOW 0
class TempSTM32WBA {
public:
    bool Init(const TempSensorCfg_t &) { return true; }
    bool UpdateData() { return std::getenv("WBA_TEMP_FAIL") == nullptr; }
    void Read(TempSensorData_t &data) { data.Temperature = 2500; }
};
static uint32_t BtWbaSleepClockSelect(void);
''' + ours.replace('uint32_t sel = RCC->BDCR1', 'RCC->BDCR1 = g_Bdcr1;\n\tuint32_t sel = RCC->BDCR1') + r'''
extern "C" void (*TestTask(unsigned Bit))(void) { return s_SeqTask[Bit]; }
'''

failed = False
with tempfile.TemporaryDirectory(prefix='wba-ll-sys-') as temp:
    t = Path(temp)
    inc = t / 'inc'
    inc.mkdir()
    (inc / 'app_conf.h').write_text(conf)
    (inc / 'mocks.h').write_text(mocks_h)
    for name, text in st_headers.items():
        (inc / name).write_text(text)
    (t / 'mocks.c').write_text(mocks_c)
    (t / 'main.c').write_text(main_c)
    (t / 'st_glue.c').write_text(st_glue)
    (t / 'ours.cpp').write_text(ours_cpp)
    cc = os.environ.get('CC', 'gcc')
    cxx = os.environ.get('CXX', 'g++')
    for debug in [[], ['-DDEBUG']]:
        common = ['-O2', '-Wall', '-I' + str(inc)] + debug
        st = t / 'st'
        mine = t / 'ours'
        objs = []
        for src in ['mocks.c', 'main.c']:
            obj = t / (src + '.o')
            subprocess.run([cc, '-std=gnu11'] + common + ['-c', str(t / src), '-o', str(obj)], check=True)
            objs.append(str(obj))
        subprocess.run([cc, '-std=gnu11'] + common + ['-I' + str(TARGET), '-I' + str(MODULES),
                        str(TARGET / 'll_sys_if.c'), str(t / 'st_glue.c')] + objs + ['-o', str(st)], check=True)
        subprocess.run([cxx, '-std=gnu++17', '-fsanitize=undefined', '-fno-sanitize-recover=all'] + common +
                       [str(t / 'ours.cpp')] + objs + ['-o', str(mine), '-fsanitize=undefined'], check=True)
        label = 'DEBUG' if debug else 'no DEBUG'
        for sel, name in enumerate(['none', 'LSE', 'LSI', 'HSE/1000']):
            a = subprocess.run([str(st), str(sel)], check=True, capture_output=True, text=True, timeout=10).stdout
            b = subprocess.run([str(mine), str(sel)], check=True, capture_output=True, text=True, timeout=10).stdout
            if a != b:
                failed = True
                print('FAIL', label, name)
                print('--- Cube ll_sys_if.c\n' + a + '--- bt_wba_platform.cpp\n' + b)
            else:
                print('PASS WBA ll_sys hooks:', label, name, 'sleep clock,', len(a.splitlines()), 'calls same as Cube')
        output = subprocess.run([str(mine), '1'], check=True, capture_output=True,
                                text=True, timeout=10,
                                env={**os.environ, 'WBA_TEMP_FAIL': '1'}).stdout
        assert 'set_temperature_sensor_state' not in output
        assert 'set_temperature_value' not in output
        print('PASS WBA temperature failure: periodic calibration retained, no fabricated reading')
sys.exit(1 if failed else 0)
