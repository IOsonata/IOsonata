#!/usr/bin/env python3
"""Host-check nRF71 SystemSleep and Nordic power source links."""
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
NRF71 = ROOT / 'ARM/Nordic/nRF71/src/system_sleep_nrf71.c'
MOCK = """
#include <stdint.h>
typedef struct { uint32_t SCR; } MockScb;
extern MockScb g_Scb;
#define SCB (&g_Scb)
#define SCB_SCR_SLEEPDEEP_Msk (1U << 2)
#define SCB_SCR_SLEEPONEXIT_Msk (1U << 1)
uint32_t MockIpsr(void);
void MockWfi(void);
#define __get_IPSR() MockIpsr()
#define __DSB() ((void)0)
#define __ISB() ((void)0)
#define __WFI() MockWfi()
"""
FIXTURE = """
#include <assert.h>
#include <stdint.h>
#include "nrf.h"
MockScb g_Scb;
static uint32_t s_Ipsr, s_Mask;
static int s_Wait;
void SystemSleep(void);
uint32_t MockIpsr(void) { return s_Ipsr; }
uint32_t DisableInterrupt(void) { uint32_t v = s_Mask; s_Mask = 1U; return v; }
void EnableInterrupt(uint32_t v) { s_Mask = v; }
void MockWfi(void)
{
    assert(s_Mask == 1U);
    assert((g_Scb.SCR & 6U) == 0U);
    s_Wait++;
}
int main(void)
{
    int i;
    for (i = 0; i < 2; i++)
    {
        s_Mask = (uint32_t)i;
        g_Scb.SCR = 0x106U;
        s_Wait = 0;
        SystemSleep();
        assert(s_Wait == 1 && g_Scb.SCR == 0x106U && s_Mask == (uint32_t)i);
        s_Ipsr = 1U;
        SystemSleep();
        assert(s_Wait == 1);
        s_Ipsr = 0U;
    }
}
"""


def main():
    for name in ('ARM/Nordic/nRF52/src/system_power_nrf52.cpp',
                 'ARM/Nordic/src/system_power_nrfx.cpp'):
        source = (ROOT / name).read_text()
        assert 'void SystemSleep(void)' in source, name
        assert 'SCB_SCR_SLEEPONEXIT_Msk' in source, name
    project = ROOT / 'ARM/Nordic/nRF71/nRF7120x/lib/ioc/.project'
    links = ET.parse(project).findall('.//link')
    assert sum(link.findtext('name') == 'src/system_sleep_nrf71.c'
               for link in links) == 1
    with tempfile.TemporaryDirectory() as temp:
        path = Path(temp)
        (path / 'coredev').mkdir()
        (path / 'nrf.h').write_text(MOCK)
        (path / 'coredev/interrupt.h').write_text(
            '#include <stdint.h>\nuint32_t DisableInterrupt(void);\n'
            'void EnableInterrupt(uint32_t);\n')
        (path / 'coredev/system_core_clock.h').write_text('void SystemSleep(void);\n')
        (path / 'test.c').write_text(FIXTURE)
        output = path / 'sleep_test'
        subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                        '-I', str(path), str(NRF71), str(path / 'test.c'),
                        '-o', str(output)], check=True)
        subprocess.run([str(output)], check=True)
    print('Nordic SystemSleep host and Eclipse-link checks PASS')


if __name__ == '__main__':
    main()
