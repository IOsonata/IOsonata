#!/usr/bin/env python3
"""Test the ST comparison application and real SVCCTL routing (CubeWBA v1.10.0)."""
from pathlib import Path
import argparse
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
parser = argparse.ArgumentParser()
parser.add_argument("cube", type=Path, help="Complete STM32CubeWBA v1.10.0 checkout")
args = parser.parse_args()
project = ROOT / "ARM/ST/STM32WBAxx/STM32WBA65/exemples/StThroughputClient"
wpan = args.cube / "Middlewares/ST/STM32_WPAN/ble"
sample = args.cube / "Projects/NUCLEO-WBA65RI/Applications/BLE/BLE_DataThroughput_Client"
source = (project / "src/prbs_client.c").read_text()
commands = sorted(set(re.findall(r"\b((?:aci_|hci_)[a-z_]+)\(", source)))
headers = "\n".join(f.read_text() for f in (wpan / "stack/include/auto").glob("*.h"))
with tempfile.TemporaryDirectory(prefix="st-comparison-") as tmp:
    temp = Path(tmp)
    mocks = {
        "app_conf.h": "#include \"cmsis_compiler.h\"\n#define CFG_BD_ADDRESS_TYPE 0\n",
        "app_common.h": "#include <stdint.h>\n#define CFG_TASK_DISCOVER_SERVICES_ID 2\n#define CFG_TASK_DATA_FROM_SERVER_ID 14\n#define CFG_SEQ_PRIO_0 0\n",
        "cmsis_compiler.h": "#include <stdint.h>\n#define __PACKED_STRUCT struct __attribute__((packed))\n#define __PACKED_UNION union __attribute__((packed))\n#define __STATIC_INLINE static inline\n",
        "stm32_seq.h": "#define UTIL_SEQ_RFU 0\nvoid UTIL_SEQ_SetTask(uint32_t, unsigned);\nvoid UTIL_SEQ_RegTask(uint32_t, unsigned, void (*)(void));\n",
        "stm32_timer.h": "typedef struct { int unused; } UTIL_TIMER_Object_t;\n#define UTIL_TIMER_OK 0\n#define UTIL_TIMER_PERIODIC 1\nuint32_t UTIL_TIMER_GetCurrentTime(void);\nint UTIL_TIMER_Create(UTIL_TIMER_Object_t *, unsigned, unsigned, void (*)(void *), void *);\nint UTIL_TIMER_Start(UTIL_TIMER_Object_t *);\n",
        "log_module.h": "void TestLog(const char *, ...);\n#define LOG_INFO_APP(...) TestLog(__VA_ARGS__)\n",
    }
    for name, content in mocks.items():
        (temp / name).write_text(content)
    code = '#include <assert.h>\n#include <stdarg.h>\n#include <stdio.h>\n'
    code += '#include "' + str(project / "src/prbs_client.c") + '"\n'
    code += 'static unsigned calls; static const char *lastCommand; static uint8_t reply; static int inEvent;\n'
    checks = {
        "aci_gap_create_connection": "assert(Conn_Interval_Min == 12 && Conn_Interval_Max == 12 && Conn_Latency == 0);",
        "hci_le_set_phy": "assert(ALL_PHYS == 0 && TX_PHYS == s_Phy && RX_PHYS == s_Phy);",
        "hci_le_set_data_length": "assert(TxOctets == 251 && TxTime == 2120);",
        "aci_gatt_write_char_value": "assert(Attr_Handle == 4 && Attribute_Val_Length == 2 && Attribute_Val[0] == 1 && Attribute_Val[1] == 0);",
    }
    for name in commands:
        match = re.search(r"tBleStatus " + name + r"\([^;]+\);", headers)
        assert match, name
        code += match.group(0)[:-1] + ' { ' + checks.get(name, '') + ' assert(!inEvent); calls++; lastCommand="' + name + '"; return reply; }\n'
    code += (ROOT / "tests/stm32wba/st_comparison_test.c").read_text()
    (temp / "test.c").write_text(code)
    native = (project / "src/app_ble.c").read_text()
    start = native.index("tBleStatus BLECB_Indication(")
    end = native.index("/* USER CODE BEGIN FD_WRAP_FUNCTIONS */", start)
    event_test = (ROOT / "tests/stm32wba/st_comparison_event_test.c").read_text()
    (temp / "event.c").write_text(event_test.replace("/* CALLBACK */", native[start:end]))
    for opt in ("-O0", "-O2"):
        exe = temp / "test"
        subprocess.run(["cc", "-std=c11", opt, "-g", "-fsanitize=undefined", "-fno-sanitize-recover=all",
                        "-include", str(project / "src/dk1_config.h"),
                        "-I" + str(temp), "-I" + str(wpan / "stack/include"),
                        "-I" + str(wpan / "svc/Inc"), "-I" + str(sample / "STM32_WPAN/App"),
                        str(temp / "test.c"), str(wpan / "svc/Src/svc_ctl.c"), "-o", str(exe)], check=True)
        subprocess.run([str(exe)], check=True)
        subprocess.run(["cc", "-std=c11", opt, "-g", "-fsanitize=undefined",
                        "-fno-sanitize-recover=all", str(temp / "event.c"), "-o", str(exe)], check=True)
        subprocess.run([str(exe)], check=True)
        print("ST comparison and split event copy", opt, "UBSan: PASS")
