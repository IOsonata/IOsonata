#!/usr/bin/env python3
"""Test the ST comparison application and real SVCCTL routing (CubeWBA v1.10.0)."""
from pathlib import Path
import argparse
import re
import subprocess
import tempfile
import xml.etree.ElementTree as ET

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
        "stm32_adv_trace.h": "#define UTIL_ADV_TRACE_OK 0\nint UTIL_ADV_TRACE_Send(const uint8_t *, uint16_t);\n",
        "log_module.h": "void TestLog(const char *, ...);\n#define LOG_INFO_APP(...) TestLog(__VA_ARGS__)\n",
    }
    for name, content in mocks.items():
        (temp / name).write_text(content)
    # Use the actual managed-project feature defines: these select real SDK
    # implementations versus 0xff stubs independently of the linked archive.
    project_xml = ET.parse(project / "ioc/.cproject")
    for config in project_xml.findall("./storageModule/cconfiguration"):
        for tool in config.findall(".//tool"):
            definitions = [value.get("value") for option in tool.findall("option")
                           if option.get("valueType") == "definedSymbols"
                           for value in option.findall("listOptionValue")]
            if not definitions:
                continue
            variants = [value for value in definitions if value.startswith("BLE_STACK_")]
            assert variants == ["BLE_STACK_BASIC_PLUS=1"], variants
            preprocessed = subprocess.check_output([
                "cc", "-E", "-P", "-I" + str(temp), "-I" + str(wpan / "stack/include"),
                *["-D" + value for value in variants],
                str(wpan / "stack/include/auto/ble_wrap.c")], text=True)
            for name in ("GAP_Ext_Scan_Parameters", "GAP_Ext_Scan_Enable", "GAP_Ext_Connect"):
                assert not re.search(r"int " + name + r"\([^)]*\)\s*\{", preprocessed), name
    print("Managed Debug/Release SDK wrappers: extended scan/connect stubs absent")
    code = '#include <assert.h>\n#include <stdarg.h>\n#include <stdio.h>\n'
    code += '#include "' + str(project / "src/prbs_client.c") + '"\n'
    code += 'static uint32_t clockMs; static unsigned commandDelay; static unsigned calls; static const char *lastCommand; static uint8_t reply; static int inEvent;\n'
    checks = {
        "aci_gap_start_scan": "assert(Procedure == GAP_OBSERVATION_PROC && Filter_Duplicates == 1);",
        "aci_gap_terminate_gap_proc": "assert(Procedure_Code == GAP_OBSERVATION_PROC || Procedure_Code == GAP_DIRECT_CONNECTION_ESTABLISHMENT_PROC);",
        "aci_gap_create_connection": "assert(Conn_Interval_Min == 12 && Conn_Interval_Max == 12 && Conn_Latency == 0);",
        "hci_le_set_phy": "assert(ALL_PHYS == 0 && TX_PHYS == s_Phy && RX_PHYS == s_Phy);",
        "hci_le_set_data_length": "assert(TxOctets == 251 && TxTime == 2120);",
        "aci_gatt_disc_all_char_desc": "assert(Char_Handle == 3 && End_Handle == 9);",
        "aci_gap_terminate": "clockMs += commandDelay;",
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
    # Compile both real host initializers against the same SDK and compare every
    # configuration field, normalizing only addresses of their static buffers.
    ios = (ROOT / "ARM/ST/STM32WBAxx/src/bt_app_stm32wba.cpp").read_text()
    cfg = '#include <assert.h>\n#include <string.h>\n#include "ble_core.h"\n#include "ble_bufsize.h"\n#include "blestack.h"\n'
    cfg += '#define PLACE_IN_SECTION(x)\n#define CFG_BLE_NVM_MAX_SIZE 384\n#define DIVC(x, y) (((x) + (y) - 1) / (y))\n#define MAX(x, y) ((x) > (y) ? (x) : (y))\n'
    cfg += native[native.index("#define BLE_GATT_BUF_SIZE"):native.index("#define BLE_DEFAULT_PIN")]
    cfg += native[native.index("static BleStack_init_t pInitParams;"):native.index("/* USER CODE BEGIN PV */")]
    cfg += 'static BleStack_init_t reference; static unsigned initCalls;\n'
    cfg += 'tBleStatus BleStack_Init(const BleStack_init_t *p) { BleStack_init_t v = *p;\n'
    cfg += 'v.bleStartRamAddress = NULL; v.bleStartRamAddress_GATT = NULL; v.nvm_cache_buffer = NULL; v.host_event_fifo_buffer = NULL;\n'
    cfg += 'if (initCalls++ == 0) reference = v; else assert(memcmp(&reference, &v, sizeof(v)) == 0); return 0; }\n'
    start = native.index("static uint8_t HOST_BLE_Init(void)\n{")
    cfg += native[start:native.index("static void Ble_Hci_Gap_Gatt_Init(void)\n{", start)]
    defaults = ios[ios.index("#ifndef CFG_BLE_NUM_LINK"):ios.index("// The RAM the stack runs in")]
    cfg += "".join("#undef " + name + "\n" for name in re.findall(r"#ifndef (CFG_BLE_\w+)", defaults))
    cfg += defaults
    cfg += ios[ios.index("static uint32_t s_BleStackRam"):ios.index("// A strong reference must pull")]
    # Compile the complete deinitializer too, so removed buffers cannot leave
    # stale references outside HOST_BLE_Init.
    cfg += 'typedef struct { int unused; } tListNode; static tListNode BleAsynchEventQueue;\n'
    cfg += '#define TRUE 1\n#define aci_reset(a,b) ((void)0)\n#define BAES_Reset() ((void)0)\n#define PKACTRL_Reset() ((void)0)\n#define BLE_TIMER_Deinit() ((void)0)\n'
    cfg += '#define LST_is_empty(q) 1\n#define LST_remove_tail(q,p) (*(p) = NULL)\n#define AMM_Free(p) 0\n'
    start = native.index("void APP_BLE_Deinit(void)\n{")
    cfg += native[start:native.index("SVCCTL_UserEvtFlowStatus_t SVCCTL_App_Notification", start)]
    start = ios.index("BleStack_init_t init;")
    cfg += 'static void TestIosInit(void) {\n' + ios[start:ios.index("BtWbaPlatformInit();", start)]
    cfg += 'assert(BleStack_Init(&init) == 0); }\n'
    cfg += 'int main(void) { TestIosInit(); assert(HOST_BLE_Init() == 0); assert(initCalls == 2); APP_BLE_Deinit(); return 0; }\n'
    (temp / "config.c").write_text(cfg)
    for opt in ("-O0", "-O2"):
        exe = temp / "test"
        subprocess.run(["cc", "-std=c11", opt, "-g", "-fsanitize=undefined", "-fno-sanitize-recover=all",
                        "-include", str(project / "src/dk1_config.h"),
                        "-I" + str(temp), "-I" + str(wpan / "stack/include"),
                        "-I" + str(wpan / "svc/Inc"), "-I" + str(sample / "STM32_WPAN/App"),
                        "-I" + str(ROOT / "include"), str(ROOT / "src/cfifo.c"),
                        str(temp / "test.c"), str(wpan / "svc/Src/svc_ctl.c"), "-o", str(exe)], check=True)
        subprocess.run([str(exe)], check=True)
        subprocess.run(["cc", "-std=c11", opt, "-g", "-fsanitize=undefined",
                        "-fno-sanitize-recover=all", str(temp / "event.c"), "-o", str(exe)], check=True)
        subprocess.run([str(exe)], check=True)
        subprocess.run(["cc", "-std=c11", opt, "-g", "-fsanitize=undefined",
                        "-fno-sanitize-recover=all", "-include", str(project / "src/dk1_config.h"),
                        "-I" + str(temp), "-I" + str(wpan / "stack/include"),
                        "-I" + str(wpan / "svc/Inc"), "-I" + str(sample / "STM32_WPAN/App"),
                        str(temp / "config.c"), "-o", str(exe)], check=True)
        subprocess.run([str(exe)], check=True)
        print("ST comparison, split event copy and matching host configuration", opt, "UBSan: PASS")
