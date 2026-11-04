#!/usr/bin/env python3
"""Check ESP32-C6 basic vectored INTMTX startup and shared driver selection."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
ESP = ROOT / "RISCV" / "Espressif"
C6 = ESP / "ESP32C" / "ESP32C6"
C3 = ESP / "ESP32C" / "ESP32C3"


class Esp32c6StartupTests(unittest.TestCase):
    def test_cpu_vectors_use_intmtx_not_clic(self):
        source = (C6 / "lib" / "src" / "Vectors_esp32c6.c").read_text()
        system = (C6 / "lib" / "src" / "system_esp32c6.c").read_text()
        linker = (ESP / "ESP32C" / "ldscript" / "gcc_esp32c6.ld").read_text()
        self.assertIn(".rept 32", source)
        self.assertIn("RISCV_TrapHandler", source)
        self.assertIn("ESP32C3_INTMTX_MAX_SOURCE           80U", source)
        self.assertIn("ESP32_INTMTX_INTR_STATUS_2_REG", source)
        self.assertIn("source = 64U + bit", source)
        self.assertIn("vt | 1UL", system)
        self.assertNotIn("vt | 0x3U", system)
        self.assertIn("ESP32_CPU_INT_ENABLE_REG = 0U", system)
        self.assertIn("KEEP(*(.iram.text.vectors))", linker)
        self.assertLess(linker.index("KEEP(*(.iram.text.vectors))"),
                        linker.index("KEEP(*(.iram.text))"))

    def test_library_links_real_gpio_uart_irq(self):
        path = C6 / "lib" / "ioc" / ".project"
        links = {
            element.findtext("name", ""): element.findtext("locationURI", "")
            for element in ET.parse(path).findall(".//linkedResources/link")
        }
        self.assertNotIn("src/Vectors_esp32_clic.c", links)
        self.assertIn("src/Vectors_esp32c6.c", links)
        for name, source in (
            ("src/coredev/esp32xx_irq.c", "esp32xx_irq.c"),
            ("src/coredev/iopincfg_esp32.c", "iopincfg_esp32.c"),
            ("src/coredev/system_esp32_uart_clock.c", "system_esp32_uart_clock.c"),
            ("src/coredev/uart_esp32.cpp", "uart_esp32.cpp"),
        ):
            with self.subTest(source=name):
                self.assertEqual(links.get(name),
                                 "PARENT-4-PROJECT_LOC/src/" + source)
                self.assertTrue((ESP / "src" / source).is_file())

    def test_c6_applications_pull_reset_entry_from_archive(self):
        shared = (ROOT / "RISCV" / "ldscript" / "gcc_riscv_flash.ld").read_text()
        self.assertIn("ENTRY(ResetEntry)", shared)
        for name in ("Blinky", "CryptoTest"):
            project = (C6 / "exemples" / name / "ioc" / ".cproject")
            tree = ET.parse(project)
            for cfg in tree.findall(".//cconfiguration"):
                config = cfg.find(".//configuration").get("name")
                flags = [
                    option.get("value", "")
                    for option in cfg.findall(".//option")
                    if option.get("name") == "Other linker flags"
                ]
                with self.subTest(name=name, configuration=config):
                    self.assertEqual(len(flags), 2)
                    self.assertTrue(all("-Wl,--undefined=ResetEntry" in x for x in flags))

    def test_cpu_local_interrupts_reserved_on_c6_only(self):
        uart = (ESP / "src" / "uart_esp32.cpp").read_text()
        pins = (ESP / "include" / "esp32xx_intmtx.h").read_text()
        self.assertIn("#if defined(ESP32C6)\n#define UART1_CPU_INT_LEVEL             5U", uart)
        self.assertIn("#  if defined(ESP32C6)\n#    define ESP32_GPIO_INTR_CPU_INT_ID      6U", pins)
        self.assertIn("#    define ESP32_GPIO_INTR_CPU_INT_ID      4U", pins)
        self.assertIn("#define UART1_CPU_INT_LEVEL             3U", uart)
        old = (C3 / "lib" / "src" / "Vectors_esp32c3.c").read_text()
        self.assertIn(".rept 32", old)


if __name__ == "__main__":
    unittest.main()
