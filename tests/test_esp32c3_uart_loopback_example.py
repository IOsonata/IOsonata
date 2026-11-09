#!/usr/bin/env python3
"""Checks for ESP32-C3 generic UART0 loopback managed example and host harness."""
from pathlib import Path
import runpy
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
EXAMPLE = ROOT / "RISCV/Espressif/ESP32C/ESP32C3/exemples/UartLoopback"
HOST = EXAMPLE / "tools/verify_uart_loopback.py"
GENERIC = ROOT / "exemples/uart/uart_loopback.cpp"


class EchoPort:
    def __init__(self, corrupt=False):
        self.received = bytearray()
        self.corrupt = corrupt

    def write(self, payload):
        wire = bytearray(payload)
        if self.corrupt:
            wire[0] ^= 0x80
        self.received.extend(wire)
        return len(payload)

    def read(self, length):
        # Deliberately fragment reads to exercise host accumulation.
        n = min(3, length, len(self.received))
        payload = bytes(self.received[:n])
        del self.received[:n]
        return payload

    def reset_input_buffer(self):
        self.received.clear()


class Esp32c3UartLoopbackTests(unittest.TestCase):
    def test_managed_eclipse_project(self):
        project = ET.parse(EXAMPLE / "ioc/.project").getroot()
        self.assertEqual(project.findtext("name"), "UartLoopback")
        links = {
            link.findtext("name"): link.findtext("locationURI")
            for link in project.findall(".//linkedResources/link")
        }
        self.assertEqual(links.get("src/uart_loopback.cpp"),
                         "PARENT-7-PROJECT_LOC/exemples/uart/uart_loopback.cpp")
        self.assertEqual(links.get("src/board.h"),
                         "PARENT-1-PROJECT_LOC/src/board.h")
        self.assertTrue(GENERIC.is_file())

        config = (EXAMPLE / "ioc/.cproject").read_text()
        ET.fromstring(config)
        self.assertIn("UartLoopback", config)
        self.assertNotIn("CryptoTest", config)
        self.assertIn('name="Debug"', config)
        self.assertIn('name="Release"', config)

    def test_uart0_board_and_generic_interrupt_path(self):
        board = (EXAMPLE / "src/board.h").read_text()
        for required in (
            "#define UART_DEVNO          0",
            "#define UART_RX_PIN         20",
            "#define UART_TX_PIN         21",
            "ESP32_PINOP_U0RXD",
            "ESP32_PINOP_U0TXD",
        ):
            self.assertIn(required, board)
        generic = GENERIC.read_text()
        self.assertIn("g_Uart.Rx(buff, len)", generic)
        self.assertIn("g_Uart.Tx(buff + offset, pending)", generic)
        self.assertIn("#define BUFFER_SIZE", generic)
        self.assertIn("#define UART_INT_MODE true", generic)
        self.assertIn("#define UART_DMA_MODE false", generic)

    def test_host_binary_round_trip_and_fragmented_reads(self):
        program = runpy.run_path(str(HOST), run_name="static_test")
        verify = program["verify_frame"]
        port = EchoPort()
        data = bytes(range(64)) + b"\x00\xff\x0d\x0a"
        self.assertEqual(verify(port, data, 0.1), len(data))
        self.assertEqual(port.received, b"")

        with self.assertRaises(program["LoopbackFailure"]):
            verify(EchoPort(corrupt=True), data, 0.1)

    def test_host_fails_on_missing_receive_data(self):
        program = runpy.run_path(str(HOST), run_name="static_test")
        class NoEcho(EchoPort):
            def write(self, payload):
                return len(payload)
        with self.assertRaises(program["LoopbackFailure"]):
            program["verify_frame"](NoEcho(), b"\x55", 0.01)


if __name__ == "__main__":
    unittest.main()
