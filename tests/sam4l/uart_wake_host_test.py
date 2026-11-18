#!/usr/bin/env python3
"""Simulated-serial acceptance tests for the SAM4L UART wake helper."""

import importlib.util
import io
import pathlib
import sys
import types
import unittest
from unittest import mock

root = pathlib.Path(__file__).resolve().parents[2]
script = root / "Python/sam4l_uart_wake.py"

serial_stub = types.ModuleType("serial")
serial_stub.SerialException = type("SerialException", (Exception,), {})
serial_stub.SerialTimeoutException = type("SerialTimeoutException", (Exception,), {})
with mock.patch.dict(sys.modules, {"serial": serial_stub}):
    spec = importlib.util.spec_from_file_location("sam4l_uart_wake_under_test", script)
    wake = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(wake)


class FakeClock:
    def __init__(self):
        self.value = 0.0
        self.delays = []

    def monotonic(self):
        self.value += 0.02
        return self.value

    def sleep(self, seconds):
        self.delays.append(seconds)
        self.value += seconds


class FakePort:
    def __init__(self, scenario):
        self.scenario = scenario
        self.baudrate = 1000000
        self.out_waiting = 0
        self.wakes = 0
        self.queue = [
            b"PowerOffDemo: Standby\r\n",
            b"UART RX: send U at 1000000 baud\r\n",
        ]

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        return False

    @property
    def in_waiting(self):
        return 0

    def read_until(self, delimiter):
        if self.queue:
            return self.queue.pop(0)
        return b""

    def write(self, data):
        if data == b"U" and self.baudrate == 1000000:
            self.queue.extend([
                b"UART RX normal: PASS\r\n",
                b"UART wake setup: send R at 1000000 baud\r\n",
            ])
        elif data == b"R" and self.baudrate == 1000000:
            self.queue.append(b"UART sleep preparation: switch host to 9600 baud\r\n")
        elif data == b"U" and self.baudrate == 9600:
            self.wakes += 1
            self.queue.extend([
                b"UART wake byte: PASS\r\n",
                b"UART rate after resume: PASS\r\n",
                b"CPU/APB clock after resume: PASS\r\n",
            ])
            count = self.wakes - 1 if self.scenario == "bad_counter" else self.wakes
            self.queue.append(
                ("Resumed from Standby, count 0x%08X\r\n" % count).encode("ascii")
            )
            if self.scenario == "ram_fail":
                self.queue.append(b"SRAM sample FAIL\r\n")
            elif self.scenario != "ram_missing":
                self.queue.append(b"SRAM sample PASS: 128 static + 128 stack bytes\r\n")
            if self.scenario == "success" and self.wakes < 2:
                self.queue.append(b"UART wake setup: send R at 1000000 baud\r\n")
        else:
            raise AssertionError("Unexpected %r at baud %d" % (data, self.baudrate))
        return len(data)


class UARTWakeHostTests(unittest.TestCase):
    def run_scenario(self, scenario, cycles, wake_delay=None):
        port = FakePort(scenario)
        clock = FakeClock()
        output = io.BytesIO()
        console = io.TextIOWrapper(output, encoding="utf-8", write_through=True)
        errors = io.StringIO()
        argv = [
            str(script), "--port", "fake", "--baud", "1000000",
            "--cycles", str(cycles), "--timeout", "2", "--total-timeout", "20",
        ]
        if wake_delay is not None:
            argv.extend(("--wake-delay", str(wake_delay)))
        with mock.patch.object(wake.serial, "Serial", return_value=port, create=True), \
                mock.patch.object(wake.time, "monotonic", clock.monotonic), \
                mock.patch.object(wake.time, "sleep", clock.sleep), \
                mock.patch.object(sys, "argv", argv), \
                mock.patch.object(sys, "stdout", console), \
                mock.patch.object(sys, "stderr", errors):
            code = wake.main()
            console.flush()
            result = output.getvalue().decode("utf-8")
        port.sleep_delays = clock.delays
        return code, result, errors.getvalue(), port

    def test_two_complete_cycles(self):
        code, output, errors, port = self.run_scenario("success", 2)
        self.assertEqual(code, 0, errors)
        self.assertIn("UART wake cycle 1/2 PASS, including SRAM.", output)
        self.assertIn("UART wake cycle 2/2 PASS, including SRAM.", output)
        self.assertEqual(port.wakes, 2)

    def test_sram_failure_rejects_cycle(self):
        code, output, _, port = self.run_scenario("ram_fail", 1)
        self.assertEqual(code, 1)
        self.assertNotIn("UART wake cycle 1/1 PASS", output)
        self.assertEqual(port.wakes, 1)

    def test_missing_sram_result_times_out(self):
        code, output, errors, _ = self.run_scenario("ram_missing", 1)
        self.assertEqual(code, 2)
        self.assertNotIn("UART wake cycle 1/1 PASS", output)
        self.assertIn("waiting for SRAM retention result", errors)

    def test_resume_counter_must_increase(self):
        code, output, errors, _ = self.run_scenario("bad_counter", 1)
        self.assertEqual(code, 1)
        self.assertNotIn("UART wake cycle 1/1 PASS", output)
        self.assertIn("expected 1", errors)

    def test_extended_sleep_current_interval(self):
        code, output, errors, port = self.run_scenario("success", 1, 10.0)
        self.assertEqual(code, 0, errors)
        self.assertIn("Wake deferred 10.00 s", output)
        self.assertIn(10.0, port.sleep_delays)
        self.assertEqual(port.wakes, 1)


if __name__ == "__main__":
    unittest.main()
