#!/usr/bin/env python3
"""Concurrent CDC loopback and ISO stress for UsbCdcIsoStress firmware.

Reuse the combo runner's stream validation and ISO burst checks. No HID or
second CDC interface is opened. The CDC echo must drain completely to pass.
"""
import argparse
import threading
import time

import usb_combo_stress as combo


class Stats(combo.Stats):
    NAMES = ("loop_tx", "loop_rx", "iso")


def prepare_cdc(comm):
    comm.dtr = False
    time.sleep(0.05)
    comm.reset_input_buffer()
    comm.reset_output_buffer()
    comm.dtr = True
    received = bytearray()
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        received.extend(comm.read(comm.in_waiting or 1))
        if b"IOsonata USB CDC ISO Stress\r\n" in received:
            comm.reset_input_buffer()
            return
    raise RuntimeError("CDC+ISO banner not received after DTR toggle")


def stop_workers(workers, tx_stop, stop, iso_stop, stats, drain_timeout):
    # Keep both readers active until the final CDC write has returned.
    tx_stop.set()
    workers[0].join(1.5)
    if workers[0].is_alive():
        stats.fail("CDC TX", "writer did not stop")
    deadline = time.monotonic() + drain_timeout
    while not stop.is_set() and time.monotonic() < deadline:
        count, *rest = stats.snapshot()
        if count["loop_tx"] == count["loop_rx"]:
            break
        time.sleep(0.01)
    iso_stop.set()
    workers[2].join(3.0)
    stop.set()
    for worker in workers:
        worker.join(2.0)
        if worker.is_alive():
            stats.fail(worker.name, "worker did not stop")


def passed(stats):
    count, _, errors, _, _, paths, failure = stats.snapshot()
    return (failure is None and errors == 0 and paths["iso"] == 0
            and count["loop_tx"] == count["loop_rx"] > 0
            and count["iso"] > 0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--baud", type=int, default=1_000_000)
    parser.add_argument("--vid", type=combo.parse_int, default=0x1209)
    parser.add_argument("--pid", type=combo.parse_int, default=0x0008)
    parser.add_argument("--serial")
    parser.add_argument("--block", type=int, default=4096)
    parser.add_argument("--iso-rounds", type=int, default=32)
    parser.add_argument("--timeout", type=int, default=1000)
    parser.add_argument("--stall-timeout", type=float, default=3.0)
    parser.add_argument("--drain-timeout", type=float, default=3.0)
    args = parser.parse_args()
    if any(value <= 0 for value in (args.duration, args.baud, args.block,
            args.iso_rounds, args.timeout, args.stall_timeout, args.drain_timeout)):
        parser.error("sizes, durations and timeouts must be positive")
    if combo.serial is None or combo.usb1 is None:
        parser.error("install pyserial and libusb1 into this Python environment")

    stats = Stats()
    start, stop, tx_stop, iso_stop = (threading.Event() for _ in range(4))
    workers = []
    comm = None
    awake = combo.keep_host_awake()
    began = time.monotonic()
    interrupted = False
    try:
        serial_number = args.serial or combo.port_serial(args.port)
        if serial_number is None:
            raise RuntimeError("cannot identify CDC device serial; provide --serial")
        print(f"Device serial   : {serial_number}")
        comm = combo.open_cdc(args.port, args.baud)
        prepare_cdc(comm)
        workers = [
            threading.Thread(name="CDC-TX", target=combo.loop_tx_worker,
                args=(comm, start, stop, stats, args.block, tx_stop), daemon=True),
            threading.Thread(name="CDC-RX", target=combo.loop_rx_worker,
                args=(comm, start, stop, stats, 4096), daemon=True),
            threading.Thread(name="ISO", target=combo.iso_worker,
                args=(args.vid, args.pid, serial_number, start, iso_stop,
                      stats, args.timeout, args.iso_rounds), daemon=True),
        ]
        for worker in workers:
            worker.start()
        began = time.monotonic()
        start.set()
        previous = {name: 0 for name in Stats.NAMES}
        report = tick = began
        wall_tick = time.time()
        while time.monotonic() - began < args.duration:
            if stop.wait(0.02):
                break
            now, wall_now = time.monotonic(), time.time()
            gap = max(now - tick, wall_now - wall_tick)
            if gap >= 0.5:
                stats.add_host_pause(gap, tick, now,
                    (wall_now - wall_tick) - (now - tick) >= 0.5)
            tick, wall_tick = now, wall_now
            count, last, errors, _, _, paths, failure = stats.snapshot()
            if failure:
                break
            if now - began >= args.stall_timeout:
                for name in Stats.NAMES:
                    if last[name] is None or now - last[name] >= args.stall_timeout:
                        stats.fail(name, "stalled")
                        stop.set()
                        break
            if now - report >= 1.0:
                rates = {n: (count[n] - previous[n]) / (now - report)
                         for n in Stats.NAMES}
                print("CDC TX/RX %.0f/%.0f B/s errors %d | ISO %.0f B/s errors %d"
                      % (rates["loop_tx"], rates["loop_rx"], errors,
                         rates["iso"], paths["iso"]), flush=True)
                report, previous = now, count
    except KeyboardInterrupt:
        interrupted = True
        stats.fail("host", "interrupted before test completion")
    except Exception as exc:
        stats.fail("startup", exc)
    finally:
        start.set()
        if workers:
            stop_workers(workers, tx_stop, stop, iso_stop, stats, args.drain_timeout)
        if comm is not None:
            comm.close()
        if awake is not None:
            awake.terminate()

    elapsed = max(time.monotonic() - began, 0.001)
    count, _, errors, _, _, paths, failure = stats.snapshot()
    print(f"\nElapsed sec     : {elapsed:.2f}")
    for name in Stats.NAMES:
        print(f"{name:16}: {count[name]} bytes, {count[name] / elapsed:.2f} B/s")
    print(f"CDC pending     : {count['loop_tx'] - count['loop_rx']}")
    print(f"CDC errors      : {errors}")
    print(f"ISO errors      : {paths['iso']}")
    print(f"ISO host misses : {stats.iso_host_miss_count()}")
    print(f"ISO host skews  : {stats.iso_host_skew_count()}")
    print(f"ISO host unsent : {stats.iso_host_unsent_count()}")
    if failure:
        print(f"Failure         : {failure}")
    pauses, seconds = stats.host_pause_summary()
    print(f"Host pauses     : {pauses} ({seconds:.1f} s)")
    if pauses or interrupted:
        print("Result          : INCONCLUSIVE")
        return 3
    success = passed(stats)
    print("Result          : " + ("PASS" if success else "FAIL"))
    return 0 if success else 1


if __name__ == "__main__":
    raise SystemExit(main())
