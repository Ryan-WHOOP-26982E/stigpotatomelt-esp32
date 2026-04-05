#!/usr/bin/env python3
"""
potatomelt telemetry receiver (BLE)
=====================================
1. Flash potatomelt with TELEMETRY_ENABLED defined in melty_config.h
2. Run this script: python3 telemetry_receiver.py [--plot]
   (No WiFi connection needed — uses Bluetooth Low Energy)

Requirements:
    pip install bleak

The script scans for a BLE device named "potatomelt", connects, and subscribes
to Nordic UART Service notifications containing CSV telemetry packets.
"""

import asyncio
import argparse
import sys
import collections
import time
import threading

from bleak import BleakScanner, BleakClient

BLE_DEVICE_NAME = "potatomelt"
BLE_NUS_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

STATE_NAMES = {
    0: "SPINNING",
    1: "READY  ",
    2: "LOW_BAT",
    3: "STALE  ",
    4: "NO_CTRL",
}

FIELDS = ["ms", "state", "target_rpm", "rpm", "throttle", "bat", "connected", "loop_us", "trim", "raw_g"]


def parse_packet(line: str):
    try:
        parts = line.strip().split(",")
        if len(parts) < len(FIELDS):
            return None
        return {
            "ms":         int(parts[0]),
            "state":      int(parts[1]),
            "target_rpm": int(parts[2]),
            "rpm":        float(parts[3]),
            "throttle":   float(parts[4]),
            "bat":        int(parts[5]),
            "connected":  int(parts[6]),
            "loop_us":    int(parts[7]),
            "trim":       float(parts[8]) if len(parts) > 8 else 1.0,
            "raw_g":      float(parts[9]) if len(parts) > 9 else 0.0,
        }
    except Exception:
        return None


async def find_device():
    """Scan for the potatomelt BLE device and return its address."""
    print(f"Scanning for '{BLE_DEVICE_NAME}'...")
    device = await BleakScanner.find_device_by_name(BLE_DEVICE_NAME, timeout=10.0)
    if device is None:
        print(f"ERROR: Could not find '{BLE_DEVICE_NAME}'. Is the robot on and TELEMETRY_ENABLED?")
        sys.exit(1)
    print(f"Found: {device.name} [{device.address}]")
    return device


async def run_terminal():
    """Connect and print telemetry to the terminal."""
    device = await find_device()

    loop_times = collections.deque(maxlen=50)
    line_buf = ""
    last_packet_time = time.monotonic()

    print(f"\n{'ms':>10}  {'state':<8}  {'RPM':>6}/{'TRPM':<6}  {'THR':>5}  "
          f"{'BAT':>4}  {'loop_ms':>7}  {'loop_max':>8}  {'trim':>7}  {'raw_g':>7}")
    print("-" * 90)

    def on_notify(_handle, data: bytearray):
        nonlocal line_buf, last_packet_time
        # BLE notifications may arrive in chunks — reassemble lines
        line_buf += data.decode(errors="replace")
        while "\n" in line_buf:
            line, line_buf = line_buf.split("\n", 1)
            p = parse_packet(line)
            if p is None:
                continue
            last_packet_time = time.monotonic()
            loop_ms = p["loop_us"] / 1000.0
            loop_times.append(loop_ms)
            loop_max = max(loop_times)
            state_name = STATE_NAMES.get(p["state"], f"?{p['state']}")
            print(
                f"{p['ms']:>10}  {state_name}  {p['rpm']:>6.0f}/{p['target_rpm']:<6}  "
                f"{p['throttle']:>5.0f}  {p['bat']:>3d}%  {loop_ms:>6.1f}ms  "
                f"{loop_max:>7.1f}ms  {p['trim']:>7.4f}  {p['raw_g']:>7.3f}g"
            )

    async with BleakClient(device) as client:
        print(f"Connected. Receiving telemetry (Ctrl+C to stop)...\n")
        await client.start_notify(BLE_NUS_TX_UUID, on_notify)
        try:
            while True:
                await asyncio.sleep(3.0)
                elapsed = time.monotonic() - last_packet_time
                if elapsed > 3.0:
                    print(f"  (no packets for {elapsed:.1f}s — is the robot still on?)")
        except asyncio.CancelledError:
            pass
        finally:
            await client.stop_notify(BLE_NUS_TX_UUID)


async def run_plot():
    """Live matplotlib plot — requires: pip install matplotlib"""
    try:
        import matplotlib.pyplot as plt
        import matplotlib.animation as animation
    except ImportError:
        print("matplotlib not found. Install it with:  pip install matplotlib")
        sys.exit(1)

    device = await find_device()

    MAX_POINTS = 300
    # Thread-safe queue for passing parsed packets from BLE callback to plot
    import queue
    pkt_queue = queue.Queue()
    line_buf_holder = [""]

    t_ms      = collections.deque(maxlen=MAX_POINTS)
    rpm_data  = collections.deque(maxlen=MAX_POINTS)
    trpm_data = collections.deque(maxlen=MAX_POINTS)
    loop_data = collections.deque(maxlen=MAX_POINTS)
    thr_data  = collections.deque(maxlen=MAX_POINTS)

    def on_notify(_handle, data: bytearray):
        line_buf_holder[0] += data.decode(errors="replace")
        while "\n" in line_buf_holder[0]:
            line, line_buf_holder[0] = line_buf_holder[0].split("\n", 1)
            p = parse_packet(line)
            if p:
                pkt_queue.put(p)

    fig, (ax_rpm, ax_loop, ax_thr) = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    fig.suptitle("potatomelt live telemetry (BLE)", fontsize=13)

    line_rpm,  = ax_rpm.plot([], [], color="royalblue",  label="RPM (measured)")
    line_trpm, = ax_rpm.plot([], [], color="orange",     label="RPM (target)", linestyle="--")
    line_loop, = ax_loop.plot([], [], color="crimson",   label="loop time (ms)")
    line_thr,  = ax_thr.plot([], [], color="seagreen",   label="throttle output")

    ax_rpm.set_ylabel("RPM");  ax_rpm.legend(loc="upper left", fontsize=8)
    ax_rpm.axhline(0, color="grey", linewidth=0.5)
    ax_loop.set_ylabel("loop ms")
    ax_loop.axhline(10, color="grey", linewidth=0.8, linestyle="--", label="10ms target")
    ax_loop.legend(loc="upper left", fontsize=8)
    ax_thr.set_ylabel("throttle"); ax_thr.set_xlabel("time (ms)")
    ax_thr.legend(loc="upper left", fontsize=8)

    # Run BLE in a background thread so matplotlib can own the main thread
    ble_loop = asyncio.new_event_loop()
    stop_event = threading.Event()

    async def ble_task():
        async with BleakClient(device) as client:
            print(f"Connected. Receiving telemetry...\n")
            await client.start_notify(BLE_NUS_TX_UUID, on_notify)
            while not stop_event.is_set():
                await asyncio.sleep(0.1)
            await client.stop_notify(BLE_NUS_TX_UUID)

    def ble_thread():
        ble_loop.run_until_complete(ble_task())

    t = threading.Thread(target=ble_thread, daemon=True)
    t.start()

    def update(_frame):
        while not pkt_queue.empty():
            p = pkt_queue.get_nowait()
            t_ms.append(p["ms"])
            rpm_data.append(p["rpm"])
            trpm_data.append(p["target_rpm"])
            loop_data.append(p["loop_us"] / 1000.0)
            thr_data.append(p["throttle"])

        if not t_ms:
            return line_rpm, line_trpm, line_loop, line_thr

        xs = list(t_ms)
        line_rpm.set_data(xs,  list(rpm_data))
        line_trpm.set_data(xs, list(trpm_data))
        line_loop.set_data(xs, list(loop_data))
        line_thr.set_data(xs,  list(thr_data))
        for ax in (ax_rpm, ax_loop, ax_thr):
            ax.relim(); ax.autoscale_view()
        return line_rpm, line_trpm, line_loop, line_thr

    _ani = animation.FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    plt.tight_layout()
    try:
        plt.show()
    finally:
        stop_event.set()


def main():
    parser = argparse.ArgumentParser(description="potatomelt BLE telemetry receiver")
    parser.add_argument("--plot", action="store_true",
                        help="Show live matplotlib graphs instead of terminal output")
    args = parser.parse_args()

    print("potatomelt BLE telemetry receiver")
    print("Make sure the robot is on and TELEMETRY_ENABLED is defined.\n")

    try:
        if args.plot:
            asyncio.run(run_plot())
        else:
            asyncio.run(run_terminal())
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
