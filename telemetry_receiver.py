#!/usr/bin/env python3
"""
potatomelt telemetry receiver
==============================
1. Flash potatomelt with TELEMETRY_ENABLED defined in melty_config.h
2. Connect your laptop to WiFi: "potatomelt" / password: "potato123"
3. Run this script: python3 telemetry_receiver.py [--plot]

Packet CSV format (sent by ESP32):
  ms, state, target_rpm, rpm, throttle, bat_pct, connected, loop_us, trim
"""

import socket
import argparse
import sys
import collections
import time

UDP_PORT = 4210

STATE_NAMES = {
    0: "SPINNING",
    1: "READY  ",
    2: "LOW_BAT",
    3: "STALE  ",
    4: "NO_CTRL",
}

FIELDS = ["ms", "state", "target_rpm", "rpm", "throttle", "bat", "connected", "loop_us", "trim", "raw_g"]


def parse_packet(raw: bytes):
    try:
        parts = raw.decode().strip().split(",")
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


def run_terminal(sock):
    """Print telemetry to the terminal with a rolling stats line."""
    loop_times = collections.deque(maxlen=50)   # keep last 50 for stats
    last_packet_time = time.monotonic()

    print(f"{'ms':>10}  {'state':<8}  {'RPM':>6}/{'TRPM':<6}  {'THR':>5}  "
          f"{'BAT':>4}  {'loop_ms':>7}  {'loop_max':>8}  {'trim':>7}  {'raw_g':>7}")
    print("-" * 90)

    while True:
        try:
            data, _ = sock.recvfrom(256)
        except socket.timeout:
            elapsed = time.monotonic() - last_packet_time
            print(f"  (no packets for {elapsed:.1f}s — is the bot on and WiFi connected?)")
            continue

        p = parse_packet(data)
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


def run_plot(sock):
    """Live matplotlib plot — requires: pip install matplotlib"""
    try:
        import matplotlib.pyplot as plt
        import matplotlib.animation as animation
    except ImportError:
        print("matplotlib not found. Install it with:  pip install matplotlib")
        sys.exit(1)

    MAX_POINTS = 300  # ~30 seconds at 10ms/packet
    t_ms       = collections.deque(maxlen=MAX_POINTS)
    rpm_data   = collections.deque(maxlen=MAX_POINTS)
    trpm_data  = collections.deque(maxlen=MAX_POINTS)
    loop_data  = collections.deque(maxlen=MAX_POINTS)
    thr_data   = collections.deque(maxlen=MAX_POINTS)

    fig, (ax_rpm, ax_loop, ax_thr) = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    fig.suptitle("potatomelt live telemetry", fontsize=13)

    line_rpm,  = ax_rpm.plot([], [], color="royalblue",  label="RPM (measured)")
    line_trpm, = ax_rpm.plot([], [], color="orange",     label="RPM (target)", linestyle="--")
    line_loop, = ax_loop.plot([], [], color="crimson",   label="loop time (ms)")
    line_thr,  = ax_thr.plot([], [], color="seagreen",  label="throttle output")

    ax_rpm.set_ylabel("RPM")
    ax_rpm.legend(loc="upper left", fontsize=8)
    ax_rpm.axhline(0, color="grey", linewidth=0.5)

    ax_loop.set_ylabel("loop ms")
    ax_loop.axhline(10, color="grey", linewidth=0.8, linestyle="--", label="10ms target")
    ax_loop.legend(loc="upper left", fontsize=8)

    ax_thr.set_ylabel("throttle")
    ax_thr.set_xlabel("time (ms)")
    ax_thr.legend(loc="upper left", fontsize=8)

    sock.setblocking(False)

    def update(_frame):
        # Drain all buffered packets each frame
        while True:
            try:
                data, _ = sock.recvfrom(256)
                p = parse_packet(data)
                if p:
                    t_ms.append(p["ms"])
                    rpm_data.append(p["rpm"])
                    trpm_data.append(p["target_rpm"])
                    loop_data.append(p["loop_us"] / 1000.0)
                    thr_data.append(p["throttle"])
            except BlockingIOError:
                break
            except Exception:
                break

        if not t_ms:
            return line_rpm, line_trpm, line_loop, line_thr

        xs = list(t_ms)
        line_rpm.set_data(xs, list(rpm_data))
        line_trpm.set_data(xs, list(trpm_data))
        line_loop.set_data(xs, list(loop_data))
        line_thr.set_data(xs, list(thr_data))

        for ax in (ax_rpm, ax_loop, ax_thr):
            ax.relim()
            ax.autoscale_view()

        return line_rpm, line_trpm, line_loop, line_thr

    _ani = animation.FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    plt.tight_layout()
    plt.show()


def main():
    parser = argparse.ArgumentParser(description="potatomelt UDP telemetry receiver")
    parser.add_argument("--plot", action="store_true",
                        help="Show live matplotlib graphs instead of terminal output")
    parser.add_argument("--port", type=int, default=UDP_PORT,
                        help=f"UDP port to listen on (default: {UDP_PORT})")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", args.port))

    if not args.plot:
        sock.settimeout(3.0)

    print(f"potatomelt telemetry receiver — listening on UDP port {args.port}")
    print(f"Connect your laptop to WiFi: '{FIELDS[0] or 'potatomelt'}' / 'potato123'")
    print(f"Mode: {'live plot' if args.plot else 'terminal'}\n")

    try:
        if args.plot:
            run_plot(sock)
        else:
            run_terminal(sock)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
