#!/usr/bin/env python3
"""Drive the electromagnet through the ESP32 (firmware: hardware/magnet_esp32/main.py).

Runs in the `lerobot` env (pyserial), NOT in a ROS shell.

    scripts/magnet.py status
    scripts/magnet.py on [SECONDS]     stays on until `off` or the firmware's cap (30 s)
    scripts/magnet.py off
    scripts/magnet.py pulse SECONDS    on, wait, off (off even on Ctrl-C)

The port defaults to the CP2102 by-id path so it can't be confused with the arm's board on
/dev/ttyACM0. DTR/RTS are held low BEFORE the port opens: the dev board wires them to EN and
GPIO0, and a toggle resets the ESP32 (magnet off, ~2 s of boot). Importable: `Magnet` below.
"""
import argparse
import glob
import time

import serial

DEFAULT_PORT_GLOB = "/dev/serial/by-id/usb-Silicon_Labs_CP2102*"


class Magnet:
    def __init__(self, port=None, timeout=1.0):
        port = port or next(iter(sorted(glob.glob(DEFAULT_PORT_GLOB))), None)
        if not port:
            raise SystemExit("magnet: no ESP32 found (CP2102 not on USB); pass --port")
        self.s = serial.Serial()
        self.s.port, self.s.baudrate, self.s.timeout = port, 115200, timeout
        self.s.dtr = self.s.rts = False
        self.s.open()
        # Opening the port RESETS the ESP32 (the dev board wires DTR/RTS to EN; the kernel
        # raises them for an instant at open): ~2 s of boot text, magnet off. So a fresh Magnet
        # always starts released. Keep reading until a real "ok pong", pinging as we wait.
        self.s.timeout = 0.2
        deadline, last_ping = time.monotonic() + 6.0, 0.0
        while time.monotonic() < deadline:
            if time.monotonic() - last_ping > 0.6:
                self.s.write(b"ping\n")
                last_ping = time.monotonic()
            if self.s.readline().decode(errors="replace").strip() == "ok pong":
                self.s.timeout = timeout
                self.s.reset_input_buffer()
                return
        raise SystemExit("magnet: ESP32 never answered ping (is main.py running?)")

    def _ask(self, cmd):
        self.s.write((cmd + "\n").encode())
        while True:
            line = self.s.readline().decode(errors="replace").strip()
            if not line:
                raise SystemExit(f"magnet: no reply to '{cmd}' (is main.py running on the ESP32?)")
            if line == "ok ready":  # boot banner from a reset, not the reply
                continue
            if line.startswith(("ok", "err")):
                return line

    def on(self, seconds=None):
        return self._ask("on" if seconds is None else f"on {seconds}")

    def off(self):
        return self._ask("off")

    def status(self):
        return self._ask("status")

    def close(self):
        self.s.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("cmd", choices=["status", "on", "off", "pulse"])
    ap.add_argument("seconds", nargs="?", type=float)
    a = ap.parse_args()
    if a.cmd == "pulse" and a.seconds is None:
        ap.error("pulse needs SECONDS")

    m = Magnet(a.port)
    try:
        if a.cmd == "pulse":
            print(m.on(a.seconds + 1))  # firmware cap is a backstop; we send the off
            time.sleep(a.seconds)
            print(m.off())
        elif a.cmd == "on":
            print(m.on(a.seconds))
        elif a.cmd == "off":
            print(m.off())
        else:
            print(m.status())
    except KeyboardInterrupt:
        print(m.off())
    finally:
        m.close()


if __name__ == "__main__":
    main()
