# Magnet driver for the ESP32 (MicroPython). Line commands over USB serial, 115200 baud:
#
#   on [seconds]   energize the magnet; auto-off after `seconds` (default MAX_ON_S)
#   off            release
#   status         -> "state on|off"
#   ping           -> "pong" (resets the heartbeat timer; see HEARTBEAT_S)
#
# Every reply is one line starting "ok" or "err". D26 drives the S8050/IRLZ44N through a
# 1k resistor; the transistor's base has a 10k pulldown so a reset or boot leaves the
# magnet off. Rationale for the limits below:
#
# MAX_ON_S caps how long one `on` can hold. The coil is ~14 ohm (0.36 A, 1.8 W at 5 V) and
# the S8050 does not saturate fully at a 3.3 V pin, so it dissipates real heat; a cap turns
# "forgot to send off" into a bounded event instead of a cooked transistor.
#
# HEARTBEAT_S = 0 disables the watchdog. Set it to e.g. 5 to drop the magnet when the Jetson
# stops talking; that also DROPS a held package, so it is a deliberate choice, not a default.
import sys
import time

import select
from machine import Pin

MAG_PIN = 26
MAX_ON_S = 30
HEARTBEAT_S = 0

mag = Pin(MAG_PIN, Pin.OUT, value=0)
poll = select.poll()
poll.register(sys.stdin, select.POLLIN)

on_until = 0      # ticks_ms deadline while the magnet is on
last_seen = time.ticks_ms()


def set_magnet(on, seconds=MAX_ON_S):
    global on_until
    mag.value(1 if on else 0)
    on_until = time.ticks_add(time.ticks_ms(), int(seconds * 1000)) if on else 0


def handle(line):
    global last_seen
    last_seen = time.ticks_ms()
    parts = line.strip().lower().split()
    if not parts:
        return
    cmd = parts[0]
    if cmd == "on":
        try:
            sec = float(parts[1]) if len(parts) > 1 else MAX_ON_S
        except ValueError:
            print("err bad seconds")
            return
        if not 0 < sec <= MAX_ON_S:
            print("err seconds must be 0..%d" % MAX_ON_S)
            return
        set_magnet(True, sec)
        print("ok on")
    elif cmd == "off":
        set_magnet(False)
        print("ok off")
    elif cmd == "status":
        print("ok state", "on" if mag.value() else "off")
    elif cmd == "ping":
        print("ok pong")
    else:
        print("err unknown command")


try:
    print("ok ready")
    while True:
        if poll.poll(50):
            handle(sys.stdin.readline())
        now = time.ticks_ms()
        if mag.value() and time.ticks_diff(now, on_until) >= 0:
            set_magnet(False)
            print("ok timeout off")
        if (HEARTBEAT_S and mag.value()
                and time.ticks_diff(now, last_seen) > HEARTBEAT_S * 1000):
            set_magnet(False)
            print("ok heartbeat off")
finally:
    mag.value(0)  # Ctrl-C into the REPL must not leave the coil energized
