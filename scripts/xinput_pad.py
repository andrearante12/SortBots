#!/usr/bin/env python3
"""Read an XInput (Xbox 360 protocol) gamepad straight over USB, no kernel driver.

Why this exists: the Jetson kernel has no `xpad` module, so a pad in XInput mode (Flydigi
Vader 5 Pro, 37d7:2401, tested) gets no /dev/input gamepad node and evdev can't see its
sticks. Interface 0 of such a pad is class 0xff / subclass 0x5d / protocol 0x01 with an IN
endpoint streaming 20-byte reports, so pyusb can read it directly.

Needs pyusb (lerobot env) and write access to the USB node, which is root:root after every
plug: `sudo chmod 666 /dev/bus/usb/BUS/DEV` (`python3 scripts/xinput_pad.py` prints it).

    from xinput_pad import XPad
    pad = XPad()
    while True:
        st = pad.poll()      # returns the latest state; never blocks past ~20 ms
        st["lx"], st["rt"], st["buttons"]["A"], ...
"""
import sys
import time

# byte 2 and byte 3 of the report; names follow the Xbox layout
_B2 = {0x01: "DUP", 0x02: "DDOWN", 0x04: "DLEFT", 0x08: "DRIGHT",
       0x10: "START", 0x20: "BACK", 0x40: "L3", 0x80: "R3"}
_B3 = {0x01: "LB", 0x02: "RB", 0x04: "GUIDE", 0x10: "A", 0x20: "B", 0x40: "X", 0x80: "Y"}
BUTTONS = list(_B2.values()) + list(_B3.values())


def parse(report):
    """20-byte XInput report -> state dict. Sticks are -1..1 with UP positive (the raw
    protocol's own sign: do not flip it here), triggers 0..1. None if not an input report."""
    if len(report) < 14 or report[0] != 0x00 or report[1] != 0x14:
        return None

    def s16(lo):
        v = report[lo] | (report[lo + 1] << 8)
        return (v - 65536 if v >= 32768 else v) / 32768.0

    held = set()
    for mask, name in _B2.items():
        if report[2] & mask:
            held.add(name)
    for mask, name in _B3.items():
        if report[3] & mask:
            held.add(name)
    return {"lt": report[4] / 255.0, "rt": report[5] / 255.0,
            "lx": s16(6), "ly": s16(8), "rx": s16(10), "ry": s16(12),
            "buttons": {b: (b in held) for b in BUTTONS}}


def deadzone(v, flat=0.12):
    return 0.0 if abs(v) < flat else (abs(v) - flat) / (1 - flat) * (1 if v > 0 else -1)


class XPad:
    def __init__(self, vid=0x37D7, pid=0x2401):
        import usb.core  # lazy: the offline tests run under system python3 with no pyusb
        import usb.util
        self._usb = usb
        self.dev = usb.core.find(idVendor=vid, idProduct=pid)
        if self.dev is None:
            raise SystemExit(f"xinput_pad: no USB device {vid:04x}:{pid:04x} (pad plugged in?)")
        self.ep_in = None
        try:
            # The kernel has already configured the device and holds the HID interfaces, so
            # set_configuration() fails with "Resource busy"; use what is active, claim only ours.
            try:
                cfg = self.dev.get_active_configuration()
            except usb.core.USBError:
                self.dev.set_configuration()
                cfg = self.dev.get_active_configuration()
            for intf in cfg:
                if (intf.bInterfaceClass, intf.bInterfaceSubClass, intf.bInterfaceProtocol) == (0xFF, 0x5D, 0x01):
                    self.intf = intf.bInterfaceNumber
                    self.ep_in = usb.util.find_descriptor(
                        intf, custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN)
            if self.ep_in is None:
                raise SystemExit("xinput_pad: no XInput interface (class ff/5d/01) on this device")
            usb.util.claim_interface(self.dev, self.intf)
        except usb.core.USBError as e:
            if e.errno == 13:
                bus, addr = self.dev.bus, self.dev.address
                raise SystemExit(f"xinput_pad: permission denied. Run: sudo chmod 666 /dev/bus/usb/{bus:03d}/{addr:03d}")
            raise
        self.state = {"lt": 0.0, "rt": 0.0, "lx": 0.0, "ly": 0.0, "rx": 0.0, "ry": 0.0,
                      "buttons": {b: False for b in BUTTONS}}
        self.last_report = 0.0

    def poll(self, timeout_ms=20):
        """Drain what's pending (one read waits up to timeout_ms), keep the latest state."""
        usb = self._usb
        for _ in range(8):
            try:
                data = self.ep_in.read(32, timeout=timeout_ms)
            except usb.core.USBTimeoutError:
                break
            except usb.core.USBError as e:
                raise SystemExit(f"xinput_pad: lost the pad ({e})")
            st = parse(bytes(data))
            if st:
                self.state = st
                self.last_report = time.monotonic()
            timeout_ms = 1
        return self.state

    def stale(self, seconds=0.5):
        """Seconds-since-last-report check. DO NOT use it to detect a lost pad: reports are sent
        only when an input changes, so an idle or steadily held pad is silent. (A disconnect
        raises in poll().)"""
        return time.monotonic() - self.last_report > seconds

    def close(self):
        try:
            self._usb.util.release_interface(self.dev, self.intf)
            self._usb.util.dispose_resources(self.dev)
        except Exception:
            pass


if __name__ == "__main__":  # live monitor: find out what your pad reports
    pad = XPad()
    print("reading; Ctrl-C to quit")
    try:
        while True:
            s = pad.poll(50)
            down = [b for b, v in s["buttons"].items() if v]
            print(f"LX{s['lx']:+.2f} LY{s['ly']:+.2f} RX{s['rx']:+.2f} RY{s['ry']:+.2f} "
                  f"LT{s['lt']:.2f} RT{s['rt']:.2f} {' '.join(down):<30}", end="\r", flush=True)
    except KeyboardInterrupt:
        print()
    finally:
        pad.close()
