# Electromagnet (ESP32 driver)

The magnet is switched by an ESP32 dev board (30-pin, CP2102) running MicroPython, not by the
Jetson's header: pins 7 and 29 stayed at 0 V under `gpioset` on this Jetson even unloaded
(2026-11-06; `jetson-io` showed them `unused`, cause not found). The Jetson only sends text
commands over USB.

```bash
scripts/magnet.py status
scripts/magnet.py pulse 2        # on 2 s, then off (off even on Ctrl-C)
scripts/magnet.py on 10          # on for 10 s (firmware cap 30 s); stays on after the script exits
scripts/magnet.py off
```

Runs in the `lerobot` env (pyserial). Firmware: `hardware/magnet_esp32/main.py`.

## Circuit

ESP32 D26 → 1 kΩ → S8050 base (10 kΩ base→emitter pulldown); magnet between +5 V and the
collector; 1N400x flyback diode across the magnet, stripe on the +5 V side; emitter, supply −
and ESP32 GND joined. Coil measures 14 Ω (0.36 A). An IRLZ44N (gate via 220 Ω) is the
better switch than the S8050, which does not saturate from a 3.3 V pin at this current.

## Things that cost time

- **Opening the serial port resets the ESP32** (DTR/RTS wired to EN), so the magnet drops and
  the board needs ~2 s to boot. `scripts/magnet.py` waits for a `ping` reply. A held magnet
  survives only while one process keeps the port open: `on N` then exiting the script is fine,
  but running another `magnet.py` command (which reopens the port) releases it.
- **The firmware caps every `on` at 30 s** (`MAX_ON_S`) and has an optional heartbeat
  watchdog (`HEARTBEAT_S`, off): a watchdog also drops a held package when the Jetson stops
  talking, so enabling it is a choice.
- **Flashing this board**: esptool's stub loader fails here ("no response"); use
  `esptool --port /dev/ttyUSB0 --baud 115200 --no-stub write-flash 0x1000 <ESP32_GENERIC.bin>`.
  `--erase-all` needs the stub, so old data stays in the filesystem partition and MicroPython
  reports "filesystem appears to be corrupted" in a boot loop; wipe it with
  `--no-stub erase-region 0x200000 0x200000` (4 MB flash), then re-flash.
- Copy the firmware with `mpremote connect /dev/ttyUSB0 cp hardware/magnet_esp32/main.py :main.py`,
  then reset the board. Both ttyUSB0 and the arm's ttyACM0 come back `root:dialout 660` after a
  re-plug; `sudo chmod 666` them.
