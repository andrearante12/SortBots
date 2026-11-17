#!/usr/bin/env python3
"""The few Feetech STS3215 servo-bus operations the wheels need, stdlib only.

lerobot's SerialMotorsBus does this for scripts/drive_bridge.py, but lerobot isn't in the
Jazzy robot container (it pulls torch, and the stack is numpy-only), so nodes/base_driver.py
speaks the bus itself. Every frame here was checked byte for byte against what lerobot sends
(captured on a pty, 2026-11-16; tests/feetech_test.py holds those frames):

    ff ff <id> <len> <instr> <params...> <checksum>     len = len(params) + 2
    checksum = ~(id + len + instr + sum(params)) & 0xff

Protocol 0 (STS/SMS): little-endian words, Goal_Velocity sign-magnitude with the sign on
bit 15, 1 Mbaud. Sync writes go to the broadcast id and get no reply; plain writes and reads
are answered by a status frame `ff ff <id> <len> <error> <data...> <checksum>`.
"""
from __future__ import annotations

import os
import select
import termios
import time

BAUD = 1_000_000
BROADCAST = 0xFE
PING, READ, WRITE, SYNC_WRITE = 0x01, 0x02, 0x03, 0x83

# STS3215 control table, the registers used here: (address, bytes).
MODEL_NUMBER = (3, 2)
OPERATING_MODE = (33, 1)   # EEPROM: 0 position, 1 velocity
TORQUE_ENABLE = (40, 1)
GOAL_VELOCITY = (46, 2)    # sign-magnitude, sign on bit 15
PRESENT_VOLTAGE = (62, 1)  # 0.1 V

STS3215_MODEL = 777
VELOCITY_MODE = 1


def checksum(body: bytes) -> int:
    return ~sum(body) & 0xFF


def frame(servo_id: int, instr: int, params: bytes = b"") -> bytes:
    body = bytes([servo_id, len(params) + 2, instr]) + params
    return b"\xff\xff" + body + bytes([checksum(body)])


def encode(value: int, size: int, sign_bit: int | None = None) -> bytes:
    """An int as `size` little-endian bytes; negative values as sign-magnitude on sign_bit."""
    if sign_bit is not None:
        # Both directions: a positive value reaching the sign bit would go out as a reverse.
        if abs(value) >= 1 << sign_bit:
            raise ValueError(f"{value} doesn't fit sign-magnitude on bit {sign_bit}")
        if value < 0:
            value = -value | (1 << sign_bit)
    if not 0 <= value < 1 << (8 * size):
        raise ValueError(f"{value} doesn't fit in {size} bytes")
    return value.to_bytes(size, "little")


def decode(data: bytes, sign_bit: int | None = None) -> int:
    value = int.from_bytes(data, "little")
    if sign_bit is not None and value & (1 << sign_bit):
        value = -(value & ((1 << sign_bit) - 1))
    return value


def velocity(raw: int) -> bytes:
    return encode(raw, 2, sign_bit=15)


def write_frame(servo_id: int, reg: tuple[int, int], value: int) -> bytes:
    addr, size = reg
    return frame(servo_id, WRITE, bytes([addr]) + encode(value, size))


def read_frame(servo_id: int, reg: tuple[int, int]) -> bytes:
    addr, size = reg
    return frame(servo_id, READ, bytes([addr, size]))


def sync_velocity_frame(raw_by_id: dict[int, int]) -> bytes:
    addr, size = GOAL_VELOCITY
    params = bytes([addr, size])
    for sid, raw in raw_by_id.items():
        params += bytes([sid]) + velocity(raw)
    return frame(BROADCAST, SYNC_WRITE, params)


def parse_status(buf: bytes, servo_id: int) -> tuple[int, bytes] | None:
    """(error byte, data) from the first valid status frame of servo_id in buf, else None."""
    i = 0
    while True:
        i = buf.find(b"\xff\xff", i)
        if i < 0 or len(buf) < i + 6:
            return None
        sid, length = buf[i + 2], buf[i + 3]
        end = i + 4 + length
        if end > len(buf):
            return None
        body = buf[i + 2:end - 1]
        if sid == servo_id and length >= 2 and checksum(body) == buf[end - 1]:
            return buf[i + 4], bytes(buf[i + 5:end - 1])
        i += 1


class BusError(RuntimeError):
    pass


class Bus:
    """One servo board's serial port. Not thread-safe: one caller at a time."""

    def __init__(self, port: str, timeout_s: float = 0.02):
        self.port, self.timeout_s = port, timeout_s
        self.fd = None

    def open(self):
        fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            attrs = termios.tcgetattr(fd)
            # raw 8N1, no flow control, no echo: a servo frame is binary.
            attrs[0] = 0                                                  # iflag
            attrs[1] = 0                                                  # oflag
            attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL       # cflag
            attrs[3] = 0                                                  # lflag
            # termios has no B1000000 constant on every platform; Linux does.
            speed = getattr(termios, "B1000000", None)
            if speed is None:
                raise BusError("this platform's termios has no 1 Mbaud")
            attrs[4] = attrs[5] = speed
            attrs[6][termios.VMIN] = 0
            attrs[6][termios.VTIME] = 0
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
            termios.tcflush(fd, termios.TCIOFLUSH)
        except Exception:
            os.close(fd)
            raise
        self.fd = fd

    def close(self):
        if self.fd is not None:
            fd, self.fd = self.fd, None
            os.close(fd)

    def _send(self, data: bytes):
        if self.fd is None:
            raise BusError("bus not open")
        termios.tcflush(self.fd, termios.TCIFLUSH)  # a stale reply must not answer this request
        view = memoryview(data)
        while view:
            n = os.write(self.fd, view)
            view = view[n:]

    def _reply(self, servo_id: int) -> tuple[int, bytes]:
        buf, deadline = b"", time.monotonic() + self.timeout_s
        while True:
            left = deadline - time.monotonic()
            if left <= 0:
                raise BusError(f"servo {servo_id}: no reply on {self.port}")
            r, _, _ = select.select([self.fd], [], [], left)
            if r:
                buf += os.read(self.fd, 256)
                got = parse_status(buf, servo_id)
                if got is not None:
                    return got

    def write(self, servo_id: int, reg: tuple[int, int], value: int, retries: int = 2):
        for attempt in range(retries + 1):
            try:
                self._send(write_frame(servo_id, reg, value))
                self._reply(servo_id)
                return
            except BusError:
                if attempt == retries:
                    raise

    def read(self, servo_id: int, reg: tuple[int, int], sign_bit: int | None = None,
             retries: int = 2) -> int:
        for attempt in range(retries + 1):
            try:
                self._send(read_frame(servo_id, reg))
                _, data = self._reply(servo_id)
                if len(data) != reg[1]:
                    raise BusError(f"servo {servo_id}: {len(data)} bytes for a {reg[1]}-byte register")
                return decode(data, sign_bit)
            except BusError:
                if attempt == retries:
                    raise

    def sync_velocity(self, raw_by_id: dict[int, int]):
        """Goal_Velocity to several servos in one unanswered broadcast frame."""
        self._send(sync_velocity_frame(raw_by_id))


def open_wheels(port: str, ids: list[int]) -> Bus:
    """Open the board and put the wheel servos in velocity mode, stopped, torque on.

    The same sequence drive_bridge.py gets from lerobot: torque off (which opens the EEPROM
    lock), Operating_Mode = velocity, Goal_Velocity 0, torque on. Operating_Mode is EEPROM,
    so it is only written when it isn't velocity already: the wheels keep it across power
    cycles, and EEPROM has a write budget.
    """
    bus = Bus(port)
    bus.open()
    try:
        for sid in ids:
            model = bus.read(sid, MODEL_NUMBER)
            if model != STS3215_MODEL:
                raise BusError(f"servo {sid} on {port} is model {model}, not an STS3215 ({STS3215_MODEL})")
        bus.sync_velocity(dict.fromkeys(ids, 0))
        for sid in ids:
            if bus.read(sid, OPERATING_MODE) != VELOCITY_MODE:
                bus.write(sid, TORQUE_ENABLE, 0)
                bus.write(sid, OPERATING_MODE, VELOCITY_MODE)
        bus.sync_velocity(dict.fromkeys(ids, 0))
        for sid in ids:
            bus.write(sid, TORQUE_ENABLE, 1)
    except Exception:
        bus.close()
        raise
    return bus


def release_wheels(bus: Bus, ids: list[int]):
    """Wheels to 0, then torque off (they free-roll and the robot can be pushed), then close.
    The zero goes out three times: a sync write is never acknowledged."""
    try:
        for _ in range(3):
            bus.sync_velocity(dict.fromkeys(ids, 0))
            time.sleep(0.005)
        for sid in ids:
            try:
                bus.write(sid, TORQUE_ENABLE, 0)
            except BusError:
                pass
    finally:
        bus.close()
