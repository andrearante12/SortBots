"""One in-place terminal status line for the teleop scripts (pure: no hardware imports)."""

RESET, DIM, GREEN, YELLOW, RED = "\x1b[0m", "\x1b[2m", "\x1b[1;32m", "\x1b[33m", "\x1b[1;31m"

# STS3215 Status register (65) bits, from Feetech's STS memory table
_FAULT_BITS = {0: "voltage", 1: "angle-sensor", 2: "overheat", 3: "overcurrent", 5: "overload"}


def servo_fault(bits):
    """Status register value -> 'overload+overheat'; unknown bits show as 'bitN'."""
    return "+".join(_FAULT_BITS.get(i, f"bit{i}") for i in range(8) if bits >> i & 1)


def status_line(width, mode, held, pose_txt, level_txt, mag_on, mag_s, ok, bus_txt, note_txt):
    """One status line, ANSI-coloured, cut to `width` by dropping segments from the right.
    Pure so the offline test can check it. Priority order is the order of the segments."""
    segs = [(mode, ""), ("HELD" if held else "idle", GREEN if held else DIM),
            (pose_txt, ""), (level_txt, DIM),
            ("MAG ON %3ds" % mag_s if mag_on else "MAG off", RED if mag_on else DIM)]
    if not ok:
        segs.append(("LIMIT", YELLOW))
    if bus_txt:
        segs.append((bus_txt, RED if "LOW" in bus_txt else DIM))
    if note_txt:
        segs.append((note_txt, YELLOW))
    while len(segs) > 1 and sum(len(t) for t, _ in segs) + 3 * (len(segs) - 1) > width:
        segs.pop()
    return " | ".join(f"{c}{t}{RESET}" if c else t for t, c in segs)
