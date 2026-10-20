"""Pure-Python decoders for the emulation's captures: classic pcap, radiotap,
802.11 (+802.11s mesh control), LLC/SNAP, batman-adv IV, Ethernet, IPv4,
UDP/TCP/ICMP and the UDP Status prototype's CDR header.

No scapy, no tshark: neither is in the VM or on the host, and the frames
here have exactly one shape each (checked against
commsim/results/viz_smoke/hwsim0.pcap, 2026-10-20):

    ACK         radiotap 14 B (Flags, Channel)           FC 0xd400, RA @4
    unicast     radiotap 22 B (TSFT, Flags, Rate, Chan)  FC 0x8803 ToDS|FromDS,
                4 addresses + QoS + 6 B mesh ctl -> LLC @38 -> 0x4305 @46
    broadcast   radiotap 22 B                            FC 0x8802 FromDS,
                3 addresses + QoS + mesh ctl -> LLC @32 -> batman @40
    mgmt        beacon 0x8000, action 0xd000 (acked, like unicast data)

batman-adv here is BATMAN_IV (version 15): on the wire `43 05 | 00 0f`.
The BATMAN_V numbering (OGM2 = 0x04) is NOT what this mesh sends.

Everything "best effort" is marked: a layout this file has not seen degrades
to `rate 0` / `kind other`, never to an exception, so one odd frame cannot
sink a 100k-frame trace.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterator, Optional

LINKTYPE_ETHERNET = 1
LINKTYPE_RADIOTAP = 127

# magic -> (byte order, timestamp fraction unit in ns)
_PCAP_MAGICS = {
    b"\xd4\xc3\xb2\xa1": ("<", 1000), b"\xa1\xb2\xc3\xd4": (">", 1000),   # microseconds
    b"\x4d\x3c\xb2\xa1": ("<", 1), b"\xa1\xb2\x3c\x4d": (">", 1),          # nanoseconds
}
PCAPNG_MAGIC = b"\x0a\x0d\x0d\x0a"


@dataclass
class Record:
    t_ns: int
    incl: int
    orig: int
    data: bytes


def pcap_linktype(path: Path | str) -> int:
    with open(path, "rb") as f:
        hdr = f.read(24)
    if hdr[:4] == PCAPNG_MAGIC:
        raise ValueError(f"{path}: pcapng is not supported; capture with tcpdump -w (classic pcap)")
    try:
        e, _ = _PCAP_MAGICS[hdr[:4]]
    except KeyError:
        raise ValueError(f"{path}: not a pcap file (magic {hdr[:4].hex()})") from None
    return struct.unpack(e + "I", hdr[20:24])[0]


def records(path: Path | str) -> Iterator[Record]:
    """Every record of a classic pcap, timestamps in ns (CLOCK_REALTIME in
    the VM, the same clock the stubs' CSVs use)."""
    with open(path, "rb") as f:
        hdr = f.read(24)
        if hdr[:4] == PCAPNG_MAGIC:
            raise ValueError(f"{path}: pcapng is not supported; capture with tcpdump -w (classic pcap)")
        try:
            e, frac_ns = _PCAP_MAGICS[hdr[:4]]
        except KeyError:
            raise ValueError(f"{path}: not a pcap file (magic {hdr[:4].hex()})") from None
        rec = struct.Struct(e + "IIII")
        while True:
            h = f.read(16)
            if len(h) < 16:
                return
            sec, frac, incl, orig = rec.unpack(h)
            data = f.read(incl)
            if len(data) < incl:
                return  # truncated tail (capture killed mid-write): keep what we have
            yield Record(sec * 1_000_000_000 + frac * frac_ns, incl, orig, data)


# ---- radiotap ---------------------------------------------------------------

# bit -> (alignment, size). Only the fields in the first "present" word;
# anything past A-MPDU (bit 20) stops the walk ("best effort": rate stays 0).
_RT_FIELDS = {
    0: (8, 8), 1: (1, 1), 2: (1, 1), 3: (2, 4), 4: (2, 2), 5: (1, 1), 6: (1, 1), 7: (2, 2),
    8: (2, 2), 9: (2, 2), 10: (1, 1), 11: (1, 1), 12: (1, 1), 13: (1, 1), 14: (2, 2),
    15: (2, 2), 16: (1, 1), 17: (1, 1), 18: (4, 8), 19: (1, 3), 20: (4, 8),
}


@dataclass
class Radiotap:
    length: int
    tsft: Optional[int] = None
    flags: int = 0
    rate_mbps: float = 0.0   # the kernel's REQUESTED rate at hwsim handoff; 0 = unknown
    freq_mhz: int = 0
    mcs: Optional[int] = None


def radiotap(b: bytes) -> Radiotap:
    if len(b) < 8 or b[0] != 0:
        return Radiotap(0)
    length = struct.unpack("<H", b[2:4])[0]
    present = struct.unpack("<I", b[4:8])[0]
    off = 8
    while b[off - 4] & 0x80 and off + 4 <= length:  # bit 31: another present word
        off += 4
    rt = Radiotap(length)
    for bit in range(0, 21):
        if not present & (1 << bit):
            continue
        align, size = _RT_FIELDS[bit]
        off = (off + align - 1) // align * align
        if off + size > length:
            break
        v = b[off:off + size]
        if bit == 0:
            rt.tsft = struct.unpack("<Q", v)[0]
        elif bit == 1:
            rt.flags = v[0]
        elif bit == 2:
            rt.rate_mbps = v[0] * 0.5
        elif bit == 3:
            rt.freq_mhz = struct.unpack("<H", v[:2])[0]
        elif bit == 19:
            rt.mcs = v[2]
        off += size
    return rt


# ---- 802.11 -----------------------------------------------------------------

TYPE_MGMT, TYPE_CTRL, TYPE_DATA = 0, 1, 2
SUBTYPE_ACK = 13
ETHERTYPE_BATMAN = 0x4305
ETHERTYPE_IPV4 = 0x0800
ETHERTYPE_ARP = 0x0806
_LLC_SNAP = b"\xaa\xaa\x03"


def mac(b: bytes) -> str:
    return ":".join(f"{x:02x}" for x in b[:6])


@dataclass
class Dot11:
    fc: int
    type: int
    subtype: int
    flags: int
    addr1: str = ""        # RA
    addr2: str = ""        # TA
    addr3: str = ""
    addr4: str = ""
    seq: int = -1
    mesh_ttl: Optional[int] = None
    mesh_seq: Optional[int] = None
    ethertype: Optional[int] = None
    payload: int = -1      # offset of the bytes after LLC/SNAP (frame-relative)
    truncated: bool = False

    @property
    def retry(self) -> bool:
        return bool(self.flags & 0x08)

    @property
    def is_ack(self) -> bool:
        return self.type == TYPE_CTRL and self.subtype == SUBTYPE_ACK

    @property
    def broadcast(self) -> bool:
        return self.addr1 == "ff:ff:ff:ff:ff:ff" or self.addr1[:2] in ("01", "33")


def dot11(b: bytes) -> Dot11:
    fc = b[0] | b[1] << 8
    d = Dot11(fc, (fc >> 2) & 3, (fc >> 4) & 0xF, b[1])
    if d.type == TYPE_CTRL:
        d.addr1 = mac(b[4:10])
        if len(b) >= 16 and d.subtype != SUBTYPE_ACK:
            d.addr2 = mac(b[10:16])
        return d
    if len(b) < 24:
        d.truncated = True
        return d
    d.addr1, d.addr2, d.addr3 = mac(b[4:10]), mac(b[10:16]), mac(b[16:22])
    d.seq = struct.unpack("<H", b[22:24])[0] >> 4
    hdr = 24
    if d.type == TYPE_DATA:
        if d.flags & 0x03 == 0x03:  # ToDS + FromDS: 4-address (mesh unicast)
            d.addr4 = mac(b[24:30])
            hdr = 30
        if d.subtype & 0x08:        # QoS data
            hdr += 2
        if d.flags & 0x80:          # +HTC (order bit on a QoS frame)
            hdr += 4
        # 802.11s mesh control (flags, ttl, seq LE32, + 6 B per extra
        # address). Mesh data frames from these interfaces always carry it,
        # whatever the QoS "mesh control present" bit says (the unicast
        # sample has the bit clear and the header present), so test for the
        # LLC/SNAP prefix instead of trusting the bit.
        if len(b) >= hdr + 6 and b[hdr:hdr + 3] != _LLC_SNAP:
            ext = b[hdr] & 0x03
            d.mesh_ttl = b[hdr + 1]
            d.mesh_seq = struct.unpack("<I", b[hdr + 2:hdr + 6])[0]
            hdr += 6 + 6 * ext
        if len(b) >= hdr + 8 and b[hdr:hdr + 3] == _LLC_SNAP:
            d.ethertype = struct.unpack(">H", b[hdr + 6:hdr + 8])[0]
            d.payload = hdr + 8
        else:
            d.truncated = len(b) < hdr + 8
    return d


# ---- batman-adv (IV, version 15) ----------------------------------------------

BAT_IV_OGM, BAT_BCAST, BAT_OGM2, BAT_UNICAST, BAT_UNICAST_FRAG = 0x00, 0x01, 0x04, 0x40, 0x41
BAT_UNICAST_4ADDR, BAT_ICMP, BAT_UNICAST_TVLV = 0x42, 0x43, 0x44
BAT_NAMES = {BAT_IV_OGM: "ogm", BAT_BCAST: "bcast", BAT_OGM2: "ogm2", BAT_UNICAST: "unicast",
             BAT_UNICAST_FRAG: "frag", BAT_UNICAST_4ADDR: "unicast4", BAT_ICMP: "bat_icmp",
             BAT_UNICAST_TVLV: "tvlv"}
BATADV_TTL = 50  # batman-adv default for unicast: hop index = 50 - ttl


@dataclass
class Batman:
    ptype: int
    version: int
    ttl: int
    dest: str = ""         # unicast family: final originator (radio MAC)
    orig: str = ""         # OGM / broadcast: originator
    seqno: int = -1
    inner: int = -1        # offset of the inner Ethernet frame, -1 if none
    ogm_count: int = 0     # aggregated OGMs in one frame
    truncated: bool = False

    @property
    def name(self) -> str:
        return BAT_NAMES.get(self.ptype, f"bat{self.ptype:02x}")


def batman(b: bytes, off: int) -> Batman:
    if len(b) < off + 2:
        return Batman(-1, 0, 0, truncated=True)
    # Type and version first: a 64-byte snaplen leaves exactly these two
    # bytes of an OGM, enough to say what it was even if not what it said.
    t, ver = b[off], b[off + 1]
    if len(b) < off + 4:
        return Batman(t, ver, 0, truncated=True)
    bm = Batman(t, ver, b[off + 2])
    if t == BAT_IV_OGM:
        # 24 B each: type ver ttl flags seqno(4) orig(6) prev_sender(6) rsvd tq tvlv_len(2)
        p = off
        while len(b) >= p + 24:
            if bm.ogm_count == 0:
                bm.seqno = struct.unpack(">I", b[p + 4:p + 8])[0]
                bm.orig = mac(b[p + 8:p + 14])
            bm.ogm_count += 1
            p += 24 + struct.unpack(">H", b[p + 22:p + 24])[0]
        bm.truncated = bm.ogm_count == 0
    elif t == BAT_BCAST:
        if len(b) >= off + 14:
            bm.seqno = struct.unpack(">I", b[off + 4:off + 8])[0]
            bm.orig = mac(b[off + 8:off + 14])
            bm.inner = off + 14
        else:
            bm.truncated = True
    elif t in (BAT_UNICAST, BAT_UNICAST_4ADDR):
        hl = 10 if t == BAT_UNICAST else 18
        if len(b) >= off + 10:
            bm.dest = mac(b[off + 4:off + 10])
            if t == BAT_UNICAST_4ADDR and len(b) >= off + 16:
                bm.orig = mac(b[off + 10:off + 16])
            bm.inner = off + hl
        else:
            bm.truncated = True
    elif t == BAT_UNICAST_FRAG:
        if len(b) >= off + 20:
            bm.dest, bm.orig = mac(b[off + 4:off + 10]), mac(b[off + 10:off + 16])
            bm.seqno = struct.unpack(">H", b[off + 16:off + 18])[0]
        else:
            bm.truncated = True
    elif t == BAT_ICMP:
        if len(b) >= off + 20:
            bm.dest, bm.orig = mac(b[off + 4:off + 10]), mac(b[off + 10:off + 16])
            bm.seqno = struct.unpack(">H", b[off + 18:off + 20])[0]
        else:
            bm.truncated = True
    elif t == BAT_UNICAST_TVLV:
        if len(b) >= off + 16:
            bm.dest, bm.orig = mac(b[off + 4:off + 10]), mac(b[off + 10:off + 16])
        else:
            bm.truncated = True
    return bm


# ---- Ethernet / IPv4 / UDP / TCP / ICMP ---------------------------------------

@dataclass
class Inner:
    """What an inner (post-batman) or bat0 Ethernet frame carries."""
    eth_dst: str = ""
    eth_src: str = ""
    ethertype: int = 0
    proto: int = 0         # IPv4 protocol: 1 icmp, 6 tcp, 17 udp
    ip_src: str = ""
    ip_dst: str = ""
    sport: int = 0
    dport: int = 0
    l4_payload: int = -1   # offset of the UDP/TCP payload, -1 if none
    l4_len: int = 0        # bytes of UDP/TCP payload (TCP: segment payload)
    tcp_flags: int = 0
    tcp_seq: int = 0
    icmp_type: int = -1
    icmp_id: int = 0
    icmp_seq: int = 0
    truncated: bool = False


def ipv4(b: bytes, off: int, out: Inner) -> Inner:
    if len(b) < off + 20 or b[off] >> 4 != 4:
        out.truncated = True
        return out
    ihl = (b[off] & 0x0F) * 4
    total = struct.unpack(">H", b[off + 2:off + 4])[0]
    out.proto = b[off + 9]
    out.ip_src = ".".join(str(x) for x in b[off + 12:off + 16])
    out.ip_dst = ".".join(str(x) for x in b[off + 16:off + 20])
    p = off + ihl
    if out.proto == 17 and len(b) >= p + 8:
        out.sport, out.dport, ulen = struct.unpack(">HHH", b[p:p + 6])
        out.l4_payload, out.l4_len = p + 8, max(0, ulen - 8)
    elif out.proto == 6 and len(b) >= p + 20:
        out.sport, out.dport = struct.unpack(">HH", b[p:p + 4])
        out.tcp_seq = struct.unpack(">I", b[p + 4:p + 8])[0]
        doff = (b[p + 12] >> 4) * 4
        out.tcp_flags = b[p + 13]
        out.l4_payload, out.l4_len = p + doff, max(0, total - ihl - doff)
    elif out.proto == 1 and len(b) >= p + 8:
        out.icmp_type = b[p]
        out.icmp_id, out.icmp_seq = struct.unpack(">HH", b[p + 4:p + 8])
    else:
        out.truncated = True
    return out


def ethernet(b: bytes, off: int) -> Inner:
    out = Inner()
    if len(b) < off + 14:
        out.truncated = True
        return out
    out.eth_dst, out.eth_src = mac(b[off:off + 6]), mac(b[off + 6:off + 12])
    out.ethertype = struct.unpack(">H", b[off + 12:off + 14])[0]
    if out.ethertype == ETHERTYPE_IPV4:
        ipv4(b, off + 14, out)
    return out


# ---- UDP Status prototype (stubs/status_stub.py) ----------------------------

STATUS_MAGIC = 0x5342
STATUS_UDP_PORT = 7450
ZENOH_TCP_PORT = 7447


@dataclass
class Status:
    version: int
    robot_id: int
    seq: int
    stamp_ns: int


def status_cdr(b: bytes, off: int, n: int) -> Optional[Status]:
    """`struct.pack("!HHI", 0x5342, 1, len)` + CDR_LE Status: encapsulation
    `00 01 00 00`, then FleetHeader{version u8 @4, robot_id u8 @5, pad 2,
    seq u32 @8, stamp.sec i32 @12, stamp.nanosec u32 @16}. Verified against
    a captured frame (robot 0, seq 20) on 2026-10-20."""
    if n < 28 or len(b) < off + 28:
        return None
    magic, topic, ln = struct.unpack("!HHI", b[off:off + 8])
    if magic != STATUS_MAGIC or topic != 1 or b[off + 8:off + 10] != b"\x00\x01":
        return None
    c = off + 8
    version, robot_id = b[c + 4], b[c + 5]
    seq, sec, nsec = struct.unpack("<IiI", b[c + 8:c + 20])
    return Status(version, robot_id, seq, sec * 1_000_000_000 + nsec)
