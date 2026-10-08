"""PHY/MAC arithmetic and propagation for the DES.

Standard 802.11 timing (OFDM / HT-mixed, 20 MHz, long GI) is physics, not a
placeholder. Everything tagged PLACEHOLDER below is a parameter the lab
testbed replaces (docs/plan.md §4): path-loss exponent, rack and shelving
attenuation, tx power, SNR thresholds, protocol overhead bytes.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

# ---- 802.11 timing (µs) -----------------------------------------------------

SLOT_US = 9.0                      # short slot; legacy 802.11b rates are off
SIFS_US = {"g24": 10.0, "g5": 16.0}
SIGNAL_EXT_US = {"g24": 6.0, "g5": 0.0}
CW_MIN, CW_MAX = 15, 1023
RETRY_LIMIT = 7
ACK_BYTES = 14
ACK_RATE_MBPS = 24.0
LEGACY_PREAMBLE_US = 20.0
HT_PREAMBLE_US = 36.0


def difs_us(band: str) -> float:
    return SIFS_US[band] + 2 * SLOT_US


def _ofdm_us(nbytes: int, rate_mbps: float, preamble_us: float, band: str) -> float:
    bits = 16 + 8 * nbytes + 6
    ndbps = rate_mbps * 4.0
    return preamble_us + 4.0 * math.ceil(bits / ndbps) + SIGNAL_EXT_US[band]


def frame_airtime_us(nbytes: int, rate_mbps: float, band: str, unicast: bool,
                     legacy: bool = False) -> float:
    """Channel occupancy of one transmission attempt, excluding DIFS/backoff.
    Unicast includes SIFS + ACK (or the equivalent ACK timeout on failure)."""
    pre = LEGACY_PREAMBLE_US if legacy else HT_PREAMBLE_US
    t = _ofdm_us(nbytes, rate_mbps, pre, band)
    if unicast:
        t += SIFS_US[band] + _ofdm_us(ACK_BYTES, ACK_RATE_MBPS, LEGACY_PREAMBLE_US, band)
    return t


# ---- frame overheads (bytes) — ESTIMATES, to be replaced by tcpdump in M1 ----

MAC_OVERHEAD = 62        # 802.11s data hdr + mesh ctl + QoS + CCMP(16) + FCS + LLC/SNAP
BATMAN_UNICAST = 24      # batman-adv unicast hdr (10) + inner Ethernet (14)
BATMAN_BCAST = 28        # batman-adv broadcast hdr + inner Ethernet
OGM_BYTES = 24           # one B.A.T.M.A.N. IV OGM
OGM_AGGREGATE_MAX = 512  # batman-adv aggregation cap per frame
IP_TCP = 52              # IPv4 + TCP with timestamps
IP_UDP = 28
# MEASURED (M1, 2026-10-03): a 64 B CDR Status rode in a 136 B TCP payload,
# so zenoh framing + rmw_zenoh attachment = 72 B per message incl. batch hdr.
ZENOH_PER_MSG = 68
ZENOH_PER_BATCH = 4
UDP_STATUS_OVERHEAD = 8  # UDP Status prototype: topic id + length ahead of the CDR bytes
CDR_ENCAP = 4
TCP_MSS = 1448
UNICAST_FRAME_OVERHEAD = MAC_OVERHEAD + BATMAN_UNICAST + IP_TCP


# ---- propagation — PLACEHOLDERS ---------------------------------------------

@dataclass
class RadioParams:
    tx_dbm: float = 20.0                 # PLACEHOLDER: router model unknown
    noise_dbm: float = -95.0
    # PLACEHOLDER. 3.0 is a generic indoor value; industrial-hall measurements
    # found near free space (~2) (arXiv:1906.12145). Swept 2.0 vs 3.0.
    pl_exponent: float = 3.0
    # Per loaded steel rack row crossed. Industry guidance, NOT peer-reviewed:
    # 15-20 dB through 20-ft steel racks (2mtechnology.net/unifi-warehouse-wifi-design),
    # 15-25 dB per row, 5 GHz worse than 2.4 (purple.ai guide). Low ends used.
    rack_db_g24: float = 15.0
    rack_db_g5: float = 20.0
    shelf_db: float = 3.0                # PLACEHOLDER per wire-shelving crossing
    gateway_rack_factor: float = 0.5     # PLACEHOLDER: gateway mounted high clears some racks
    multicast_rate_mbps: float = 12.0
    cs_dbm: float = -82.0                # carrier-sense threshold (reported, see medium.py)
    ht: float = 1.0                      # 1 = HT MCS rates; 0 = legacy OFDM (what hwsim's minstrel used)
    max_rate_mbps: float = 1e9           # cap: a conservative rate controller (emulation sat at 9-24 Mb/s)

    @classmethod
    def from_config(cls, cfg: dict) -> "RadioParams":
        r = cfg.get("radio_model", {})
        return cls(**{k: r[k] for k in cls.__dataclass_fields__ if k in r})


FREQ_MHZ = {"g24": 2437.0, "g5": 5180.0}


def fspl_1m_db(band: str) -> float:
    return 20 * math.log10(FREQ_MHZ[band]) - 27.55


def snr_db(p: RadioParams, band: str, dist_m: float, racks: int, shelves: int,
           gateway: bool = False) -> float:
    d = max(dist_m, 1.0)
    rack_db = p.rack_db_g24 if band == "g24" else p.rack_db_g5
    rack_loss = racks * rack_db * (p.gateway_rack_factor if gateway else 1.0)
    pl = fspl_1m_db(band) + 10 * p.pl_exponent * math.log10(d) + rack_loss + shelves * p.shelf_db
    return p.tx_dbm - pl - p.noise_dbm


# HT MCS0-7 (1 stream, 20 MHz, long GI): (rate Mbps, SNR needed dB) — PLACEHOLDER
# thresholds; the lab's loss-vs-SNR measurement replaces them.
HT_RATES = [(6.5, 5.0), (13.0, 8.0), (19.5, 11.0), (26.0, 14.0),
            (39.0, 17.0), (52.0, 21.0), (58.5, 23.0), (65.0, 25.0)]
LEGACY_THRESH = {6.0: 4.0, 12.0: 7.0, 24.0: 13.0}
RATE_MARGIN_DB = 2.0       # Minstrel stand-in: pick the fastest rate with this margin


def per(snr: float, thresh: float, nbytes: int) -> float:
    """Frame error rate: logistic around the rate's SNR threshold for a
    1000 B frame; shorter frames get a modest SNR credit (1.5 dB per decade
    of length). PLACEHOLDER curve shape.

    The first version scaled success as PSR_1000 ** (L/1000), which let a
    100 B frame through 32% of the time at 1.3 dB SNR — 5.7 dB below the
    12 Mb/s threshold. batman's OGM-based link quality then stayed above the
    routing cutoff and the DES routed over links that could not carry a
    single unicast frame (sim_log step 18)."""
    shift = 1.5 * math.log10(1000.0 / max(nbytes, 50))
    x = snr - (thresh - shift)
    return 1.0 / (1.0 + math.exp(2.0 * x))


# Legacy OFDM (rate, SNR needed) — PLACEHOLDER thresholds. Used by the slow-rate
# sensitivity runs and the emulation cross-check (hwsim's minstrel picked
# legacy rates). Restored 2026-10-04: the per() rewrite had deleted it.
LEGACY_RATES = [(6.0, 4.0), (9.0, 5.0), (12.0, 7.0), (18.0, 9.0), (24.0, 13.0),
                (36.0, 16.0), (48.0, 20.0), (54.0, 21.0)]


def pick_unicast_rate(snr: float, ht: bool = True, cap: float = 1e9) -> tuple[float, float] | None:
    table = [rt for rt in (HT_RATES if ht else LEGACY_RATES) if rt[0] <= cap]
    best = None
    for rate, th in table:
        if snr >= th + RATE_MARGIN_DB:
            best = (rate, th)
    if best is None and snr >= table[0][1]:
        best = table[0]
    return best


def bcast_threshold(rate: float) -> float:
    return LEGACY_THRESH.get(rate, 7.0)
