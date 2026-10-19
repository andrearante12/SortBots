"""splat/convert.py: 3DGS PLY <-> the dashboard's .splat format. numpy only."""
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "splat"))

import convert  # noqa: E402
import dataset  # noqa: E402


def _record(n=4):
    means = np.arange(n * 3, dtype=np.float32).reshape(n, 3)
    log_scales = np.log(np.full((n, 3), 0.1, np.float32))
    quats = np.tile(np.array([[2.0, 0, 0, 0]], np.float32), (n, 1))   # unnormalised w
    sh0 = np.zeros((n, 1, 3), np.float32)                              # colour 0.5
    opac = np.full(n, 10.0, np.float32)                                # ~opaque
    return convert.to_ply_record(means, log_scales, quats, sh0, opac)


def test_splat_record_is_32_bytes_in_the_viewers_layout():
    s = convert.ply_to_splat(_record(1))
    raw = s.tobytes()
    assert len(raw) == 32
    pos = np.frombuffer(raw[0:12], "<f4")
    scale = np.frombuffer(raw[12:24], "<f4")
    np.testing.assert_allclose(pos, [0, 1, 2])
    np.testing.assert_allclose(scale, [0.1, 0.1, 0.1], rtol=1e-6)   # exp of log
    assert list(raw[24:27]) == [128, 128, 128]                      # 0.5 + C0*0
    assert raw[27] == 255                                           # sigmoid(10)
    assert list(raw[28:32]) == [255, 128, 128, 128]                 # w=1 after normalise


def test_ply_round_trip_through_the_reader(tmp_path):
    rec = _record()
    convert.write_ply(tmp_path / "m.ply", rec)
    back = dataset.read_ply_vertices(tmp_path / "m.ply")
    for k in rec.dtype.names:
        np.testing.assert_array_equal(back[k], rec[k])


def test_prunes_transparent_huge_and_nonfinite():
    rec = _record(4)
    rec["opacity"][0] = -20.0          # invisible
    rec["scale_0"][1] = np.log(5.0)    # 5 m floater
    rec["x"][2] = np.nan
    s = convert.ply_to_splat(rec)
    np.testing.assert_allclose(s["pos"], [[9, 10, 11]])


def test_sorted_most_visible_first_and_capped():
    rec = _record(3)
    for i, sc in enumerate([0.01, 0.3, 0.1]):
        for a in range(3):
            rec[f"scale_{a}"][i] = np.log(sc)
    s = convert.ply_to_splat(rec)
    np.testing.assert_allclose(s["pos"][:, 0], [3, 6, 0])
    assert len(convert.ply_to_splat(rec, max_gaussians=2)) == 2


def test_cli_writes_a_splat(tmp_path):
    convert.write_ply(tmp_path / "m.ply", _record())
    assert convert.main([str(tmp_path / "m.ply"), str(tmp_path / "m.splat")]) == 0
    assert (tmp_path / "m.splat").stat().st_size == 4 * 32
