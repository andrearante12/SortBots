"""webui/serve.py's /api/consoles list (configs/consoles.yaml): the header's SIM/REAL menu
turns every entry into a link, so anything but a named http(s) console of a known
platform is dropped."""
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "webui"))
import serve  # noqa: E402


def test_repo_config_lists_both_platforms():
    plats = {c["platform"] for c in serve._load_consoles()}
    assert plats == {"sim", "real"}


def test_bad_entries_dropped(tmp_path):
    p = tmp_path / "consoles.yaml"
    p.write_text(
        "consoles:\n"
        "  - {name: ok, url: 'http://x:8081/', platform: sim}\n"
        "  - {name: script, url: 'javascript:alert(1)', platform: sim}\n"
        "  - {name: moon, url: 'http://y/', platform: moon}\n"
        "  - {url: 'http://z/', platform: real}\n"
        "  - not a mapping\n")
    assert serve._load_consoles(p) == [{"name": "ok", "url": "http://x:8081/", "platform": "sim"}]


def test_missing_or_broken_file_is_empty(tmp_path):
    assert serve._load_consoles(tmp_path / "nope.yaml") == []
    bad = tmp_path / "bad.yaml"
    bad.write_text("consoles: [unclosed")
    assert serve._load_consoles(bad) == []
