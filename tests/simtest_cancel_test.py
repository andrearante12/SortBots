"""scripts/simtest.py has to hear the dashboard's Stop test whatever it inherited.

    /usr/bin/python3 -m pytest tests/simtest_cancel_test.py
"""
from __future__ import annotations

import signal
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent

# Run as a child with SIGINT ignored, the way a backgrounded console's children start.
CHILD = f"""
import importlib.util, signal, sys
spec = importlib.util.spec_from_file_location("simtest_cli", {str(REPO_ROOT / "scripts/simtest.py")!r})
cli = importlib.util.module_from_spec(spec)
sys.modules["simtest_cli"] = cli
spec.loader.exec_module(cli)
assert signal.getsignal(signal.SIGINT) == signal.SIG_IGN
cli.main(["list"])
sys.exit(0 if signal.getsignal(signal.SIGINT) is signal.default_int_handler else 7)
"""


def test_an_inherited_ignored_sigint_is_undone():
    r = subprocess.run([sys.executable, "-c", CHILD], cwd=REPO_ROOT, capture_output=True, text=True,
                       preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_IGN))
    assert r.returncode == 0, r.stderr[-800:]
