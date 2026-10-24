#!/usr/bin/env python3
"""Fail CI when pytest skips something it isn't expected to skip.

`importorskip` and "map not present" skips are right for a laptop and wrong for
CI: if a dependency drops out of requirements-dev.txt, or a fixture stops being
generated, the tests vanish and the run stays green. Feed this a --junitxml
file plus the skip reasons (regex) that ARE expected; anything else fails.

    pytest --junitxml=junit.xml; scripts/check_skips.py junit.xml 'cadquery' ...
"""
import re
import sys
import xml.etree.ElementTree as ET


def main(argv: list[str]) -> int:
    if not argv:
        print(__doc__, file=sys.stderr)
        return 2
    allowed = [re.compile(p) for p in argv[1:]]
    bad, total = [], 0
    for case in ET.parse(argv[0]).getroot().iter("testcase"):
        sk = case.find("skipped")
        if sk is None:
            continue
        total += 1
        msg = sk.get("message") or ""
        if not any(p.search(msg) for p in allowed):
            bad.append(f"{case.get('classname')}::{case.get('name')}: {msg}")
    print(f"[skips] {total} skipped, {len(bad)} unexpected")
    for b in bad:
        print(f"  UNEXPECTED {b}", file=sys.stderr)
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
