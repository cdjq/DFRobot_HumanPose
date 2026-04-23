#!/usr/bin/env python3
"""
Demo entry: run the Python driver unit test suite (no sensor, no PinPong on PC).

Usage (from ``python/raspberrypi``):

  python examples/run_unit_tests_demo.py

Or:

  python -m unittest discover -s tests -p "test_*.py" -v
"""

from __future__ import annotations

import os
import subprocess
import sys


def main() -> int:
  root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
  cmd = [
    sys.executable,
    "-m",
    "unittest",
    "discover",
    "-s",
    "tests",
    "-p",
    "test_*.py",
    "-v",
  ]
  return subprocess.call(cmd, cwd=root)


if __name__ == "__main__":
  raise SystemExit(main())
