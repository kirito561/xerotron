"""Backwards-compatible entry point delegating to python/main.py."""

import sys
from pathlib import Path

THIS_DIR = Path(__file__).resolve().parent
if str(THIS_DIR) not in sys.path:
    sys.path.insert(0, str(THIS_DIR))

from main import main

if __name__ == "__main__":
    raise SystemExit(main(["--hold"]))
