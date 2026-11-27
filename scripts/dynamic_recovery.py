#!/usr/bin/env python3
"""Closed-loop recovery benchmark: generate, check, run, verify and summarize."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from benchmarks.dynamic_recovery.cli import main

if __name__ == '__main__':
    raise SystemExit(main())
