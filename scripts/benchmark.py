#!/usr/bin/env python3
"""Dataset entry point; works from any current working directory."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from benchmarks.generic_motion_v1.cli import benchmark_main

if __name__ == '__main__':
    benchmark_main()
