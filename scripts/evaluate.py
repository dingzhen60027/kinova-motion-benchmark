#!/usr/bin/env python3
"""Evaluation entry point reusing the existing planners and validator."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from benchmarks.generic_motion_v1.cli import evaluate_main

if __name__ == '__main__':
    evaluate_main()
