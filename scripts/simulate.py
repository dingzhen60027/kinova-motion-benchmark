#!/usr/bin/env python3
"""MuJoCo scene building, physical execution and rendering of frozen tasks."""
from pathlib import Path
import os
import sys

os.environ.setdefault('MUJOCO_GL', 'egl')
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from benchmarks.generic_motion_v1.simulation.cli import main

if __name__ == '__main__':
    raise SystemExit(main())
