#!/usr/bin/env bash
set -euo pipefail
CSDECOMP_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
unset PYTHONHOME PYTHONPATH LD_LIBRARY_PATH
export PYTHONNOUSERSITE=1
export OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-4}"
cd "$CSDECOMP_ROOT"
exec "$CSDECOMP_ROOT/.conda/envs/csdecomp/bin/python" "$@"
