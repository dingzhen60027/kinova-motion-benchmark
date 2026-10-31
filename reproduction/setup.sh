#!/usr/bin/env bash
# Restore the measured environment, or verify the existing project environment.
set -euo pipefail
CSDECOMP_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CSDECOMP_PREFIX="$CSDECOMP_ROOT/.conda/envs/csdecomp"
export CONDA_PKGS_DIRS="$CSDECOMP_ROOT/.conda/pkgs"
cd "$CSDECOMP_ROOT"
if [[ ! -x "$CSDECOMP_PREFIX/bin/python" ]]; then
    conda create --yes --prefix "$CSDECOMP_PREFIX" --file reproduction/conda-explicit.txt
    "$CSDECOMP_PREFIX/bin/python" -m pip install -r reproduction/requirements.lock.txt
fi
mkdir -p local
if [[ ! -e local/python.sh ]]; then
    cp reproduction/bootstrap/python.sh local/python.sh
elif ! cmp -s local/python.sh reproduction/bootstrap/python.sh; then
    echo 'local/python.sh differs from the measured launcher; inspect before continuing.' >&2
    exit 1
fi
bash local/python.sh -c 'import sys; assert sys.version_info[:2] == (3, 10), sys.version'
bash local/python.sh -m pip check
bash local/python.sh -m pytest -q reproduction/tests
