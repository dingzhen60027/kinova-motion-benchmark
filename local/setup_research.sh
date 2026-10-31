#!/usr/bin/env bash
# Reuse a complete project environment; create the full research stack if absent.
set -euo pipefail
CSDECOMP_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CSDECOMP_PREFIX="$CSDECOMP_ROOT/.conda/envs/csdecomp"
export CONDA_PKGS_DIRS="$CSDECOMP_ROOT/.conda/pkgs"
cd "$CSDECOMP_ROOT"
if [[ ! -x "$CSDECOMP_PREFIX/bin/python" ]]; then
    conda create --yes --prefix "$CSDECOMP_PREFIX" --file reproduction/conda-explicit.txt
    bash local/python.sh -m pip install -r reproduction/requirements.lock.txt
elif [[ "${1:-}" == "--repair" ]]; then
    bash local/python.sh -m pip install -r reproduction/requirements.lock.txt
fi
bash local/research.sh doctor --dependencies-only
echo 'Research dependencies verified. Use: bash local/research.sh doctor --probe'
