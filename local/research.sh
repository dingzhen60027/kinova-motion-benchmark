#!/usr/bin/env bash
set -euo pipefail
CSDECOMP_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec bash "$CSDECOMP_ROOT/local/python.sh" -m research_platform "$@"
