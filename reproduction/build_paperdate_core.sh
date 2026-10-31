#!/usr/bin/env bash
# Isolated build of the paper-date public core. Does not install a wheel.
set -euo pipefail
CSDECOMP_REPRO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PAPERDATE_SOURCE="$CSDECOMP_REPRO_ROOT/reproduction/vendor/csdecomp_paperdate"
PAPERDATE_BAZEL="$CSDECOMP_REPRO_ROOT/reproduction/vendor/paperdate_build_tools/bazel-7.2.1-linux-x86_64"
PAPERDATE_OUTPUT="$CSDECOMP_REPRO_ROOT/reproduction/vendor/paperdate_bazel_output"
PAPERDATE_STAGE="$CSDECOMP_REPRO_ROOT/reproduction/vendor/paperdate_python/pycsdecomp"
PAPERDATE_DISTDIR="$CSDECOMP_REPRO_ROOT/reproduction/vendor/paperdate_distdir"
PAPERDATE_CUDA="${PAPERDATE_CUDA:-/usr/local/cuda-12.8}"
PAPERDATE_JOBS="${PAPERDATE_JOBS:-4}"
test -x "$PAPERDATE_BAZEL"
test -f "$PAPERDATE_SOURCE/MODULE.bazel"
test -x "$PAPERDATE_CUDA/bin/nvcc"

command_args=(
  "$PAPERDATE_BAZEL"
  --ignore_all_rc_files
  "--output_base=$PAPERDATE_OUTPUT"
  build
  //csdecomp/src/pybind/pycsdecomp:pycsdecomp_bindings
  -c opt
  "--jobs=$PAPERDATE_JOBS"
  "--distdir=$PAPERDATE_DISTDIR"
  "--repo_env=CUDA_PATH=$PAPERDATE_CUDA"
  "--action_env=CUDA_PATH=$PAPERDATE_CUDA"
  --@rules_cuda//cuda:archs=sm_75
)
if [[ "${1:-}" == --print-command ]]; then
  printf '%q ' "${command_args[@]}"
  printf '\n'
  exit 0
fi

cd "$PAPERDATE_SOURCE"
"${command_args[@]}"
mkdir -p "$PAPERDATE_STAGE"
cp csdecomp/src/pybind/pycsdecomp/__init__.py "$PAPERDATE_STAGE/"
cp csdecomp/src/pybind/pycsdecomp/drake_csd_bridge.py "$PAPERDATE_STAGE/"
cp bazel-bin/csdecomp/src/pybind/pycsdecomp/pycsdecomp_bindings.so "$PAPERDATE_STAGE/"
printf 'Isolated package staged at %s\n' "$PAPERDATE_STAGE"
