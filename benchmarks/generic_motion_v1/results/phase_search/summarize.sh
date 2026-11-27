#!/usr/bin/env bash
set -euo pipefail
cd "$(git -C "$(dirname "$0")" rev-parse --show-toplevel)"
base=runs/phase_search
verify_and_summarize() {
  local stage="$1"
  while ! test -f "$base/$stage/verification.json"; do
    if ! tmux has-session -t kinova-phase-search 2>/dev/null; then
      echo "Evaluation coordinator stopped before $stage was verified."
      exit 1
    fi
    sleep 15
  done
  bash local/python.sh scripts/compare_candidate.py --run "$base/$stage" --output "$base/$stage-comparison" > "$base/$stage-comparison.log"
}
verify_and_summarize qualification
verify_and_summarize dev
verify_and_summarize clock_ablation
bash local/python.sh scripts/compare_candidate.py --run "$base/dev" --clock-ablation "$base/clock_ablation" --output "$base/dev-comparison" > "$base/dev-comparison.log"
verify_and_summarize test
