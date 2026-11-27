#!/usr/bin/env bash
set -euo pipefail
root="$(git -C "$(dirname "$0")" rev-parse --show-toplevel)"
cd "$root"
export CUDA_VISIBLE_DEVICES=0 OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
base=runs/phase_search
run_split() {
  local split="$1" target="$2" track="$3" config="$4"
  shift 4
  if [[ -f "$target/run_manifest.json" ]]; then
    bash local/python.sh scripts/evaluate.py resume --run "$target"
  else
    bash local/python.sh scripts/evaluate.py batch --split "$split" --track "$track" --methods PHASE_SEARCH --seeds "$@" --config "$config" --output "$target"
  fi
  bash local/python.sh scripts/evaluate.py verify --run "$target"
  bash local/python.sh - "$target" <<'PY'
import json,sys,collections
from pathlib import Path
p=Path(sys.argv[1]); v=json.loads((p/'verification.json').read_text())
assert v['complete'] and v['integrity_passed'],v
outcomes=collections.defaultdict(collections.Counter)
for job in json.loads((p/'run_manifest.json').read_text())['jobs']:
 for attempt in sorted((p/'jobs'/job['job_id']/'attempts').iterdir()):
  receipt=json.loads((attempt/'receipt.json').read_text())
  if not receipt.get('retry_pending'):
   outcomes[job['objective']][receipt['status']]+=1
   break
assert all(c['success_validated_numerical']>0 and not (set(c)&{'unsupported','infrastructure_error','model_consistency_failed'}) for c in outcomes.values()),outcomes
print(p,dict(outcomes),flush=True)
PY
}
run_split smoke "$base/qualification" both configs/evaluation.json 0
run_split dev "$base/dev" both configs/evaluation.json 0 1 2
run_split dev "$base/clock_ablation" time "$base/clock_ablation.json" 0
run_split test "$base/test" both configs/evaluation.json 0 1 2
