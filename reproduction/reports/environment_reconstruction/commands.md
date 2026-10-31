# Actual commands and saved outputs

Executed in `/home/eric/wjg/wd/csdecomp` on 2026-10-26, using the existing
Conda launcher. Preparation and native jobs use the source hashes, Python and
package versions in `run_manifest.json`. Each archived job also records its
exact child command and output receipt in `execution.json`.

```bash
bash local/python.sh -m reproduction.environment_reconstruction.forest_reference

# Initial complete run, retained with its original source snapshot.
bash local/python.sh -m reproduction.environment_reconstruction.run prepare \
  --run reproduction/reports/environment_reconstruction/run \
  > reproduction/reports/environment_reconstruction/prepare.log 2>&1
bash local/python.sh -m reproduction.environment_reconstruction.run smoke \
  --run reproduction/reports/environment_reconstruction/run \
  > reproduction/reports/environment_reconstruction/smoke.log 2>&1

# Full repeat after fixing interrupted-job evidence preservation.
bash local/python.sh -m reproduction.environment_reconstruction.run prepare \
  --run reproduction/reports/environment_reconstruction/run_final \
  > reproduction/reports/environment_reconstruction/prepare_final.log 2>&1
bash local/python.sh -m reproduction.environment_reconstruction.run smoke \
  --run reproduction/reports/environment_reconstruction/run_final \
  > reproduction/reports/environment_reconstruction/smoke_final.log 2>&1
bash local/python.sh -m reproduction.environment_reconstruction.run verify \
  --run reproduction/reports/environment_reconstruction/run_final \
  > reproduction/reports/environment_reconstruction/verify.log 2>&1

bash local/python.sh -m pytest -q \
  reproduction/environment_reconstruction/test_fixed_forest.py \
  reproduction/environment_reconstruction/test_run_integrity.py \
  > reproduction/reports/environment_reconstruction/tests.log 2>&1

conda list --explicit -p .conda/envs/csdecomp \
  > reproduction/reports/environment_reconstruction/conda-explicit.txt

bash local/python.sh -m reproduction.environment_reconstruction.render \
  reproduction/reports/environment_reconstruction/run_final \
  reproduction/reports/environment_reconstruction/preview

bash local/python.sh -m reproduction.environment_reconstruction.bundle
```

The final `smoke` command was also executed through `subprocess.run` inside a
Python SHA audit. All 240 run files were hashed before and after: no changes.
Its actual output is `resume.log`, and its comparison is `resume_check.json`.
`initial_final_comparison.json` checks all 24 saved status/start/goal/path records
across the two runs; every compared value is identical.

CPU failure diagnosis is rerunnable without replanning:

```bash
CUDA_VISIBLE_DEVICES='' bash local/python.sh \
  reproduction/reports/environment_reconstruction/diagnose_failures.py
```

The individual geometry/source, renderer and task-semantics audit scripts and
their execution records are retained alongside their evidence. No setup or
environment replacement was performed. Preparation records actual `nvidia-smi
-q`, `uname -a` and `python -m pip freeze` outputs inside each run.

The original runner intentionally refuses a new `prepare` in an existing
directory, changed frozen inputs, changed completed outputs and direct case
overwrites. Use a new path when reproducing the commands as a new attempt.
