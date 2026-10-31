# Executed commands: historical Forest v1

For the current Forest/FR3 protocol see [run commands](paper_protocol_v3_commands.json) and [verification commands](paper_protocol_verification_commands.json). This historical ledger remains unchanged in substance.

Working directory: `/home/eric/wjg/wd/csdecomp`. All entries below completed with exit status 0.

The full Forest run took 647.8 seconds in the case driver. Failure states are scientific outcomes and remain in the result files.

## experiment_dependencies

```bash
bash local/python.sh -m pip install --no-deps pybezier==0.1.4 ./reproduction/vendor/scsplanning cvxpy==1.7.5 clarabel==0.11.1 gurobipy
```

## numerical_gpu_tests

```bash
bash local/python.sh -m pytest -q reproduction/tests
```

Evidence: `logs/reproduction_tests_final.log`.

7 passed in 0.60s.

## forest_full

```bash
bash local/python.sh -m reproduction.run run
```

Evidence: `logs/forest_full.log`.

400 cases, 391 eligible, 1955 method attempts.

## kinova_auxiliary

```bash
bash local/python.sh -m reproduction.kinova
```

Evidence: `logs/kinova_auxiliary.log`.

## forest_throughput

```bash
bash local/python.sh -m reproduction.throughput
```

## nto_control_count_ablation

```bash
bash local/python.sh -m reproduction.ablation --run-dir reproduction/runs/20261025T063552.273227Z_forest
```

## summarize

```bash
bash local/python.sh -m reproduction.analyze --run-dir reproduction/runs/20261025T063552.273227Z_forest --output reproduction/reports/forest
```

Evidence: `reports/forest/summary.json`.

## plot_and_replay

```bash
bash local/python.sh -m reproduction.plot_results --run-dir reproduction/runs/20261025T063552.273227Z_forest --summary-dir reproduction/reports/forest --output reproduction/figures
```

Evidence: `figures/figure_sources.json`.

## artifact_audit

```bash
bash local/python.sh -m reproduction.audit --run-dir reproduction/runs/20261025T063552.273227Z_forest --output reproduction/reports/audit
```

Evidence: `reports/audit/audit.json`.

## resume_immutability

```bash
bash local/python.sh -m reproduction.run resume --run-dir reproduction/runs/20261025T063552.273227Z_forest
```

## existing_environment_setup

```bash
bash reproduction/setup.sh
```

Evidence: `environment.json`.

Reused Conda prefix; pip check passed; 7 GPU/numerical tests passed in 0.61s.

## final_compile

```bash
bash local/python.sh -m compileall -q reproduction
```

## final_pip_check

```bash
bash local/python.sh -m pip check
```

Evidence: `logs/pip_check_final.log`.
