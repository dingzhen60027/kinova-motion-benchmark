# Strengthened baseline acceptance: independent final audit

Audit date: 2026-10-27. Scope: `runs/v1_20261027_{forest,fr3,regression}` and `research_platform/reports/strengthened_baselines/v1_20261027`. The auditor did not edit source code, configurations, or run artifacts and did not start GPU work. This Markdown report is the only audit output written to the repository.

**Verdict: the recorded acceptance counts, shared-input comparisons, failure retention, and frozen-source identities pass this audit.** The six development cases support the narrow statement that the strengthened implementations passed more of these fixed checks. They do not establish a general success rate, a new research contribution, or reproduction of the authors' original benchmark.

## 1. What was independently checked

The audit read raw manifests, identities, dependency receipts, process records, input and seed arrays, candidate coefficients, validation records, every strengthened-method round, and report CSVs. It invoked `verify_inputs(..., current_sources=True)`, `verify_run`, and `verify_attempt` without rerunning planning. It also recomputed shared input hashes directly, evaluated candidate endpoints from their saved representations, and compared every NTO's saved original warm start and every geometric pipeline's initial path with the shared DRM seed.

All Python audit invocations used `PYTHONDONTWRITEBYTECODE=1 bash local/python.sh` and read-only inline scripts. The source modules were not imported from an alternative checkout. The Forest polynomial checker was rerun on saved candidates using CPU only; this reproduces the frozen numerical check, rather than supplying an independent exact-arithmetic proof. FR3 collision queries were not rerun in this final audit; its retained predicate arrays and validation decisions were inspected.

| Run | Cases | Planned jobs / retained attempts | Method jobs | Strict successes | Failures | Frozen source entries |
|---|---:|---:|---:|---:|---:|---:|
| Forest development, environment seeds 10–13 | 4 | 41 / 41 | 32 | 28 | 4 | 57 |
| FR3 development, environment seeds 10–11 | 2 | 21 / 21 | 16 | 12 | 4 | 55 |
| Figure 6 regression, environment seed 0 | 1 | 11 / 11 | 8 | 6 | 2 | 57 |

Each job has exactly one sealed attempt. All 73 receipts and their 1,292 inventoried member files verified; including the receipts, there are 1,365 retained job files. No retry or best-attempt selection enters these counts. All three runs report `integrity_passed=true` and `complete=true`; all three correctly report `all_valid=false` because numerical failures remain.

Every frozen source entry matches the current working tree at audit time. The 55 source entries common to all three runs have identical hashes. Forest's two additional entries are its environment-specific source assets, not a changed solver. The source snapshots include `strong_nto.py` and `continuous_checks.py`.

## 2. Shared task identity and acceptance

For each of the seven cases, all eight methods have the same nonempty `input_hash`. The auditor recomputed that hash from the resolved protocol identity, shared `input.json`, and shared `seed_path.npz`, and checked the selected planning receipt in each method's dependency lineage. NTO's `warm_start_original_nodes` and the geometric methods' saved initial path are byte-value equivalent to the shared DRM array. No method-specific IK or substituted endpoint was found.

Endpoint evaluation from the saved polygon, Bézier, or B-spline representation stays below the common `1e-5` endpoint tolerance for every candidate, including failures. Distance methods retain distance objectives and a path-parameter axis; time methods retain duration in seconds, the same velocity/acceleration bounds, and stationary-endpoint checks. Report strict-success fields agree with the raw three-part rule: successful terminal status, successful solver, and passed independent validation. No failed solver is counted as success because its candidate is finite or collision-free.

All 40 Forest method candidates, including the eight Figure 6 regression candidates, contain the common continuous-parameter polynomial geometry check. CPU recomputation reproduced every resolved/collision decision and every recorded minimum clearance exactly (`maximum absolute difference = 0.0` in this audit). This is a float64 stationary-point check, not an interval or exact-arithmetic certificate. FR3 remains a sampled check against the matched sphere/box model. The two environments must not be described as having the same continuous safety guarantee.

The strong methods' 1 mm clearance is an **internal optimization target**, separate from the shared zero-penetration acceptance condition. Their endpoint audits all report compatibility with this target; none changed endpoints. FR3 records `not_available_on_full_curve` when a full-curve minimum clearance is unavailable. Therefore a successful FR3 strong run does not establish continuous 1 mm clearance.

For all 14 strong-method executions, final candidate control points and knots match the last saved round, and final validation equals that round's saved validation. All 30 round directories retain their solver parameters, candidate arrays, solver details, validation, trajectory samples, and status record. No stale validation from a preceding iterate was found.

## 3. Budget and timing interpretation

Every recorded process has the common 120 s external job watchdog. Every strong-method manifest declares the 30 s cumulative internal scope; successive solver limits decrease rather than reset. All accepted strong candidates completed within this internal budget. The largest successful strong cumulative time was approximately 0.933 s; the retained FR3 distance failure consumed 21.941950 s internally and 22.604857 s of external process wall time over six rounds.

These facts must not be reported as a uniform 30 s hard deadline for all methods. The old NTO and LGCS options have solver-specific meanings, while geometric recovery can call a solver repeatedly and SCSTO relies on the outer watchdog. Native solver work is not asynchronously interrupted at exactly the strong internal deadline; late completion is checked and rejected before success. The common forced process boundary is the external watchdog, with its documented cleanup overhead.

Method wall time excludes shared scene/roadmap/planning jobs. Cost means measure trajectory quality, not computation time. In particular, a shorter executed trajectory does not imply a faster planner. The summary must not call method-only timing full end-to-end latency.

## 4. Counts and paired quality comparisons

The following counts were recomputed from raw terminal receipts and result flags, excluding Figure 6 regression:

| Method | Forest development | FR3 development | Combined fixed development cases |
|---|---:|---:|---:|
| DRM | 4/4 | 2/2 | 6/6 |
| LSCS | 4/4 | 2/2 | 6/6 |
| LGCS | 4/4 | 2/2 | 6/6 |
| SCSTO | 4/4 | 2/2 | 6/6 |
| NTO_distance | 2/4 | 0/2 | 2/6 |
| NTO_distance_strong | 4/4 | 1/2 | 5/6 |
| NTO_time | 2/4 | 1/2 | 3/6 |
| NTO_time_strong | 4/4 | 2/2 | 6/6 |

Thus **distance 2/6 → 5/6 and time 3/6 → 6/6 are correct descriptive counts for this six-case development set**. Combined counts are useful bookkeeping, not an estimated population success probability. Report Forest and FR3 separately alongside the combined total because they differ in model, dimension, and verification scope. The extra Figure 6 run has old distance/time 0/1 and strong distance/time 1/1 and must remain separate.

Success-only method means cannot be directly compared when their successful cases differ. The actual common-success comparisons are:

- Strong versus old distance: two common cases, both Forest e010/e013; both length ratios are exactly 1.0. There are no common successful FR3 distance cases. The five-versus-two success result does not establish a mean path-length improvement over old NTO.
- Strong versus old time: three common cases. Strong/old duration ratios are approximately 0.380110 for Forest e010, 0.380110 for Forest e013, and 0.817213 for FR3 e010. Comparing the old three-success mean with the strong six-success mean would mix different tasks.
- Strong time versus SCSTO: all six development cases are common successes. Strong/SCSTO duration ratios are 0.760220, 0.949123, 0.831694, 0.760220 for Forest e010–e013 and 0.945646, 0.833571 for FR3 e010–e011. These are observations on six tasks, not evidence of a general ranking or computational speedup.

## 5. Similar Forest costs do not indicate a broken seed

Forest e010 and e013 share the protocol's fixed endpoints and both have a two-node straight DRM path, but their obstacle scenes are different. The saved center arrays were regenerated with the declared `np.random.default_rng(environment_seed)` sampling rule: seeds 10, 11, 12, and 13 all match their saved arrays exactly. For e010/e013 the largest center-coordinate difference is 5.8738187948 m. Their center-array hashes are respectively `1132b2304b72a261b2071134b98a1e1f1cd771e63be9f67be990d7ff99fdfd32` and `955f6daf08c2aa84822fb2004fdbe448e414dab5556fe1979b8d7192cb42abd9`.

The identical straight DRM has minimum clearance 0.3691284752 m in e010 and 0.0703328470 m in e013. The old four-control-point time trajectories have identical control arrays and duration 23.3160903454 s. This explains their identical old-NTO cost without any failed scene randomization.

SCSTO's saved control arrays differ by up to 0.1709647006, although their durations differ by only 1.75e-8 s. Strong time uses 16 controls and one solve in e010, versus 24 controls and two solves in e013. Their sampled normalized-progress curves differ by up to 0.1237359840 in an individual coordinate; minimum clearances are approximately 0.3459724130 m and 0.0010000000 m. Their durations differ by approximately 6.33e-11 s. Hence these are different trajectories/constraint interactions with almost equal time cost, not duplicated candidates. The exact optimization explanation for that near equality is not established here. These relatively easy straight-seed cases remain in the fixed denominator and are not replaced to make the benchmark look harder.

## 6. Retained failures and limits

The eight development failures are Forest e011 old distance/time, Forest e012 old distance/time, FR3 e010 old distance, and FR3 e011 old distance/strong distance/old time. Figure 6 adds old distance validation failure and old time solver failure. Each has candidate JSON/NPZ, validation, process logs, and an intact receipt. The strong FR3 distance failure retains six rounds and is still `solver_failed`; its diagnostic investigations do not overwrite or upgrade the original run.

No missing methods, removed cases, silent retries, weakened strong-method collision predicate, or mismatched saved validation was found. The audit does not claim that numerical tests exhaust all bugs, that the reconstructed FR3 model equals the authors' private asset, that these baselines are globally optimal, or that this small implementation acceptance constitutes a formal paper evaluation.

## Evidence entry points

- [Frozen acceptance protocol](v1_20261027/protocol.json), [actual commands](v1_20261027/commands.json), [recorded checks](v1_20261027/checks.json).
- [Forest raw run](../../../runs/v1_20261027_forest/manifest.json), [summary](../../../runs/v1_20261027_forest/reports/summary.csv), [paired comparisons](../../../runs/v1_20261027_forest/reports/paired.csv).
- [FR3 raw run](../../../runs/v1_20261027_fr3/manifest.json), [summary](../../../runs/v1_20261027_fr3/reports/summary.csv), [paired comparisons](../../../runs/v1_20261027_fr3/reports/paired.csv).
- [Separate Figure 6 regression](../../../runs/v1_20261027_regression/manifest.json).
