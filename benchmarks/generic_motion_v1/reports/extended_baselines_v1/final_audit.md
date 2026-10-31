# Extended baseline independent final audit

Passed: **True**

Source commit: `3414dd08f165e0cee0856e86007b53d9b6e97720`

Checks: 222397; errors: 0.

| Quantity | Count |
|---|---:|
| candidate_validations | 732 |
| candidates | 732 |
| checks | 222397 |
| dependency_files | 1845 |
| frontend_validations | 460 |
| frozen_data_files | 99 |
| frozen_roadmap_files | 15 |
| invalid_slots | 12 |
| legal_slots | 1128 |
| model_asset_files | 6 |
| new_test_execution_records | 0 |
| preexisting_test_runs | 24 |
| raw_bytes | 403702799 |
| raw_files | 34178 |
| receipts | 1140 |
| registered_slots | 1140 |
| source_snapshot_files | 220 |
| validated_successes | 717 |

| Method | Registered | Legal | Invalid | Success | Candidate | Statuses |
|---|---:|---:|---:|---:|---:|---|
| LSCS | 190 | 188 | 2 | 113 | 113 | {'frontend_failed': 75, 'invalid_endpoint': 2, 'success_validated_numerical': 113} |
| RRTConnect | 190 | 188 | 2 | 153 | 154 | {'success_validated_numerical': 153, 'invalid_endpoint': 2, 'planner_timeout': 34, 'validation_undetermined': 1} |
| RRT* | 190 | 188 | 2 | 126 | 126 | {'planner_timeout': 62, 'success_validated_numerical': 126, 'invalid_endpoint': 2} |
| CHOMP | 190 | 188 | 2 | 112 | 113 | {'frontend_failed': 75, 'invalid_endpoint': 2, 'success_validated_numerical': 112, 'solver_failed': 1} |
| STOMP | 190 | 188 | 2 | 113 | 113 | {'frontend_failed': 75, 'invalid_endpoint': 2, 'success_validated_numerical': 113} |
| TrajOpt | 190 | 188 | 2 | 100 | 113 | {'frontend_failed': 75, 'invalid_endpoint': 2, 'success_validated_numerical': 100, 'collision_failed': 13} |

| Check category | Checks | Failed |
|---|---:|---:|
| attempts | 1140 | 0 |
| candidate_evidence | 3336 | 0 |
| candidate_numeric | 21341 | 0 |
| classification | 1140 | 0 |
| completion | 12 | 0 |
| data_files | 99 | 0 |
| external_dependency_files | 1845 | 0 |
| freeze | 8 | 0 |
| frontend_evidence | 460 | 0 |
| frontend_numeric | 13340 | 0 |
| frontend_pairing | 1900 | 0 |
| input_identity | 4560 | 0 |
| invalid_inputs | 12 | 0 |
| matrix | 17 | 0 |
| metric_cluster_intervals | 528 | 0 |
| metric_groups | 1201 | 0 |
| metric_rows | 38592 | 0 |
| metric_statistics | 30492 | 0 |
| metrics | 23 | 0 |
| model_assets | 6 | 0 |
| model_identity | 2280 | 0 |
| pairing_seeds | 10260 | 0 |
| protocol | 4 | 0 |
| raw_hashes | 34178 | 0 |
| receipts | 36458 | 0 |
| result_identity | 9120 | 0 |
| result_numeric | 4392 | 0 |
| roadmap_files | 15 | 0 |
| roadmap_identity | 32 | 0 |
| run_identity | 44 | 0 |
| runtime_hashes | 55 | 0 |
| runtime_identity | 1140 | 0 |
| snapshot_hashes | 220 | 0 |
| source_inventory | 5 | 0 |
| success_truth | 717 | 0 |
| test_protection | 1 | 0 |
| timing | 3420 | 0 |
| totals | 4 | 0 |

## Evidence bindings

Metrics SHA256: `a7f322c61dc9ba41c193c0a9880e739c56f3068f25cfd57b15a6ff16102a962d`
Execution freeze SHA256: `ea2a06af6444f36421f16333136322b1d6cf2d830e39fbe688e83cdb506b6273`
Run index SHA256: `c5bae13d610138c96ca078fd9782aae750a0a5fa22bd4d1c0482c8c9be492c16`

## Errors

None.

## Audit limits

- Collision checks are audited from saved native boolean and signed-distance arrays; no fresh geometry or GPU calls are made.
- Finite numerical collision sampling is not a proof of continuous collision freedom.
- No-new-test-call check binds frozen test input hashes and saved process evidence; it cannot detect unlogged external executions.
- Native solver convergence is separate from valid returned trajectory success; a validated nonconverged incumbent may qualify.
