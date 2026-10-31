# Sealed development benchmark results

Run: `/home/eric/wjg/wd/csdecomp/benchmarks/generic_motion_v1/results/dev_eizo_v2`; source commit `5dc135f67f0ec2817fdd7da93260f8e2349a38b9`.

All 360 planned EI-ZO jobs are sealed and their saved-file SHA-256 receipts were verified. This is development evidence from 60 fixed cases in 20 independent obstacle scenes, with three paired roadmap seeds and two objectives. It is not a formal test-set result or an exact Franka/FR3 reproduction.

Missing stage durations and absent candidate metrics stay missing. Finite recorded values are summarized without zero imputation; failure metrics describe saved candidates, not valid solutions. Success means the saved trajectory passed finite numerical checks, not a mathematical continuous collision-free certificate.

## Status and scene-cluster uncertainty

| Objective | Family | Legal/planned | Success / legal | Success / recorded DRM success | Terminal statuses |
|---|---|---:|---|---|---|
| minimum_distance | all | 180/180 | 98/180 (54.4%; 95% CI 35.6%–72.2%) | 98/106 (92.5%; 95% CI 86.5%–97.2%) | frontend_failed: 69; infrastructure_error: 5; solver_failed: 2; success_validated_numerical: 98; timeout: 6 |
| minimum_distance | bottleneck | 45/45 | 41/45 (91.1%; 95% CI 82.2%–100.0%) | 41/44 (93.2%; 95% CI 84.1%–100.0%) | frontend_failed: 1; success_validated_numerical: 41; timeout: 3 |
| minimum_distance | dense_clutter | 45/45 | 7/45 (15.6%; 95% CI 0.0%–42.2%) | 7/7 (100.0%; 95% CI 100.0%–100.0%) | frontend_failed: 33; infrastructure_error: 5; success_validated_numerical: 7 |
| minimum_distance | route_choice | 45/45 | 36/45 (80.0%; 95% CI 51.1%–97.8%) | 36/39 (92.3%; 95% CI 84.8%–97.8%) | frontend_failed: 6; solver_failed: 2; success_validated_numerical: 36; timeout: 1 |
| minimum_distance | sparse_clutter | 45/45 | 14/45 (31.1%; 95% CI 2.2%–68.9%) | 14/16 (87.5%; 95% CI 66.7%–100.0%) | frontend_failed: 29; success_validated_numerical: 14; timeout: 2 |
| minimum_time | all | 180/180 | 100/180 (55.6%; 95% CI 36.7%–73.9%) | 100/106 (94.3%; 95% CI 88.0%–99.0%) | frontend_failed: 69; infrastructure_error: 5; success_validated_numerical: 100; timeout: 6 |
| minimum_time | bottleneck | 45/45 | 41/45 (91.1%; 95% CI 82.2%–100.0%) | 41/44 (93.2%; 95% CI 84.1%–100.0%) | frontend_failed: 1; success_validated_numerical: 41; timeout: 3 |
| minimum_time | dense_clutter | 45/45 | 7/45 (15.6%; 95% CI 0.0%–42.2%) | 7/7 (100.0%; 95% CI 100.0%–100.0%) | frontend_failed: 33; infrastructure_error: 5; success_validated_numerical: 7 |
| minimum_time | route_choice | 45/45 | 38/45 (84.4%; 95% CI 53.3%–100.0%) | 38/39 (97.4%; 95% CI 88.9%–100.0%) | frontend_failed: 6; success_validated_numerical: 38; timeout: 1 |
| minimum_time | sparse_clutter | 45/45 | 14/45 (31.1%; 95% CI 2.2%–68.9%) | 14/16 (87.5%; 95% CI 66.7%–100.0%) | frontend_failed: 29; success_validated_numerical: 14; timeout: 2 |

The percentile bootstrap resamples physical scene IDs with all endpoint/roadmap pairs intact (2,000 replicates, PCG64 seed 45827). Each family has only five independent scenes. Seed repetitions are not new environments. Conditional rates include only recorded DRM successes; unknown/unreached DRM outcomes remain outside that conditional denominator.

## Timing and saved-candidate statistics

Cold time is measured by the parent process and includes imports, model/CUDA initialization, planning, validation and output. Warm query time is the recorded online block within that same execution, after setup; it is not a separate resident-service throughput trial. Timeouts retain measured wall time, including supervisor overhead; durations are not clipped to 60 seconds.

### minimum_distance / all

| Field | Outcome stratum | Recorded / missing | Mean | p50 | p90 | p95 | Min | Max |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| cold_e2e_seconds | all_finished | 180 / 0 | 3.377 | 1.315 | 1.963 | 2.853 | 0.8652 | 60.11 |
| cold_e2e_seconds | success | 98 / 0 | 1.69 | 1.523 | 2.022 | 2.225 | 1.076 | 4.55 |
| cold_e2e_seconds | failure | 82 / 0 | 5.395 | 1.082 | 1.335 | 60.11 | 0.8652 | 60.11 |
| warm_query_seconds | all_finished | 169 / 11 | 0.6104 | 0.5164 | 1.174 | 1.3 | 0.005579 | 3.716 |
| warm_query_seconds | success | 98 / 0 | 0.8841 | 0.7914 | 1.264 | 1.467 | 0.343 | 3.716 |
| warm_query_seconds | failure | 71 / 11 | 0.2325 | 0.08424 | 0.5056 | 0.5225 | 0.005579 | 1.215 |
| sampled_gpu_peak_mib | all_finished | 180 / 0 | 2674 | 4420 | 4420 | 4420 | 10 | 4420 |
| sampled_gpu_peak_mib | success | 98 / 0 | 4420 | 4420 | 4420 | 4420 | 4420 | 4420 |
| sampled_gpu_peak_mib | failure | 82 / 0 | 587.5 | 190 | 208 | 4420 | 10 | 4420 |
| regions | all_finished | 102 / 78 | 2.157 | 2 | 4 | 4 | 1 | 5 |
| regions | success | 98 / 0 | 2.133 | 2 | 4 | 4 | 1 | 5 |
| regions | failure | 4 / 78 | 2.75 | 3 | 3 | 3 | 2 | 3 |
| facets | all_finished | 102 / 78 | 80.73 | 74 | 138.7 | 170.8 | 25 | 189 |
| facets | success | 98 / 0 | 80.14 | 73.5 | 139.9 | 171 | 25 | 189 |
| facets | failure | 4 / 78 | 95 | 100.5 | 106 | 106 | 73 | 106 |
| recovery_attempts | all_finished | 100 / 80 | 0.1 | 0 | 0 | 1 | 0 | 3 |
| recovery_attempts | success | 98 / 0 | 0.102 | 0 | 0 | 1 | 0 | 3 |
| recovery_attempts | failure | 2 / 80 | 0 | 0 | 0 | 0 | 0 | 0 |
| normalized_path_length | all_finished | 100 / 80 | 1.129 | 1.209 | 1.624 | 1.704 | 0.4012 | 1.86 |
| normalized_path_length | success | 98 / 0 | 1.126 | 1.202 | 1.629 | 1.708 | 0.4012 | 1.86 |
| normalized_path_length | failure | 2 / 80 | 1.273 | 1.273 | 1.295 | 1.297 | 1.245 | 1.3 |
| minimum_sampled_clearance_m | all_finished | 100 / 80 | 0.0139 | 0.01903 | 0.01903 | 0.01903 | -0.02757 | 0.01903 |
| minimum_sampled_clearance_m | success | 98 / 0 | 0.01451 | 0.01903 | 0.01903 | 0.01903 | 0.0001587 | 0.01903 |
| minimum_sampled_clearance_m | failure | 2 / 80 | -0.01614 | -0.01614 | -0.006998 | -0.005855 | -0.02757 | -0.004712 |

| Saved stage timer | All mean / n / missing | Success mean / n / missing | Failure mean / n / missing |
|---|---:|---:|---:|
| cold_e2e_seconds | 1.045 / 169 / 11 | 1.319 / 98 / 0 | 0.6672 / 71 / 11 |
| collision_recovery_seconds | 0.0009944 / 100 / 80 | 0.001015 / 98 / 0 | 0 / 2 / 80 |
| drm_path_seconds | 0.1296 / 137 / 43 | 0.04142 / 98 / 0 | 0.3513 / 39 / 43 |
| eizo_inflation_seconds | 0.1373 / 100 / 80 | 0.1359 / 98 / 0 | 0.2037 / 2 / 80 |
| gpu_initialization_warmup_seconds | 0.1182 / 180 / 0 | 0.1181 / 98 / 0 | 0.1184 / 82 / 0 |
| model_setup_seconds | 0.01449 / 180 / 0 | 0.0145 / 98 / 0 | 0.01447 / 82 / 0 |
| online_collision_map_seconds | 0.00189 / 180 / 0 | 0.001533 / 98 / 0 | 0.002316 / 82 / 0 |
| roadmap_load_seconds | 0.0123 / 180 / 0 | 0.01235 / 98 / 0 | 0.01223 / 82 / 0 |
| trajectory_optimization_seconds | 0.0008652 / 100 / 80 | 0.0008642 / 98 / 0 | 0.0009138 / 2 / 80 |
| validation_seconds | 0.699 / 100 / 80 | 0.6974 / 98 / 0 | 0.7815 / 2 / 80 |
| warm_query_seconds | 0.6104 / 169 / 11 | 0.8841 / 98 / 0 | 0.2325 / 71 / 11 |

### minimum_distance / bottleneck

| Field | Outcome stratum | Recorded / missing | Mean | p50 | p90 | p95 | Min | Max |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| cold_e2e_seconds | all_finished | 45 / 0 | 5.436 | 1.521 | 1.96 | 48.52 | 0.8661 | 60.11 |
| cold_e2e_seconds | success | 41 / 0 | 1.547 | 1.521 | 1.948 | 1.955 | 1.076 | 2.157 |
| cold_e2e_seconds | failure | 4 / 0 | 45.3 | 60.11 | 60.11 | 60.11 | 0.8661 | 60.11 |
| warm_query_seconds | all_finished | 42 / 3 | 0.7192 | 0.718 | 1.134 | 1.218 | 0.03199 | 1.435 |
| warm_query_seconds | success | 41 / 0 | 0.736 | 0.7208 | 1.136 | 1.221 | 0.343 | 1.435 |
| warm_query_seconds | failure | 1 / 3 | 0.03199 | 0.03199 | 0.03199 | 0.03199 | 0.03199 | 0.03199 |
| sampled_gpu_peak_mib | all_finished | 45 / 0 | 4326 | 4420 | 4420 | 4420 | 188 | 4420 |
| sampled_gpu_peak_mib | success | 41 / 0 | 4420 | 4420 | 4420 | 4420 | 4420 | 4420 |
| sampled_gpu_peak_mib | failure | 4 / 0 | 3362 | 4420 | 4420 | 4420 | 188 | 4420 |
| regions | all_finished | 42 / 3 | 2.119 | 2 | 3 | 4 | 1 | 5 |
| regions | success | 41 / 0 | 2.122 | 2 | 3 | 4 | 1 | 5 |
| regions | failure | 1 / 3 | 2 | 2 | 2 | 2 | 2 | 2 |
| facets | all_finished | 42 / 3 | 81.52 | 77.5 | 140.5 | 171.9 | 25 | 189 |
| facets | success | 41 / 0 | 81.73 | 79 | 142 | 173 | 25 | 189 |
| facets | failure | 1 / 3 | 73 | 73 | 73 | 73 | 73 | 73 |
| recovery_attempts | all_finished | 41 / 4 | 0.02439 | 0 | 0 | 0 | 0 | 1 |
| recovery_attempts | success | 41 / 0 | 0.02439 | 0 | 0 | 0 | 0 | 1 |
| recovery_attempts | failure | 0 / 4 | — | — | — | — | — | — |
| normalized_path_length | all_finished | 41 / 4 | 1.057 | 0.9963 | 1.62 | 1.653 | 0.4895 | 1.746 |
| normalized_path_length | success | 41 / 0 | 1.057 | 0.9963 | 1.62 | 1.653 | 0.4895 | 1.746 |
| normalized_path_length | failure | 0 / 4 | — | — | — | — | — | — |
| minimum_sampled_clearance_m | all_finished | 41 / 4 | 0.01517 | 0.01903 | 0.01903 | 0.01903 | 0.0002926 | 0.01903 |
| minimum_sampled_clearance_m | success | 41 / 0 | 0.01517 | 0.01903 | 0.01903 | 0.01903 | 0.0002926 | 0.01903 |
| minimum_sampled_clearance_m | failure | 0 / 4 | — | — | — | — | — | — |

| Saved stage timer | All mean / n / missing | Success mean / n / missing | Failure mean / n / missing |
|---|---:|---:|---:|
| cold_e2e_seconds | 1.152 / 42 / 3 | 1.169 / 41 / 0 | 0.4531 / 1 / 3 |
| collision_recovery_seconds | 0.0001634 / 41 / 4 | 0.0001634 / 41 / 0 | — / 0 / 4 |
| drm_path_seconds | 0.03678 / 44 / 1 | 0.03526 / 41 / 0 | 0.05762 / 3 / 1 |
| eizo_inflation_seconds | 0.1161 / 41 / 4 | 0.1161 / 41 / 0 | — / 0 / 4 |
| gpu_initialization_warmup_seconds | 0.1152 / 45 / 0 | 0.116 / 41 / 0 | 0.1071 / 4 / 0 |
| model_setup_seconds | 0.0145 / 45 / 0 | 0.01451 / 41 / 0 | 0.01439 / 4 / 0 |
| online_collision_map_seconds | 0.001654 / 45 / 0 | 0.001667 / 41 / 0 | 0.00153 / 4 / 0 |
| roadmap_load_seconds | 0.01231 / 45 / 0 | 0.01234 / 41 / 0 | 0.01204 / 4 / 0 |
| trajectory_optimization_seconds | 0.0007848 / 41 / 4 | 0.0007848 / 41 / 0 | — / 0 / 4 |
| validation_seconds | 0.5765 / 41 / 4 | 0.5765 / 41 / 0 | — / 0 / 4 |
| warm_query_seconds | 0.7192 / 42 / 3 | 0.736 / 41 / 0 | 0.03199 / 1 / 3 |

### minimum_distance / dense_clutter

| Field | Outcome stratum | Recorded / missing | Mean | p50 | p90 | p95 | Min | Max |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| cold_e2e_seconds | all_finished | 45 / 0 | 1.211 | 1.297 | 1.733 | 1.903 | 0.868 | 2.386 |
| cold_e2e_seconds | success | 7 / 0 | 1.857 | 1.746 | 2.254 | 2.32 | 1.297 | 2.386 |
| cold_e2e_seconds | failure | 38 / 0 | 1.092 | 1.084 | 1.327 | 1.331 | 0.868 | 1.336 |
| warm_query_seconds | all_finished | 40 / 5 | 0.376 | 0.4032 | 0.8517 | 1.225 | 0.01185 | 1.646 |
| warm_query_seconds | success | 7 / 0 | 1.056 | 0.9119 | 1.457 | 1.552 | 0.5997 | 1.646 |
| warm_query_seconds | failure | 33 / 5 | 0.2318 | 0.1092 | 0.4902 | 0.5223 | 0.01185 | 0.5401 |
| sampled_gpu_peak_mib | all_finished | 45 / 0 | 835.9 | 194 | 4420 | 4420 | 10 | 4420 |
| sampled_gpu_peak_mib | success | 7 / 0 | 4420 | 4420 | 4420 | 4420 | 4420 | 4420 |
| sampled_gpu_peak_mib | failure | 38 / 0 | 175.7 | 190 | 204 | 206.3 | 10 | 208 |
| regions | all_finished | 7 / 38 | 2.857 | 3 | 4 | 4 | 2 | 4 |
| regions | success | 7 / 0 | 2.857 | 3 | 4 | 4 | 2 | 4 |
| regions | failure | 0 / 38 | — | — | — | — | — | — |
| facets | all_finished | 7 / 38 | 98 | 98 | 137.2 | 138.1 | 57 | 139 |
| facets | success | 7 / 0 | 98 | 98 | 137.2 | 138.1 | 57 | 139 |
| facets | failure | 0 / 38 | — | — | — | — | — | — |
| recovery_attempts | all_finished | 7 / 38 | 0 | 0 | 0 | 0 | 0 | 0 |
| recovery_attempts | success | 7 / 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| recovery_attempts | failure | 0 / 38 | — | — | — | — | — | — |
| normalized_path_length | all_finished | 7 / 38 | 1.171 | 1.135 | 1.616 | 1.723 | 0.6071 | 1.83 |
| normalized_path_length | success | 7 / 0 | 1.171 | 1.135 | 1.616 | 1.723 | 0.6071 | 1.83 |
| normalized_path_length | failure | 0 / 38 | — | — | — | — | — | — |
| minimum_sampled_clearance_m | all_finished | 7 / 38 | 0.01522 | 0.01903 | 0.01903 | 0.01903 | 0.001339 | 0.01903 |
| minimum_sampled_clearance_m | success | 7 / 0 | 0.01522 | 0.01903 | 0.01903 | 0.01903 | 0.001339 | 0.01903 |
| minimum_sampled_clearance_m | failure | 0 / 38 | — | — | — | — | — | — |

| Saved stage timer | All mean / n / missing | Success mean / n / missing | Failure mean / n / missing |
|---|---:|---:|---:|
| cold_e2e_seconds | 0.8099 / 40 / 5 | 1.489 / 7 / 0 | 0.6658 / 33 / 5 |
| collision_recovery_seconds | 0 / 7 / 38 | 0 / 7 / 0 | — / 0 / 38 |
| drm_path_seconds | 0.3465 / 23 / 22 | 0.1216 / 7 / 0 | 0.445 / 16 / 22 |
| eizo_inflation_seconds | 0.2297 / 7 / 38 | 0.2297 / 7 / 0 | — / 0 / 38 |
| gpu_initialization_warmup_seconds | 0.1176 / 45 / 0 | 0.1179 / 7 / 0 | 0.1175 / 38 / 0 |
| model_setup_seconds | 0.0145 / 45 / 0 | 0.01449 / 7 / 0 | 0.0145 / 38 / 0 |
| online_collision_map_seconds | 0.002707 / 45 / 0 | 0.002153 / 7 / 0 | 0.00281 / 38 / 0 |
| roadmap_load_seconds | 0.01225 / 45 / 0 | 0.01239 / 7 / 0 | 0.01222 / 38 / 0 |
| trajectory_optimization_seconds | 0.0008829 / 7 / 38 | 0.0008829 / 7 / 0 | — / 0 / 38 |
| validation_seconds | 0.696 / 7 / 38 | 0.696 / 7 / 0 | — / 0 / 38 |
| warm_query_seconds | 0.376 / 40 / 5 | 1.056 / 7 / 0 | 0.2318 / 33 / 5 |

### minimum_distance / route_choice

| Field | Outcome stratum | Recorded / missing | Mean | p50 | p90 | p95 | Min | Max |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| cold_e2e_seconds | all_finished | 45 / 0 | 2.971 | 1.523 | 2.079 | 2.686 | 0.8707 | 60.11 |
| cold_e2e_seconds | success | 36 / 0 | 1.736 | 1.625 | 2.06 | 2.35 | 1.086 | 3.701 |
| cold_e2e_seconds | failure | 9 / 0 | 7.909 | 1.306 | 13.58 | 36.85 | 0.8707 | 60.11 |
| warm_query_seconds | all_finished | 44 / 1 | 0.8635 | 0.7981 | 1.253 | 1.306 | 0.02175 | 2.805 |
| warm_query_seconds | success | 36 / 0 | 0.9286 | 0.8193 | 1.27 | 1.489 | 0.3431 | 2.805 |
| warm_query_seconds | failure | 8 / 1 | 0.5705 | 0.5098 | 0.948 | 1.081 | 0.02175 | 1.215 |
| sampled_gpu_peak_mib | all_finished | 45 / 0 | 3856 | 4420 | 4420 | 4420 | 188 | 4420 |
| sampled_gpu_peak_mib | success | 36 / 0 | 4420 | 4420 | 4420 | 4420 | 4420 | 4420 |
| sampled_gpu_peak_mib | failure | 9 / 0 | 1601 | 194 | 4420 | 4420 | 188 | 4420 |
| regions | all_finished | 38 / 7 | 1.895 | 2 | 3 | 4 | 1 | 4 |
| regions | success | 36 / 0 | 1.833 | 1.5 | 3 | 4 | 1 | 4 |
| regions | failure | 2 / 7 | 3 | 3 | 3 | 3 | 3 | 3 |
| facets | all_finished | 38 / 7 | 72.34 | 64.5 | 120.3 | 167.6 | 26 | 171 |
| facets | success | 36 / 0 | 70.78 | 62.5 | 122.5 | 168 | 26 | 171 |
| facets | failure | 2 / 7 | 100.5 | 100.5 | 104.9 | 105.5 | 95 | 106 |
| recovery_attempts | all_finished | 38 / 7 | 0.07895 | 0 | 0 | 0.15 | 0 | 2 |
| recovery_attempts | success | 36 / 0 | 0.08333 | 0 | 0 | 0.25 | 0 | 2 |
| recovery_attempts | failure | 2 / 7 | 0 | 0 | 0 | 0 | 0 | 0 |
| normalized_path_length | all_finished | 38 / 7 | 1.151 | 1.225 | 1.45 | 1.555 | 0.4012 | 1.673 |
| normalized_path_length | success | 36 / 0 | 1.144 | 1.223 | 1.457 | 1.558 | 0.4012 | 1.673 |
| normalized_path_length | failure | 2 / 7 | 1.273 | 1.273 | 1.295 | 1.297 | 1.245 | 1.3 |
| minimum_sampled_clearance_m | all_finished | 38 / 7 | 0.01284 | 0.01903 | 0.01903 | 0.01903 | -0.02757 | 0.01903 |
| minimum_sampled_clearance_m | success | 36 / 0 | 0.01445 | 0.01903 | 0.01903 | 0.01903 | 0.002333 | 0.01903 |
| minimum_sampled_clearance_m | failure | 2 / 7 | -0.01614 | -0.01614 | -0.006998 | -0.005855 | -0.02757 | -0.004712 |

| Saved stage timer | All mean / n / missing | Success mean / n / missing | Failure mean / n / missing |
|---|---:|---:|---:|
| cold_e2e_seconds | 1.299 / 44 / 1 | 1.365 / 36 / 0 | 1.004 / 8 / 1 |
| collision_recovery_seconds | 0.000858 / 38 / 7 | 0.0009057 / 36 / 0 | 0 / 2 / 7 |
| drm_path_seconds | 0.08581 / 44 / 1 | 0.03241 / 36 / 0 | 0.3261 / 8 / 1 |
| eizo_inflation_seconds | 0.1451 / 38 / 7 | 0.1419 / 36 / 0 | 0.2037 / 2 / 7 |
| gpu_initialization_warmup_seconds | 0.1191 / 45 / 0 | 0.1193 / 36 / 0 | 0.1181 / 9 / 0 |
| model_setup_seconds | 0.01451 / 45 / 0 | 0.01452 / 36 / 0 | 0.01445 / 9 / 0 |
| online_collision_map_seconds | 0.001634 / 45 / 0 | 0.001493 / 36 / 0 | 0.002199 / 9 / 0 |
| roadmap_load_seconds | 0.01236 / 45 / 0 | 0.01238 / 36 / 0 | 0.0123 / 9 / 0 |
| trajectory_optimization_seconds | 0.0007736 / 38 / 7 | 0.0007658 / 36 / 0 | 0.0009138 / 2 / 7 |
| validation_seconds | 0.7471 / 38 / 7 | 0.7452 / 36 / 0 | 0.7815 / 2 / 7 |
| warm_query_seconds | 0.8635 / 44 / 1 | 0.9286 / 36 / 0 | 0.5705 / 8 / 1 |

### minimum_distance / sparse_clutter

| Field | Outcome stratum | Recorded / missing | Mean | p50 | p90 | p95 | Min | Max |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| cold_e2e_seconds | all_finished | 45 / 0 | 3.892 | 1.086 | 2.076 | 4.378 | 0.8652 | 60.11 |
| cold_e2e_seconds | success | 14 / 0 | 1.904 | 1.52 | 3.233 | 3.991 | 1.294 | 4.55 |
| cold_e2e_seconds | failure | 31 / 0 | 4.79 | 0.8831 | 1.31 | 30.71 | 0.8652 | 60.11 |
| warm_query_seconds | all_finished | 43 / 2 | 0.4631 | 0.3214 | 0.9601 | 1.303 | 0.005579 | 3.716 |
| warm_query_seconds | success | 14 / 0 | 1.118 | 0.7307 | 2.396 | 3.157 | 0.491 | 3.716 |
| warm_query_seconds | failure | 29 / 2 | 0.147 | 0.0347 | 0.3948 | 0.4034 | 0.005579 | 0.4438 |
| sampled_gpu_peak_mib | all_finished | 45 / 0 | 1678 | 192 | 4420 | 4420 | 10 | 4420 |
| sampled_gpu_peak_mib | success | 14 / 0 | 4420 | 4420 | 4420 | 4420 | 4420 | 4420 |
| sampled_gpu_peak_mib | failure | 31 / 0 | 440.1 | 188 | 198 | 2310 | 10 | 4420 |
| regions | all_finished | 15 / 30 | 2.6 | 2 | 4 | 4.3 | 2 | 5 |
| regions | success | 14 / 0 | 2.571 | 2 | 4 | 4.35 | 2 | 5 |
| regions | failure | 1 / 30 | 3 | 3 | 3 | 3 | 3 | 3 |
| facets | all_finished | 15 / 30 | 91.67 | 74 | 150.4 | 170 | 52 | 184 |
| facets | success | 14 / 0 | 90.64 | 74 | 153.8 | 171 | 52 | 184 |
| facets | failure | 1 / 30 | 106 | 106 | 106 | 106 | 106 | 106 |
| recovery_attempts | all_finished | 14 / 31 | 0.4286 | 0 | 1.7 | 2.35 | 0 | 3 |
| recovery_attempts | success | 14 / 0 | 0.4286 | 0 | 1.7 | 2.35 | 0 | 3 |
| recovery_attempts | failure | 0 / 31 | — | — | — | — | — | — |
| normalized_path_length | all_finished | 14 / 31 | 1.259 | 1.31 | 1.773 | 1.811 | 0.7176 | 1.86 |
| normalized_path_length | success | 14 / 0 | 1.259 | 1.31 | 1.773 | 1.811 | 0.7176 | 1.86 |
| normalized_path_length | failure | 0 / 31 | — | — | — | — | — | — |
| minimum_sampled_clearance_m | all_finished | 14 / 31 | 0.01238 | 0.01233 | 0.01903 | 0.01903 | 0.0001587 | 0.01903 |
| minimum_sampled_clearance_m | success | 14 / 0 | 0.01238 | 0.01233 | 0.01903 | 0.01903 | 0.0001587 | 0.01903 |
| minimum_sampled_clearance_m | failure | 0 / 31 | — | — | — | — | — | — |

| Saved stage timer | All mean / n / missing | Success mean / n / missing | Failure mean / n / missing |
|---|---:|---:|---:|
| cold_e2e_seconds | 0.8994 / 43 / 2 | 1.554 / 14 / 0 | 0.5834 / 29 / 2 |
| collision_recovery_seconds | 0.004296 / 14 / 31 | 0.004296 / 14 / 0 | — / 0 / 31 |
| drm_path_seconds | 0.1691 / 26 / 19 | 0.04258 / 14 / 0 | 0.3166 / 12 / 19 |
| eizo_inflation_seconds | 0.1318 / 14 / 31 | 0.1318 / 14 / 0 | — / 0 / 31 |
| gpu_initialization_warmup_seconds | 0.121 / 45 / 0 | 0.1212 / 14 / 0 | 0.1209 / 31 / 0 |
| model_setup_seconds | 0.01445 / 45 / 0 | 0.01445 / 14 / 0 | 0.01445 / 31 / 0 |
| online_collision_map_seconds | 0.001564 / 45 / 0 | 0.0009364 / 14 / 0 | 0.001847 / 31 / 0 |
| roadmap_load_seconds | 0.01227 / 45 / 0 | 0.0123 / 14 / 0 | 0.01225 / 31 / 0 |
| trajectory_optimization_seconds | 0.00134 / 14 / 31 | 0.00134 / 14 / 0 | — / 0 / 31 |
| validation_seconds | 0.9289 / 14 / 31 | 0.9289 / 14 / 0 | — / 0 / 31 |
| warm_query_seconds | 0.4631 / 43 / 2 | 1.118 / 14 / 0 | 0.147 / 29 / 2 |

### minimum_time / all

| Field | Outcome stratum | Recorded / missing | Mean | p50 | p90 | p95 | Min | Max |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| cold_e2e_seconds | all_finished | 180 / 0 | 3.529 | 1.517 | 2.391 | 2.623 | 0.8664 | 60.11 |
| cold_e2e_seconds | success | 100 / 0 | 1.965 | 1.948 | 2.419 | 2.601 | 1.296 | 3.053 |
| cold_e2e_seconds | failure | 80 / 0 | 5.484 | 1.082 | 1.333 | 60.11 | 0.8664 | 60.11 |
| warm_query_seconds | all_finished | 169 / 11 | 0.7754 | 0.6783 | 1.579 | 1.724 | 0.005622 | 2.161 |
| warm_query_seconds | success | 100 / 0 | 1.17 | 1.145 | 1.698 | 1.776 | 0.536 | 2.161 |
| warm_query_seconds | failure | 69 / 11 | 0.203 | 0.08447 | 0.4735 | 0.49 | 0.005622 | 0.5542 |
| sampled_gpu_peak_mib | all_finished | 180 / 0 | 2675 | 4420 | 4420 | 4420 | 10 | 4426 |
| sampled_gpu_peak_mib | success | 100 / 0 | 4420 | 4420 | 4420 | 4420 | 4420 | 4426 |
| sampled_gpu_peak_mib | failure | 80 / 0 | 494.5 | 190 | 208 | 4420 | 10 | 4420 |
| regions | all_finished | 102 / 78 | 2.157 | 2 | 4 | 4 | 1 | 5 |
| regions | success | 100 / 0 | 2.15 | 2 | 4 | 4 | 1 | 5 |
| regions | failure | 2 / 78 | 2.5 | 2.5 | 2.9 | 2.95 | 2 | 3 |
| facets | all_finished | 102 / 78 | 80.63 | 73.5 | 138.7 | 170.7 | 25 | 189 |
| facets | success | 100 / 0 | 80.45 | 73.5 | 139.3 | 171 | 25 | 189 |
| facets | failure | 2 / 78 | 89.5 | 89.5 | 102.7 | 104.3 | 73 | 106 |
| recovery_attempts | all_finished | 100 / 80 | 0.01 | 0 | 0 | 0 | 0 | 1 |
| recovery_attempts | success | 100 / 0 | 0.01 | 0 | 0 | 0 | 0 | 1 |
| recovery_attempts | failure | 0 / 80 | — | — | — | — | — | — |
| normalized_path_length | all_finished | 100 / 80 | 1.204 | 1.279 | 1.738 | 1.894 | 0.4016 | 2.024 |
| normalized_path_length | success | 100 / 0 | 1.204 | 1.279 | 1.738 | 1.894 | 0.4016 | 2.024 |
| normalized_path_length | failure | 0 / 80 | — | — | — | — | — | — |
| minimum_sampled_clearance_m | all_finished | 100 / 80 | 0.0147 | 0.01903 | 0.01903 | 0.01903 | 0.0005933 | 0.01903 |
| minimum_sampled_clearance_m | success | 100 / 0 | 0.0147 | 0.01903 | 0.01903 | 0.01903 | 0.0005933 | 0.01903 |
| minimum_sampled_clearance_m | failure | 0 / 80 | — | — | — | — | — | — |

| Saved stage timer | All mean / n / missing | Success mean / n / missing | Failure mean / n / missing |
|---|---:|---:|---:|
| cold_e2e_seconds | 1.206 / 169 / 11 | 1.601 / 100 / 0 | 0.6346 / 69 / 11 |
| collision_recovery_seconds | 8.615e-05 / 100 / 80 | 8.615e-05 / 100 / 0 | — / 0 / 80 |
| drm_path_seconds | 0.1239 / 137 / 43 | 0.03829 / 100 / 0 | 0.3554 / 37 / 43 |
| eizo_inflation_seconds | 0.1296 / 100 / 80 | 0.1296 / 100 / 0 | — / 0 / 80 |
| gpu_initialization_warmup_seconds | 0.1141 / 180 / 0 | 0.1138 / 100 / 0 | 0.1146 / 80 / 0 |
| model_setup_seconds | 0.0145 / 180 / 0 | 0.01453 / 100 / 0 | 0.01447 / 80 / 0 |
| online_collision_map_seconds | 0.001888 / 180 / 0 | 0.001544 / 100 / 0 | 0.002317 / 80 / 0 |
| roadmap_load_seconds | 0.01228 / 180 / 0 | 0.01229 / 100 / 0 | 0.01226 / 80 / 0 |
| trajectory_optimization_seconds | 0.0173 / 100 / 80 | 0.0173 / 100 / 0 | — / 0 / 80 |
| validation_seconds | 0.9768 / 100 / 80 | 0.9768 / 100 / 0 | — / 0 / 80 |
| warm_query_seconds | 0.7754 / 169 / 11 | 1.17 / 100 / 0 | 0.203 / 69 / 11 |

### minimum_time / bottleneck

| Field | Outcome stratum | Recorded / missing | Mean | p50 | p90 | p95 | Min | Max |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| cold_e2e_seconds | all_finished | 45 / 0 | 5.679 | 1.743 | 2.386 | 48.61 | 0.873 | 60.11 |
| cold_e2e_seconds | success | 41 / 0 | 1.814 | 1.739 | 2.202 | 2.378 | 1.296 | 2.596 |
| cold_e2e_seconds | failure | 4 / 0 | 45.3 | 60.11 | 60.11 | 60.11 | 0.873 | 60.11 |
| warm_query_seconds | all_finished | 42 / 3 | 1.02 | 1.044 | 1.454 | 1.652 | 0.03713 | 1.765 |
| warm_query_seconds | success | 41 / 0 | 1.044 | 1.044 | 1.455 | 1.66 | 0.5683 | 1.765 |
| warm_query_seconds | failure | 1 / 3 | 0.03713 | 0.03713 | 0.03713 | 0.03713 | 0.03713 | 0.03713 |
| sampled_gpu_peak_mib | all_finished | 45 / 0 | 4326 | 4420 | 4420 | 4420 | 188 | 4420 |
| sampled_gpu_peak_mib | success | 41 / 0 | 4420 | 4420 | 4420 | 4420 | 4420 | 4420 |
| sampled_gpu_peak_mib | failure | 4 / 0 | 3362 | 4420 | 4420 | 4420 | 188 | 4420 |
| regions | all_finished | 42 / 3 | 2.119 | 2 | 3 | 4 | 1 | 5 |
| regions | success | 41 / 0 | 2.122 | 2 | 3 | 4 | 1 | 5 |
| regions | failure | 1 / 3 | 2 | 2 | 2 | 2 | 2 | 2 |
| facets | all_finished | 42 / 3 | 81.5 | 77.5 | 140.5 | 171.9 | 25 | 189 |
| facets | success | 41 / 0 | 81.71 | 79 | 142 | 173 | 25 | 189 |
| facets | failure | 1 / 3 | 73 | 73 | 73 | 73 | 73 | 73 |
| recovery_attempts | all_finished | 41 / 4 | 0 | 0 | 0 | 0 | 0 | 0 |
| recovery_attempts | success | 41 / 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| recovery_attempts | failure | 0 / 4 | — | — | — | — | — | — |
| normalized_path_length | all_finished | 41 / 4 | 1.13 | 1.1 | 1.736 | 1.883 | 0.4931 | 1.951 |
| normalized_path_length | success | 41 / 0 | 1.13 | 1.1 | 1.736 | 1.883 | 0.4931 | 1.951 |
| normalized_path_length | failure | 0 / 4 | — | — | — | — | — | — |
| minimum_sampled_clearance_m | all_finished | 41 / 4 | 0.01496 | 0.01903 | 0.01903 | 0.01903 | 0.001294 | 0.01903 |
| minimum_sampled_clearance_m | success | 41 / 0 | 0.01496 | 0.01903 | 0.01903 | 0.01903 | 0.001294 | 0.01903 |
| minimum_sampled_clearance_m | failure | 0 / 4 | — | — | — | — | — | — |

| Saved stage timer | All mean / n / missing | Success mean / n / missing | Failure mean / n / missing |
|---|---:|---:|---:|
| cold_e2e_seconds | 1.452 / 42 / 3 | 1.475 / 41 / 0 | 0.486 / 1 / 3 |
| collision_recovery_seconds | 0 / 41 / 4 | 0 / 41 / 0 | — / 0 / 4 |
| drm_path_seconds | 0.0347 / 44 / 1 | 0.03328 / 41 / 0 | 0.05413 / 3 / 1 |
| eizo_inflation_seconds | 0.1098 / 41 / 4 | 0.1098 / 41 / 0 | — / 0 / 4 |
| gpu_initialization_warmup_seconds | 0.1145 / 45 / 0 | 0.1144 / 41 / 0 | 0.1151 / 4 / 0 |
| model_setup_seconds | 0.01449 / 45 / 0 | 0.01449 / 41 / 0 | 0.01449 / 4 / 0 |
| online_collision_map_seconds | 0.001674 / 45 / 0 | 0.001688 / 41 / 0 | 0.001536 / 4 / 0 |
| roadmap_load_seconds | 0.01228 / 45 / 0 | 0.01226 / 41 / 0 | 0.01246 / 4 / 0 |
| trajectory_optimization_seconds | 0.01718 / 41 / 4 | 0.01718 / 41 / 0 | — / 0 / 4 |
| validation_seconds | 0.8754 / 41 / 4 | 0.8754 / 41 / 0 | — / 0 / 4 |
| warm_query_seconds | 1.02 / 42 / 3 | 1.044 / 41 / 0 | 0.03713 / 1 / 3 |

### minimum_time / dense_clutter

| Field | Outcome stratum | Recorded / missing | Mean | p50 | p90 | p95 | Min | Max |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| cold_e2e_seconds | all_finished | 45 / 0 | 1.282 | 1.093 | 2.076 | 2.516 | 0.8664 | 3.053 |
| cold_e2e_seconds | success | 7 / 0 | 2.355 | 2.175 | 2.913 | 2.983 | 1.733 | 3.053 |
| cold_e2e_seconds | failure | 38 / 0 | 1.084 | 1.082 | 1.326 | 1.329 | 0.8664 | 1.338 |
| warm_query_seconds | all_finished | 40 / 5 | 0.4481 | 0.379 | 1.287 | 1.771 | 0.01197 | 2.161 |
| warm_query_seconds | success | 7 / 0 | 1.504 | 1.312 | 2.023 | 2.092 | 0.9246 | 2.161 |
| warm_query_seconds | failure | 33 / 5 | 0.2241 | 0.1056 | 0.4832 | 0.4917 | 0.01197 | 0.4975 |
| sampled_gpu_peak_mib | all_finished | 45 / 0 | 840.9 | 200 | 4420 | 4420 | 10 | 4420 |
| sampled_gpu_peak_mib | success | 7 / 0 | 4420 | 4420 | 4420 | 4420 | 4420 | 4420 |
| sampled_gpu_peak_mib | failure | 38 / 0 | 181.6 | 192 | 206 | 208 | 10 | 208 |
| regions | all_finished | 7 / 38 | 2.857 | 3 | 4 | 4 | 2 | 4 |
| regions | success | 7 / 0 | 2.857 | 3 | 4 | 4 | 2 | 4 |
| regions | failure | 0 / 38 | — | — | — | — | — | — |
| facets | all_finished | 7 / 38 | 98 | 98 | 137.2 | 138.1 | 57 | 139 |
| facets | success | 7 / 0 | 98 | 98 | 137.2 | 138.1 | 57 | 139 |
| facets | failure | 0 / 38 | — | — | — | — | — | — |
| recovery_attempts | all_finished | 7 / 38 | 0 | 0 | 0 | 0 | 0 | 0 |
| recovery_attempts | success | 7 / 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| recovery_attempts | failure | 0 / 38 | — | — | — | — | — | — |
| normalized_path_length | all_finished | 7 / 38 | 1.27 | 1.251 | 1.775 | 1.864 | 0.6675 | 1.953 |
| normalized_path_length | success | 7 / 0 | 1.27 | 1.251 | 1.775 | 1.864 | 0.6675 | 1.953 |
| normalized_path_length | failure | 0 / 38 | — | — | — | — | — | — |
| minimum_sampled_clearance_m | all_finished | 7 / 38 | 0.01745 | 0.01903 | 0.01903 | 0.01903 | 0.01341 | 0.01903 |
| minimum_sampled_clearance_m | success | 7 / 0 | 0.01745 | 0.01903 | 0.01903 | 0.01903 | 0.01341 | 0.01903 |
| minimum_sampled_clearance_m | failure | 0 / 38 | — | — | — | — | — | — |

| Saved stage timer | All mean / n / missing | Success mean / n / missing | Failure mean / n / missing |
|---|---:|---:|---:|
| cold_e2e_seconds | 0.878 / 40 / 5 | 1.929 / 7 / 0 | 0.6552 / 33 / 5 |
| collision_recovery_seconds | 0 / 7 / 38 | 0 / 7 / 0 | — / 0 / 38 |
| drm_path_seconds | 0.3322 / 23 / 22 | 0.1107 / 7 / 0 | 0.4291 / 16 / 22 |
| eizo_inflation_seconds | 0.2252 / 7 / 38 | 0.2252 / 7 / 0 | — / 0 / 38 |
| gpu_initialization_warmup_seconds | 0.1127 / 45 / 0 | 0.109 / 7 / 0 | 0.1134 / 38 / 0 |
| model_setup_seconds | 0.0145 / 45 / 0 | 0.0145 / 7 / 0 | 0.0145 / 38 / 0 |
| online_collision_map_seconds | 0.002687 / 45 / 0 | 0.002116 / 7 / 0 | 0.002792 / 38 / 0 |
| roadmap_load_seconds | 0.01222 / 45 / 0 | 0.01217 / 7 / 0 | 0.01222 / 38 / 0 |
| trajectory_optimization_seconds | 0.0241 / 7 / 38 | 0.0241 / 7 / 0 | — / 0 / 38 |
| validation_seconds | 1.135 / 7 / 38 | 1.135 / 7 / 0 | — / 0 / 38 |
| warm_query_seconds | 0.4481 / 40 / 5 | 1.504 / 7 / 0 | 0.2241 / 33 / 5 |

### minimum_time / route_choice

| Field | Outcome stratum | Recorded / missing | Mean | p50 | p90 | p95 | Min | Max |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| cold_e2e_seconds | all_finished | 45 / 0 | 3.247 | 1.967 | 2.589 | 2.61 | 0.8798 | 60.11 |
| cold_e2e_seconds | success | 38 / 0 | 2.067 | 2.162 | 2.589 | 2.6 | 1.296 | 2.613 |
| cold_e2e_seconds | failure | 7 / 0 | 9.65 | 1.298 | 24.85 | 42.48 | 0.8798 | 60.11 |
| warm_query_seconds | all_finished | 44 / 1 | 1.137 | 1.19 | 1.713 | 1.768 | 0.02209 | 1.792 |
| warm_query_seconds | success | 38 / 0 | 1.253 | 1.251 | 1.722 | 1.777 | 0.536 | 1.792 |
| warm_query_seconds | failure | 6 / 1 | 0.4028 | 0.4581 | 0.5215 | 0.5378 | 0.02209 | 0.5542 |
| sampled_gpu_peak_mib | all_finished | 45 / 0 | 3856 | 4420 | 4420 | 4420 | 188 | 4420 |
| sampled_gpu_peak_mib | success | 38 / 0 | 4420 | 4420 | 4420 | 4420 | 4420 | 4420 |
| sampled_gpu_peak_mib | failure | 7 / 0 | 795.4 | 192 | 1884 | 3152 | 188 | 4420 |
| regions | all_finished | 38 / 7 | 1.895 | 2 | 3 | 4 | 1 | 4 |
| regions | success | 38 / 0 | 1.895 | 2 | 3 | 4 | 1 | 4 |
| regions | failure | 0 / 7 | — | — | — | — | — | — |
| facets | all_finished | 38 / 7 | 72.26 | 64.5 | 120.3 | 165.9 | 26 | 171 |
| facets | success | 38 / 0 | 72.26 | 64.5 | 120.3 | 165.9 | 26 | 171 |
| facets | failure | 0 / 7 | — | — | — | — | — | — |
| recovery_attempts | all_finished | 38 / 7 | 0 | 0 | 0 | 0 | 0 | 0 |
| recovery_attempts | success | 38 / 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| recovery_attempts | failure | 0 / 7 | — | — | — | — | — | — |
| normalized_path_length | all_finished | 38 / 7 | 1.219 | 1.327 | 1.583 | 1.644 | 0.4016 | 1.807 |
| normalized_path_length | success | 38 / 0 | 1.219 | 1.327 | 1.583 | 1.644 | 0.4016 | 1.807 |
| normalized_path_length | failure | 0 / 7 | — | — | — | — | — | — |
| minimum_sampled_clearance_m | all_finished | 38 / 7 | 0.0143 | 0.01903 | 0.01903 | 0.01903 | 0.001755 | 0.01903 |
| minimum_sampled_clearance_m | success | 38 / 0 | 0.0143 | 0.01903 | 0.01903 | 0.01903 | 0.001755 | 0.01903 |
| minimum_sampled_clearance_m | failure | 0 / 7 | — | — | — | — | — | — |

| Saved stage timer | All mean / n / missing | Success mean / n / missing | Failure mean / n / missing |
|---|---:|---:|---:|
| cold_e2e_seconds | 1.568 / 44 / 1 | 1.685 / 38 / 0 | 0.8289 / 6 / 1 |
| collision_recovery_seconds | 0 / 38 / 7 | 0 / 38 / 0 | — / 0 / 7 |
| drm_path_seconds | 0.08137 / 44 / 1 | 0.03004 / 38 / 0 | 0.4064 / 6 / 1 |
| eizo_inflation_seconds | 0.1356 / 38 / 7 | 0.1356 / 38 / 0 | — / 0 / 7 |
| gpu_initialization_warmup_seconds | 0.1136 / 45 / 0 | 0.1145 / 38 / 0 | 0.1087 / 7 / 0 |
| model_setup_seconds | 0.01455 / 45 / 0 | 0.01458 / 38 / 0 | 0.01444 / 7 / 0 |
| online_collision_map_seconds | 0.001644 / 45 / 0 | 0.001505 / 38 / 0 | 0.002399 / 7 / 0 |
| roadmap_load_seconds | 0.01238 / 45 / 0 | 0.01238 / 38 / 0 | 0.01236 / 7 / 0 |
| trajectory_optimization_seconds | 0.01475 / 38 / 7 | 0.01475 / 38 / 0 | — / 0 / 7 |
| validation_seconds | 1.065 / 38 / 7 | 1.065 / 38 / 0 | — / 0 / 7 |
| warm_query_seconds | 1.137 / 44 / 1 | 1.253 / 38 / 0 | 0.4028 / 6 / 1 |

### minimum_time / sparse_clutter

| Field | Outcome stratum | Recorded / missing | Mean | p50 | p90 | p95 | Min | Max |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| cold_e2e_seconds | all_finished | 45 / 0 | 3.907 | 1.086 | 2.378 | 2.727 | 0.8717 | 60.11 |
| cold_e2e_seconds | success | 14 / 0 | 1.933 | 1.74 | 2.382 | 2.533 | 1.51 | 2.813 |
| cold_e2e_seconds | failure | 31 / 0 | 4.798 | 0.8931 | 1.306 | 30.71 | 0.8717 | 60.11 |
| warm_query_seconds | all_finished | 43 / 2 | 0.4706 | 0.3207 | 1.376 | 1.588 | 0.005622 | 1.9 |
| warm_query_seconds | success | 14 / 0 | 1.148 | 0.9453 | 1.635 | 1.74 | 0.714 | 1.9 |
| warm_query_seconds | failure | 29 / 2 | 0.1435 | 0.03552 | 0.3942 | 0.3982 | 0.005622 | 0.4344 |
| sampled_gpu_peak_mib | all_finished | 45 / 0 | 1679 | 190 | 4420 | 4426 | 10 | 4426 |
| sampled_gpu_peak_mib | success | 14 / 0 | 4422 | 4420 | 4426 | 4426 | 4420 | 4426 |
| sampled_gpu_peak_mib | failure | 31 / 0 | 440.1 | 188 | 200 | 2310 | 10 | 4420 |
| regions | all_finished | 15 / 30 | 2.6 | 2 | 4 | 4.3 | 2 | 5 |
| regions | success | 14 / 0 | 2.571 | 2 | 4 | 4.35 | 2 | 5 |
| regions | failure | 1 / 30 | 3 | 3 | 3 | 3 | 3 | 3 |
| facets | all_finished | 15 / 30 | 91.27 | 74 | 149.2 | 169.1 | 52 | 181 |
| facets | success | 14 / 0 | 90.21 | 74 | 152.9 | 169.9 | 52 | 181 |
| facets | failure | 1 / 30 | 106 | 106 | 106 | 106 | 106 | 106 |
| recovery_attempts | all_finished | 14 / 31 | 0.07143 | 0 | 0 | 0.35 | 0 | 1 |
| recovery_attempts | success | 14 / 0 | 0.07143 | 0 | 0 | 0.35 | 0 | 1 |
| recovery_attempts | failure | 0 / 31 | — | — | — | — | — | — |
| normalized_path_length | all_finished | 14 / 31 | 1.349 | 1.335 | 1.982 | 2.022 | 0.7698 | 2.024 |
| normalized_path_length | success | 14 / 0 | 1.349 | 1.335 | 1.982 | 2.022 | 0.7698 | 2.024 |
| normalized_path_length | failure | 0 / 31 | — | — | — | — | — | — |
| minimum_sampled_clearance_m | all_finished | 14 / 31 | 0.01363 | 0.01618 | 0.01903 | 0.01903 | 0.0005933 | 0.01903 |
| minimum_sampled_clearance_m | success | 14 / 0 | 0.01363 | 0.01618 | 0.01903 | 0.01903 | 0.0005933 | 0.01903 |
| minimum_sampled_clearance_m | failure | 0 / 31 | — | — | — | — | — | — |

| Saved stage timer | All mean / n / missing | Success mean / n / missing | Failure mean / n / missing |
|---|---:|---:|---:|
| cold_e2e_seconds | 0.9015 / 43 / 2 | 1.576 / 14 / 0 | 0.576 / 29 / 2 |
| collision_recovery_seconds | 0.0006154 / 14 / 31 | 0.0006154 / 14 / 0 | — / 0 / 31 |
| drm_path_seconds | 0.1627 / 26 / 19 | 0.03916 / 14 / 0 | 0.3069 / 12 / 19 |
| eizo_inflation_seconds | 0.1231 / 14 / 31 | 0.1231 / 14 / 0 | — / 0 / 31 |
| gpu_initialization_warmup_seconds | 0.1157 / 45 / 0 | 0.1122 / 14 / 0 | 0.1173 / 31 / 0 |
| model_setup_seconds | 0.01447 / 45 / 0 | 0.01452 / 14 / 0 | 0.01444 / 31 / 0 |
| online_collision_map_seconds | 0.001545 / 45 / 0 | 0.0009444 / 14 / 0 | 0.001817 / 31 / 0 |
| roadmap_load_seconds | 0.01225 / 45 / 0 | 0.01223 / 14 / 0 | 0.01226 / 31 / 0 |
| trajectory_optimization_seconds | 0.0212 / 14 / 31 | 0.0212 / 14 / 0 | — / 0 / 31 |
| validation_seconds | 0.9558 / 14 / 31 | 0.9558 / 14 / 0 | — / 0 / 31 |
| warm_query_seconds | 0.4706 / 43 / 2 | 1.148 / 14 / 0 | 0.1435 / 29 / 2 |

## Offline roadmap cost

| Roadmap | Requested / retained nodes | Directed edges | Wall seconds | Native map bytes |
|---|---:|---:|---:|---:|
| kinova7-full-n6000-s0 | 6000 / 5981 | 72210 | 43.227026 | 3781181 |
| kinova7-full-n6000-s1 | 6000 / 5975 | 71782 | 43.116614 | 3772949 |
| kinova7-full-n6000-s2 | 6000 / 5981 | 73662 | 43.369164 | 3784953 |

Three builds total: 129.712804 seconds; native serialized maps total 11339083 bytes. Each offline roadmap is counted once; no offline cost is folded into online per-query timing. Stage timers need not sum to total wall time because initialization, serialization and other work are included in the total.

## Bounded numerical region coverage repair

The fixed handoff cap is 1e-6 rad after normalizing each inequality row. The implementation keeps the normal matrix, fixed manifest endpoints and collision models unchanged; it only increases b by the required bounded positive anchor residual. Over-cap corrections are refused. Counts below include reused and recovery regions.

| Objective | Family | Saved checks | Accepted | Rejected | Jobs with correction | Max accepted normalized correction | Max attempted correction |
|---|---|---:|---:|---:|---:|---:|---:|
| minimum_distance | all | 257 | 257 | 0 | 45 | 3.0585825e-07 | 3.0585825e-07 |
| minimum_distance | bottleneck | 91 | 91 | 0 | 21 | 3.0585825e-07 | 3.0585825e-07 |
| minimum_distance | dense_clutter | 20 | 20 | 0 | 5 | 2.3365481e-07 | 2.3365481e-07 |
| minimum_distance | route_choice | 82 | 82 | 0 | 10 | 2.7699388e-07 | 2.7699388e-07 |
| minimum_distance | sparse_clutter | 64 | 64 | 0 | 9 | 1.7607393e-07 | 1.7607393e-07 |
| minimum_time | all | 222 | 222 | 0 | 45 | 3.0585825e-07 | 3.0585825e-07 |
| minimum_time | bottleneck | 89 | 89 | 0 | 21 | 3.0585825e-07 | 3.0585825e-07 |
| minimum_time | dense_clutter | 20 | 20 | 0 | 5 | 2.3365481e-07 | 2.3365481e-07 |
| minimum_time | route_choice | 72 | 72 | 0 | 10 | 2.7699388e-07 | 2.7699388e-07 |
| minimum_time | sparse_clutter | 41 | 41 | 0 | 9 | 1.7607393e-07 | 1.7607393e-07 |

Every correction record and each rejected artifact path are retained in `development_results.json`.

## Native stderr diagnostics

Categories overlap; the original terminal failure statuses remain unchanged.

| Category | Affected jobs |
|---|---:|
| drm_no_connectable_node_assertion | 10 |
| native_assertion | 10 |
| process_signal | 10 |
| supervisor_timeout_termination | 12 |

- `dev-sparse_clutter-004-p02-EI_ZO-minimum_distance-1b976479`: timeout, exit -15; `/home/eric/wjg/wd/csdecomp/benchmarks/generic_motion_v1/results/dev_eizo_v2/jobs/dev-sparse_clutter-004-p02-EI_ZO-minimum_distance-1b976479/attempts/0001/stderr.log`.

- `dev-sparse_clutter-004-p02-EI_ZO-minimum_time-0af5b79d`: timeout, exit -15; `/home/eric/wjg/wd/csdecomp/benchmarks/generic_motion_v1/results/dev_eizo_v2/jobs/dev-sparse_clutter-004-p02-EI_ZO-minimum_time-0af5b79d/attempts/0001/stderr.log`.

- `dev-dense_clutter-003-p00-EI_ZO-minimum_distance-ef5847b2`: infrastructure_error, exit -6; `/home/eric/wjg/wd/csdecomp/benchmarks/generic_motion_v1/results/dev_eizo_v2/jobs/dev-dense_clutter-003-p00-EI_ZO-minimum_distance-ef5847b2/attempts/0001/stderr.log`.
  - Line 7: `python: csdecomp/src/planning/drm_planner.cpp:249: int32_t csdecomp::DrmPlanner::GetClosestNonCollidingConfigurationInRoadMap(const VectorXf&, Eigen::VectorXf*): Assertion 'closest_node >= 0' failed.`

- `dev-dense_clutter-003-p00-EI_ZO-minimum_time-36f03a85`: infrastructure_error, exit -6; `/home/eric/wjg/wd/csdecomp/benchmarks/generic_motion_v1/results/dev_eizo_v2/jobs/dev-dense_clutter-003-p00-EI_ZO-minimum_time-36f03a85/attempts/0001/stderr.log`.
  - Line 7: `python: csdecomp/src/planning/drm_planner.cpp:249: int32_t csdecomp::DrmPlanner::GetClosestNonCollidingConfigurationInRoadMap(const VectorXf&, Eigen::VectorXf*): Assertion 'closest_node >= 0' failed.`

- `dev-dense_clutter-003-p01-EI_ZO-minimum_distance-577c7ad4`: infrastructure_error, exit -6; `/home/eric/wjg/wd/csdecomp/benchmarks/generic_motion_v1/results/dev_eizo_v2/jobs/dev-dense_clutter-003-p01-EI_ZO-minimum_distance-577c7ad4/attempts/0001/stderr.log`.
  - Line 7: `python: csdecomp/src/planning/drm_planner.cpp:249: int32_t csdecomp::DrmPlanner::GetClosestNonCollidingConfigurationInRoadMap(const VectorXf&, Eigen::VectorXf*): Assertion 'closest_node >= 0' failed.`

- `dev-dense_clutter-003-p01-EI_ZO-minimum_time-651ba183`: infrastructure_error, exit -6; `/home/eric/wjg/wd/csdecomp/benchmarks/generic_motion_v1/results/dev_eizo_v2/jobs/dev-dense_clutter-003-p01-EI_ZO-minimum_time-651ba183/attempts/0001/stderr.log`.
  - Line 7: `python: csdecomp/src/planning/drm_planner.cpp:249: int32_t csdecomp::DrmPlanner::GetClosestNonCollidingConfigurationInRoadMap(const VectorXf&, Eigen::VectorXf*): Assertion 'closest_node >= 0' failed.`

- `dev-bottleneck-000-p02-EI_ZO-minimum_distance-1fff7e28`: timeout, exit -15; `/home/eric/wjg/wd/csdecomp/benchmarks/generic_motion_v1/results/dev_eizo_v2/jobs/dev-bottleneck-000-p02-EI_ZO-minimum_distance-1fff7e28/attempts/0001/stderr.log`.

- `dev-bottleneck-000-p02-EI_ZO-minimum_time-e0a9f09d`: timeout, exit -15; `/home/eric/wjg/wd/csdecomp/benchmarks/generic_motion_v1/results/dev_eizo_v2/jobs/dev-bottleneck-000-p02-EI_ZO-minimum_time-e0a9f09d/attempts/0001/stderr.log`.

- `dev-bottleneck-004-p01-EI_ZO-minimum_distance-28d1e757`: timeout, exit -15; `/home/eric/wjg/wd/csdecomp/benchmarks/generic_motion_v1/results/dev_eizo_v2/jobs/dev-bottleneck-004-p01-EI_ZO-minimum_distance-28d1e757/attempts/0001/stderr.log`.

- `dev-bottleneck-004-p01-EI_ZO-minimum_time-f610ef6e`: timeout, exit -15; `/home/eric/wjg/wd/csdecomp/benchmarks/generic_motion_v1/results/dev_eizo_v2/jobs/dev-bottleneck-004-p01-EI_ZO-minimum_time-f610ef6e/attempts/0001/stderr.log`.

- `dev-bottleneck-000-p02-EI_ZO-minimum_distance-9be0ba73`: timeout, exit -15; `/home/eric/wjg/wd/csdecomp/benchmarks/generic_motion_v1/results/dev_eizo_v2/jobs/dev-bottleneck-000-p02-EI_ZO-minimum_distance-9be0ba73/attempts/0001/stderr.log`.

- `dev-bottleneck-000-p02-EI_ZO-minimum_time-014ce6ec`: timeout, exit -15; `/home/eric/wjg/wd/csdecomp/benchmarks/generic_motion_v1/results/dev_eizo_v2/jobs/dev-bottleneck-000-p02-EI_ZO-minimum_time-014ce6ec/attempts/0001/stderr.log`.

- `dev-sparse_clutter-004-p01-EI_ZO-minimum_distance-da514ee1`: timeout, exit -15; `/home/eric/wjg/wd/csdecomp/benchmarks/generic_motion_v1/results/dev_eizo_v2/jobs/dev-sparse_clutter-004-p01-EI_ZO-minimum_distance-da514ee1/attempts/0001/stderr.log`.

- `dev-sparse_clutter-004-p01-EI_ZO-minimum_time-ae90d03b`: timeout, exit -15; `/home/eric/wjg/wd/csdecomp/benchmarks/generic_motion_v1/results/dev_eizo_v2/jobs/dev-sparse_clutter-004-p01-EI_ZO-minimum_time-ae90d03b/attempts/0001/stderr.log`.

- `dev-dense_clutter-004-p00-EI_ZO-minimum_distance-b3590e78`: infrastructure_error, exit -6; `/home/eric/wjg/wd/csdecomp/benchmarks/generic_motion_v1/results/dev_eizo_v2/jobs/dev-dense_clutter-004-p00-EI_ZO-minimum_distance-b3590e78/attempts/0001/stderr.log`.
  - Line 7: `python: csdecomp/src/planning/drm_planner.cpp:249: int32_t csdecomp::DrmPlanner::GetClosestNonCollidingConfigurationInRoadMap(const VectorXf&, Eigen::VectorXf*): Assertion 'closest_node >= 0' failed.`

- `dev-dense_clutter-004-p00-EI_ZO-minimum_time-6e79cbd6`: infrastructure_error, exit -6; `/home/eric/wjg/wd/csdecomp/benchmarks/generic_motion_v1/results/dev_eizo_v2/jobs/dev-dense_clutter-004-p00-EI_ZO-minimum_time-6e79cbd6/attempts/0001/stderr.log`.
  - Line 7: `python: csdecomp/src/planning/drm_planner.cpp:249: int32_t csdecomp::DrmPlanner::GetClosestNonCollidingConfigurationInRoadMap(const VectorXf&, Eigen::VectorXf*): Assertion 'closest_node >= 0' failed.`

- `dev-dense_clutter-004-p01-EI_ZO-minimum_distance-11fc00bb`: infrastructure_error, exit -6; `/home/eric/wjg/wd/csdecomp/benchmarks/generic_motion_v1/results/dev_eizo_v2/jobs/dev-dense_clutter-004-p01-EI_ZO-minimum_distance-11fc00bb/attempts/0001/stderr.log`.
  - Line 7: `python: csdecomp/src/planning/drm_planner.cpp:249: int32_t csdecomp::DrmPlanner::GetClosestNonCollidingConfigurationInRoadMap(const VectorXf&, Eigen::VectorXf*): Assertion 'closest_node >= 0' failed.`

- `dev-dense_clutter-004-p01-EI_ZO-minimum_time-1ddb2470`: infrastructure_error, exit -6; `/home/eric/wjg/wd/csdecomp/benchmarks/generic_motion_v1/results/dev_eizo_v2/jobs/dev-dense_clutter-004-p01-EI_ZO-minimum_time-1ddb2470/attempts/0001/stderr.log`.
  - Line 7: `python: csdecomp/src/planning/drm_planner.cpp:249: int32_t csdecomp::DrmPlanner::GetClosestNonCollidingConfigurationInRoadMap(const VectorXf&, Eigen::VectorXf*): Assertion 'closest_node >= 0' failed.`

- `dev-dense_clutter-004-p02-EI_ZO-minimum_distance-b35cf9e2`: infrastructure_error, exit -6; `/home/eric/wjg/wd/csdecomp/benchmarks/generic_motion_v1/results/dev_eizo_v2/jobs/dev-dense_clutter-004-p02-EI_ZO-minimum_distance-b35cf9e2/attempts/0001/stderr.log`.
  - Line 7: `python: csdecomp/src/planning/drm_planner.cpp:249: int32_t csdecomp::DrmPlanner::GetClosestNonCollidingConfigurationInRoadMap(const VectorXf&, Eigen::VectorXf*): Assertion 'closest_node >= 0' failed.`

- `dev-dense_clutter-004-p02-EI_ZO-minimum_time-c2683f2a`: infrastructure_error, exit -6; `/home/eric/wjg/wd/csdecomp/benchmarks/generic_motion_v1/results/dev_eizo_v2/jobs/dev-dense_clutter-004-p02-EI_ZO-minimum_time-c2683f2a/attempts/0001/stderr.log`.
  - Line 7: `python: csdecomp/src/planning/drm_planner.cpp:249: int32_t csdecomp::DrmPlanner::GetClosestNonCollidingConfigurationInRoadMap(const VectorXf&, Eigen::VectorXf*): Assertion 'closest_node >= 0' failed.`

- `dev-route_choice-003-p01-EI_ZO-minimum_distance-10dbcdce`: timeout, exit -15; `/home/eric/wjg/wd/csdecomp/benchmarks/generic_motion_v1/results/dev_eizo_v2/jobs/dev-route_choice-003-p01-EI_ZO-minimum_distance-10dbcdce/attempts/0001/stderr.log`.

- `dev-route_choice-003-p01-EI_ZO-minimum_time-269a6c06`: timeout, exit -15; `/home/eric/wjg/wd/csdecomp/benchmarks/generic_motion_v1/results/dev_eizo_v2/jobs/dev-route_choice-003-p01-EI_ZO-minimum_time-269a6c06/attempts/0001/stderr.log`.

A failed roadmap connection or native assertion is not evidence that the original continuous problem is globally infeasible. No failed endpoint, obstacle, radius or task objective was changed for this report.

## Interpretation limits and provenance

Ordered region/facet counts can include repeated regions; they are not unique inflation-call counts. Native pre-shortcut paths and collision-query counters are not exposed. Therefore no equal collision-query-budget claim is justified. The wall-clock budget is shared; the open-source solver route and numerical validation are described in `baseline_implementation_audit.md`. Development success-subset costs are descriptive and are not rankings against another method.

Verified 8405 files (99623693 bytes) against frozen hashes. All 360 executed evaluation configurations exactly match the single frozen manifest configuration. Run manifest SHA-256: `566fb98317896ab9a0018fe6fa4d6cbbc4d4a7862bbf96abce061dd559f78d91`. Summary SHA-256: `20888d4a5a38d05ef127d0fefce062fed75b7de758d6587616a39a635bcb82fa`.

Reproduce this CPU-only report from the repository:

```bash
bash local/python.sh benchmarks/generic_motion_v1/reports/development_result_analysis.py
```
