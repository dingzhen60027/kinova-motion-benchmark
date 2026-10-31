# Independent development-result audit

Inspected all 360 jobs in `benchmarks/generic_motion_v1/results/rss2025_dev_v1`, frozen at `e1abf004b4347dff75b15a8868e14b732d3bf539`. This was read-only CPU inspection of saved evidence; no GPU, planning, or collision evaluation was run.

Verified **360 receipts and 11,822 sealed files** against their recorded SHA-256 values. Every currently inspected runtime source matches its frozen source snapshot. Recorded outcomes are 220 numerical successes, 138 frontend failures, and 2 infrastructure errors.

Audited **all 220 successful candidates**: **220 passed**, **0 rejected**. Coverage includes every actual successful candidate in every objective/family group, without selecting best cases.

| Objective | Family | Saved successes | Audit passes |
|---|---|---:|---:|
| minimum_distance | bottleneck | 45 | 45 |
| minimum_distance | dense_clutter | 8 | 8 |
| minimum_distance | route_choice | 39 | 39 |
| minimum_distance | sparse_clutter | 18 | 18 |
| minimum_time | bottleneck | 45 | 45 |
| minimum_time | dense_clutter | 8 | 8 |
| minimum_time | route_choice | 39 | 39 |
| minimum_time | sparse_clutter | 18 | 18 |

## Checks actually performed

- Reconstructed position, time and segment arrays exactly; required finite 7D endpoints, typed collision arrays, positive clearances, correct objective units and cost, joint bounds, sample-step limits, and successful saved validation.
- Verified candidate segment count against active regions; required every original DRM edge to belong to one contiguous span, seed indices to belong to that span, anchors to satisfy their halfspaces, and adjacent regions to retain their shared original waypoint. Candidate control points were independently checked against their assigned halfspaces.
- Checked the frozen model/map identity, requested roadmap profile and actual native-map hash. Confirmed every inspected successful process returned normally within its recorded outer deadline and its reported warm/cold timings satisfy their declared ordering.
- For all **110 time candidates**, evaluated position and derivatives independently with SciPy BPoly on **10,001 points per segment**. Checked endpoint rest, C0/C1 joins, derivative limits, duration sums, and consistency with the saved derivative-control bounds.

## Numerical maxima

| Quantity | Maximum observed |
|---|---:|
| maximum_endpoint_error_rad | 1.24723632597e-08 |
| maximum_sample_step_rad | 0.00499996114736 |
| maximum_scipy_position_disagreement_rad | 3.10862446895e-15 |
| maximum_scipy_abs_velocity_rad_s | 1.0000000001 |
| maximum_scipy_abs_acceleration_rad_s2 | 1.00000000099 |
| maximum_scipy_endpoint_velocity_rad_s | 1.30183188686e-10 |
| maximum_scipy_C0_C1_residual | 4.89733897879e-09 |
| maximum_handoff_normalized_correction | 0 |

The small derivative values above 1 remain within the unchanged 1e−5 numerical feasibility tolerance. Complete per-candidate values and source hashes are in the JSON.

## Recovery and branch coverage

Inspected **43 completed native edit iterations**, **175 native handoff checks**, and their saved original seed edges, collision-region indices, raw projections/optimized collisions, and recovery events. Every inspected handoff passed; 0 required a nonzero outward correction, all within the fixed 1e−6 cap.

Successful candidates with initial consecutive reuse: **0**; skipped redundant original edges: **0**. Native recovery events reinflating an original span: **0**. These are observed counts from mappings and recovery receipts, not inferred from code availability.

## Limits

- This is a read-only CPU artifact audit, not a new planner, GPU, or collision-checking run.
- Original collision arrays are inspected only after exact reconstruction of q, time and segment labels; no continuous-collision proof is claimed.
- Independent SciPy derivative sampling corroborates, and does not replace, the saved conservative full-segment derivative-control bounds.
- Warm timing remains the online portion of one cold execution, not a separate hot-repeat experiment.
- No successful dev candidate triggered initial consecutive-region reuse; that branch retains only CPU regression evidence for these successful dev records.
- No successful dev recovery triggered original-span reinflation; that branch retains CPU regression evidence, not demonstrated GPU execution in these successful dev records.

No candidate, mapping, derivative, input-contract, artifact-hash, recovery-index, or metric inconsistency was found in this complete successful-candidate audit.

Full evidence: [independent_dev_audit.json](independent_dev_audit.json). CPU audit wall time: 4.110 s.
