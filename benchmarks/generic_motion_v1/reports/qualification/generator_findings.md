# Generator and frozen-input qualification

This audit examines input construction and admission. It does not execute a
planner on the formal test split. The original frozen manifests, scenes,
arrays, configuration, receipts and checksum list are retained.

## Reproduced defects and repairs

| Severity | Reproduced defect | Consequence and repair |
|---|---|---|
| P1 | Deleting all `route_choice` rows from a mock smoke manifest, then updating its manifest hash, was accepted: 8 cases instead of the configured 10. | The previous audit did not establish dataset completeness. Preflight now derives all expected IDs and counts from the frozen split configuration, rejecting omissions, duplicates and unexpected IDs. |
| P1 | Deleting a raw input NPZ and its receipt entry was accepted. | A claimed successful input audit did not guarantee that endpoint proposals and straight-line observations were retained. Canonical input paths, row hashes, full receipt inventory and actual files are now mandatory. |
| P1 | Changing endpoint and method seeds, with row/manifest hashes updated, was accepted. | Randomness provenance was not verified. All domain seeds and paired seeds are derived again; every endpoint proposal is replayed from PCG64, including rejected proposals and unused batch tails. First-accepted start/goal selection and rejection counts are reconstructed. |
| P1 | A negative joint margin of −0.5 rad produced an accepted pair containing 3 joint-limit violations in the deterministic mock reproduction. | The public configurable endpoint generator could weaken its own limits. Nonfinite/negative margins, invalid bins, malformed counts and unsupported model intervals now fail before sampling. The actual frozen configuration uses +0.02 rad and is not affected by this parameter bug. |
| P2 | Raw observations, endpoint TCP metadata, status labels and numerical-line statistics could be changed without semantic rechecking. | Full qualification now independently repeats both collision predicates for all saved proposals and line samples, checks TCP/clearance metadata and rejects unknown statuses. No planning result is consulted. |
| P2 | At a world x-coordinate of `2**60`, local grid offsets disappeared during float64 addition, yet the previous voxelizer reported a positive 5.358981 mm coverage margin for a radius-0.1 m source sphere covered by radius-0.04 m balls. | A float32-cast error alone did not bound all coordinate arithmetic. A conservative transform-roundoff budget now rejects this unrepresentable geometry. Nonfinite yaw, float32 overflow and excessive cell counts are also rejected. Frozen metre-scale scenes are unaffected. |

`generator_adversarial_before.json` and `generator_adversarial_after.json`
contain the actual before/after results for the first four defects. These are
CPU unit reproductions with an explicitly geometry-free mock robot, not
Kinova experiment results. The focused test log is `generator_pytest.log`:
56 tests passed, including the unchanged default deterministic generation and
conservative-coverage tests.

The modified code preserves the default RNG streams, proposal order,
geometry, selected endpoints, and saved input formats. A current source hash
must differ from its archived predecessor; the old hash is not rewritten.
The passing current G0 receipt must match the actual model source, robot and
collision-geometry hashes. New input audit reports separately record the
historical generation provenance and the current verification provenance.

## Limits retained explicitly

- A complete self-consistent receipt is not a cryptographic signature from a
  trusted publisher. The original frozen checksum list and archived generator
  sources remain the historical trust anchor; a fully rewritten dataset and
  rewritten receipt must not be treated as the old dataset merely because
  their internal hashes agree.
- `regenerate=False` skips fresh collision checks of all rejected proposals.
  Its report explicitly sets `proposal_collision_checks_repeated=false`.
  Qualification uses the default `regenerate=True`.
- `bottleneck` and `route_choice` centre ranges are currently constants in
  `scene.py`; `roi_min_m`/`roi_max_m` directly control the clutter families.
  Therefore v1 is not fully configurable by the JSON file alone. The source
  hashes freeze these ranges. Unifying these ranges would change the scene
  distribution and belongs in a new protocol, not a silent v1 repair.
- Conservative spheres may reject motion that would clear the original
  physical geometry. The original geometry, cell bounds, quantization bounds
  and ball radius remain available. No radius or robot model was changed to
  increase a method's success rate.
- Invalid endpoint searches remain in each split, with all proposals and
  rejection counts. Their presence does not establish global infeasibility.
  Straight-line and trajectory collision checks remain numerical sampling;
  their acceptance is not a continuous collision-free proof.

## Commands and actual full-input evidence

Run from the repository root using its existing Conda launcher:

```bash
bash local/python.sh -m benchmarks.generic_motion_v1.reports.qualification.generator_adversarial
bash local/python.sh -m pytest benchmarks/generic_motion_v1/tests/test_generator.py benchmarks/generic_motion_v1/tests/test_generator_integrity.py -q
bash local/python.sh -m benchmarks.generic_motion_v1.reports.qualification.generator_reaudit
```

The full input-only run writes new `generator_{smoke,dev,test}_audit.json`
files and `generator_reaudit_receipt.json`. The latter records actual
`passed` flags, errors, elapsed time, proposal counts, report hashes, and
manifest hashes before/after. The runner refuses to overwrite these evidence
files and returns a nonzero exit code if any split fails. Full stdout is
saved separately as `generator_reaudit.log`.

The actual input-only requalification completed successfully:

| Split | Cases / scene IDs | Valid / invalid endpoints | Raw proposals replayed and rechecked | Straight-line samples | Elapsed seconds |
|---|---:|---:|---:|---:|---:|
| smoke | 10 / 5 | 8 / 2 | 8,768 | 1,004 | 5.584 |
| dev | 60 / 20 | 60 / 0 | 4,544 | 8,544 | 11.647 |
| test | 420 / 84 | 395 / 25 | 133,504 | 51,463 | 123.138 |

All three `passed` flags are true. All case/scene errors are empty. The run
checked 146,816 proposal observations and 61,011 straight-line observations;
these are check counts, not claims of independent random environments.
All valid endpoint penetration counts are zero. Non-open scene geometries,
individual endpoints, endpoint pairs, scene seeds and endpoint seeds have no
cross-split intersection. Open-control layouts still represent the same base
geometry and are not counted as independent random environments.

The exact original manifest SHA256 values remained unchanged:

```text
smoke 92e19a293861fa1b93a2dba3919ae3bc3eec584f3845709dc99f2ce98e453720
dev   32de21ac2474846ae3cdc9867a3f41585cf80e7d0d4531e26285de216ef56000
test  9fcbe5ccae28d8681586b1ac6c9986f51f7cc7b5cdc19352c101910e3b6ed03b
```

The 27 originally invalid endpoint searches remain present. The adversarial
defects concern missing admission checks and unsafe nondefault inputs; this
full requalification found no corresponding corruption of the frozen v1
inputs. See `generator_qualification_summary.json` for aggregate check
counts, collision-disagreement counts and retained limitations.
