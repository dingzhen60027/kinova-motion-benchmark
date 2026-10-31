# Test split freeze audit

This is the RSS-inspired Kinova-7D adaptation. It is not an exact reproduction of the FR3 paper benchmark.

Frozen **420 cases / 84 layout IDs** after recorded G0/G1/G2 acceptance. Input audit passed: **True**. No test planning jobs were created or run.

Legal inputs: **395**; invalid inputs retained: **25**. Numerically colliding straight lines: **290/395** legal inputs. Global problem feasibility remains unknown; finite sampling is not a continuous collision certificate.

| Family | Cases | Layout IDs | Legal / invalid | Straight blocked | Obstacles min/median/max | Voxels min/median/max |
|---|---:|---:|---:|---:|---|---|
| bottleneck | 100 | 20 | 100 / 0 | 74/100 | 2/2/2 | 320/320/320 |
| dense_clutter | 100 | 20 | 85 / 15 | 74/85 | 20/26.5/40 | 331/476.5/757 |
| open_control | 20 | 4 | 20 / 0 | 9/20 | 0/0/0 | 0/0/0 |
| route_choice | 100 | 20 | 95 / 5 | 65/95 | 12/12/12 | 285/350.5/453 |
| sparse_clutter | 100 | 20 | 95 / 5 | 68/95 | 6/9/12 | 85/219.5/415 |

The four open-control layout IDs represent one actual geometry. The other 80 layouts have 80 distinct random geometries. No non-open geometry, scene seed, endpoint seed, or individual endpoint configuration is shared with smoke or dev.

All invalid cases and rejection counts remain in the frozen manifest. Scene rejection uses fixed-equipment overlap only; endpoint rejection uses collision, clearance, limits and predeclared distance bins. No planner outcome is used for selection.

The JSON companion includes per-family distance distributions, raw-voxel memory counts, collision disagreements, all SHA256 values and actual CLI timing records. Both original geometry and its conservative 0.04 m sphere cover are preserved.

- Manifest SHA256: `9fcbe5ccae28d8681586b1ac6c9986f51f7cc7b5cdc19352c101910e3b6ed03b`
- Generator / copied test config SHA256: `ac60e554b442d10bf820b642fc4425d4619f12fd8c2d236f1cee10c7c91a0f8f`
- Geometry protocol SHA256: `d10cb605984c58a9b901b7c29eb134b3391f95d86d8fd8770f2942f25ee97f4e`
- SHA256SUMS entries: **518**; checksum file: `manifests/test.SHA256SUMS`.

Commands executed from the repository root:

```bash
bash local/python.sh -m benchmarks.generic_motion_v1 generate --split test --seed 20261027 --output benchmarks/generic_motion_v1 --acceptance benchmarks/generic_motion_v1/reports/acceptance.json
bash local/python.sh -m benchmarks.generic_motion_v1 audit --manifest benchmarks/generic_motion_v1/manifests/test.jsonl --output benchmarks/generic_motion_v1/reports/test_input_audit.json
bash local/python.sh benchmarks/generic_motion_v1/reports/test_freeze_analysis.py
sha256sum -c benchmarks/generic_motion_v1/manifests/test.SHA256SUMS
```

Full generation and audit stdout are saved in `reports/test_freeze_generate.stdout.log` and `reports/test_freeze_input_audit.stdout.log`.
