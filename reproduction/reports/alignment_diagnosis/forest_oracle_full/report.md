# Full Forest exact segment/circle oracle diagnosis

All **400 original Forest cases** were solved again. None reused a ten-case pilot solution, and none was removed or regenerated. The original DRM node count, initial path, endpoints, domain, PWL path-length objective and SNOPT settings were retained. The sole optimization-formulation change was replacing Nc pointwise collision constraints by every segment/circle exact clearance constraint with a **1 mm margin**. This is a mechanism diagnostic, **not the paper's NTO**, and does not replace the original 30% result.

| Measurement | Full oracle result |
|---|---:|
| Scheduled and actually solved | 400 / 400 |
| SNOPT reported success | 400 / 400 |
| Continuous PWL/circle zero-collision check | 400 / 400 |
| Full acceptance including 1 mm margin at 1e−6 m tolerance | **399 / 400** |
| Originally invalid DRM seeds retained and accepted | 12 / 12 |
| Nontrivial multi-segment cases fully accepted | 279 / 280 |
| Original pointwise NTO accepted on the same 400 inputs | 120 / 400 |

One case remains a documented failure: **e09_r04_n00200**. SNOPT returns success and the continuous path is collision-free, but its minimum clearance is **0.998438989 mm**. This misses the requested 1 mm margin by **1.561011 µm**, exceeding the unchanged 1 µm acceptance tolerance. The record remains `validation_failed`; no tolerance was relaxed and it was not rerun until successful.

| Environment seed | Cases | Original NTO strict successes | Oracle fully accepted | Oracle zero-collision candidates |
|---|---:|---:|---:|---:|
| 0 | 40 | 0 | 40 | 40 |
| 1 | 40 | 40 | 40 | 40 |
| 2 | 40 | 0 | 40 | 40 |
| 3 | 40 | 0 | 40 | 40 |
| 4 | 40 | 40 | 40 | 40 |
| 5 | 40 | 40 | 40 | 40 |
| 6 | 40 | 0 | 40 | 40 |
| 7 | 40 | 0 | 40 | 40 |
| 8 | 40 | 0 | 40 | 40 |
| 9 | 40 | 0 | 39 | 40 |

All 400 analytic distance Jacobians and path-length gradients passed central finite-difference checks, including AutoDiff identity/chain propagation controls. A separate known feasible detour was solved; a fixed direct path through a circle was correctly reported infeasible. The raw unsuccessful negative-control candidate is also retained. All 1,602 original input files and the frozen executable sources verified unchanged during this run. The independent postrun line-circle helper reproduced every candidate clearance within **1.22e-15 m**, found no collision, checked identical node counts and original path arrays, and confirmed no final length exceeded its DRM warm-start length. This check preserves the one margin failure.

The same-node-count oracle supports a narrower conclusion than paper reproduction: the current path representations and warm starts can support nontrivial valid PWL optimization when collision avoidance is enforced continuously against the declared circles. The earlier failure pattern therefore cannot be attributed simply to insufficient original control points or universally impossible scenes. Exact constraints are stronger and have different nonlinear derivatives than the pointwise Drake transcription; they do not recover the authors' undisclosed initialization, clearance or collocation policy. Local SNOPT success is not a proof of global shortest paths.

`results.csv` contains all 400 original/new costs and states, with originally colliding NTO costs explicitly tagged by their saved status. `cases/*` retains original inputs, full candidate control points, every exact-constraint distance/residual, solver details, SNOPT logs, original NTO results and independent validation. `manifest.json` contains the fixed selection rule, configuration, command, runtime and all input/source hashes. `forest_oracle.py` is the exact runtime source snapshot. `report_generated.md` retains the mechanically generated per-case table. The separate ten-case pilot and its failed implementation preflight are unchanged. Timings are diagnostic only and are not used to compare with the paper.

Actual command:

```bash
bash local/python.sh -m reproduction.alignment_diagnosis.forest_oracle --all-cases --output reproduction/reports/alignment_diagnosis/forest_oracle_full
```
