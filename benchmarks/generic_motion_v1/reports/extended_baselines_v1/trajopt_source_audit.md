# TrajOpt source and integration audit

The extension calls the original author's **`sco::BasicTrustRegionSQP`** and the
bundled BPMPD quadratic-program solver. It uses the frozen benchmark's seven
joint variables, joint bounds, collision primitives, collision exclusions and
online voxel spheres. The current build is ready in
`.local_setup/solver_probes/trajopt_native_v2`; Conda packages were not changed.

## Author sources and numerical method

Primary sources:

- [Schulman et al., IJRR 2014](https://rll.berkeley.edu/trajopt/ijrr/2013-IJRR-TRAJOPT.pdf),
  *Motion planning with sequential convex optimization and convex collision checking*.
- [Author TrajOpt repository at the pinned revision](https://github.com/joschu/trajopt/tree/9c634c060f7e2d61c4a888b0a5700d668a5d16f8).
- [Original trust-region SQP implementation](https://github.com/joschu/trajopt/blob/9c634c060f7e2d61c4a888b0a5700d668a5d16f8/src/sco/optimizers.cpp).
- [Author installation documentation](https://github.com/joschu/trajopt/blob/9c634c060f7e2d61c4a888b0a5700d668a5d16f8/doc/install.rst).

The pinned commit is `9c634c060f7e2d61c4a888b0a5700d668a5d16f8`.
`trajopt_author_core.tar.gz` preserves the author `src/sco`, supporting utilities,
BSD-2-Clause license and bundled Linux x86-64 BPMPD static library. The build
receipt records its SHA-256. The author documentation identifies BPMPD as the
free Linux solver distributed with TrajOpt; this run used it without a Gurobi
license. The bundled binary makes this dependency specific to the tested Linux
x86-64 environment. Its presence is not a claim that BPMPD source code was
released under the source repository's license.

The adapter supplies the exact quadratic objective
`sum_i ||q[i+1] - q[i]||²`, linearizes signed-clearance inequalities and lets the
author core apply its hinge merit penalties, trust-region acceptance/rejection,
trust-box updates and outer penalty increases. The common comparison metric is
PWL arc length `sum_i ||q[i+1] - q[i]||`; the native quadratic objective is saved
separately. It does not call IPOPT under a TrajOpt name.

`trajopt_dependency_audit.json` verifies that the extracted versions of
`optimizers.cpp`, `modeling.cpp`, `solver_interface.cpp`, `expr_ops.cpp`,
`expr_vec_ops.cpp`, `modeling_utils.cpp` and `num_diff.cpp` are byte-identical to
the pinned author sources. The C ABI problem adapter is local
`native/trajopt_bridge.cpp`.

## Geometry and fixed numerical policy

Initialization is shared with the CHOMP/STOMP adapters: add uniform arc-length
knots to a target of 64 and retain every original DRM vertex, with a maximum of
256 knots. No corner is cut by resampling. The supplied DRM endpoints are fixed
optimization bounds; their discrepancy from immutable task endpoints is checked
using the common `endpoint_tolerance_rad` and recorded. No endpoint is sampled
again for this method.

There are three collocation subdivisions per current PWL segment. At each
sample, the constraint is the minimum signed clearance over the exact canonical
static candidate pairs and all full-radius online voxel spheres, with a 1 mm
target clearance. This minimum constraint has the same collision-free feasible
set as requiring every checked pair to meet the clearance at that sample; its
hinge penalty is an aggregate minimum-distance penalty rather than the original
OpenRAVE integration's sum of individual contact penalties.

The minimum-distance derivative uses Drake's signed-distance witnesses and
point Jacobians for static geometry, plus the exact sphere/box primitive
derivative for online voxels. Feature ties, ambiguous interior box faces and
coincident centres use symmetric finite differences. This avoids changing
robot boxes into spheres. It also avoids thousands of complete geometry queries
for each ordinary SQP iteration.

The local adapter uses sampled segment constraints. It does not implement the
paper's OpenRAVE swept convex-hull collision constraints. The independent common
validator checks the returned PWL path using both CUDA and Drake at the same
finite joint-space sampling resolution as all other methods. Passing that check
is numerical collision evidence, not a mathematical continuous-safety proof.

The initial trust box is 0.1 rad, maximum SQP iterations 50, initial merit
coefficient 10, maximum penalty levels 5, constraint tolerance `1e-4`, and minimum
model improvement `1e-4`. The remaining trust-region constants are unchanged
author defaults. The optimizer has a 45-second internal cap within the root
runner's 60-second cold deadline, leaving time for final validation.

`CONVERGED` is distinct from a validated incumbent at an iteration, penalty or
internal time limit. Limit termination can publish only an actual accepted,
changed SQP iterate. A failed solver or an interrupted run with only its original
DRM initialization cannot publish that initialization as an optimized solution.
Candidate records contain `solver_converged`, `native_status`, `termination` and
the native objective. Every candidate still needs common independent validation.

## Transport failure found and repaired

The small CPU QP worked with the original BPMPD pipe transport. A real 64-knot
canonical query exposed a 2013 serializer assumption: one POSIX `read` was
expected to return an entire vector. A large sparse QP payload was split by the
pipe, the child asserted, and the parent retained an unused pipe write end and
waited for data. The actual failed log and partially written query are preserved
under `trajopt_canonical_probe_v4*`. Only that probe's parent process was stopped.

The isolated build now applies a transport-only patch: loop until the requested
byte count is transferred, handle `EINTR`, report EOF/closed peer, enforce a
monotonic transport deadline, and close the parent's unused pipe ends. The
original archive remains unchanged. `trajopt_transport.patch` and
`native/trajopt_safe_io.hpp` expose the full repair. No SQP decision rule or QP
coefficient is changed. The caller additionally needs `-no-pie` to link the
author's precompiled archive on this Ubuntu toolchain.

Successful build binary SHA-256 values:

| Binary | SHA-256 |
|---|---|
| `bpmpd_caller` | `a4005d171e882841747d342012286d883205ce7e20a52ac5b9e5c13c7b6306b6` |
| `libtrajopt_benchmark.so` | `82e9e16f13325caa36ba8ac5716d50cc3ec7aa28bff795094bc116b9a8538c40` |

Build commands, flags, archive/bridge/shim hashes and compiler identification are
in `trajopt_build.json`, `trajopt_build.log` and `trajopt_dependency_audit.json`.

## Executed checks

- A small, unobstructed seven-dimensional native CPU fixture converged in five
  QP solves, reducing its quadratic objective from 0.715 to 0.333333 while fixing
  both endpoints. Raw controls and every SQP evaluation were retained.
- Twelve random configurations were checked both without added voxels and with
  three online spheres. Signed distances matched `Model.clearance` exactly;
  maximum gradient error against central differences was `1.23e-10`.
- Twenty-four deliberately near/inside online-voxel probes matched the exact
  clearance with zero error and gradients within `1.72e-10`. Those probes include
  negative signed distances; arrays and all individual errors are saved.
- The default 64-knot policy ran on the frozen `smoke-bottleneck-000-p00` task
  using its previously validated DRM seed for this engineering check. The author
  core converged after 14 QP solves in 2.630839 seconds. Its returned path passed
  the common CUDA/Drake validator, with PWL length 7.814311 rad. The complete
  evidence is `trajopt_canonical_probe_v5/` and its log. This adapter check is not
  pooled into the new timed comparison matrix.
- `test_trajopt_baseline.py`: 24 tests passed. These include actual native
  convergence, an accepted changed iterate at a one-iteration cap, preserved
  callback failures, the common endpoint-tolerance policy, fixed endpoints,
  source/build availability and real canonical geometry derivatives.

`trajopt_probe_history.json` retains failed attempts, including the incorrect
probe manifest key, an initially shadowed Python variable, the rejection of a
partial failed-DRM path, an empty overly strict gradient-probe selection and the
large-QP pipe failure. Failed evidence was not converted into success records.

## Commands

Run from the repository root, using the existing Conda launcher:

```bash
bash local/python.sh -m benchmarks.generic_motion_v1.runners.trajopt_baseline --build
bash local/python.sh -m benchmarks.generic_motion_v1.runners.trajopt_baseline
bash local/python.sh -m pytest -q benchmarks/generic_motion_v1/tests/test_trajopt_baseline.py
timeout 65s bash local/python.sh benchmarks/generic_motion_v1/reports/extended_baselines_v1/trajopt_probe.py --out benchmarks/generic_motion_v1/reports/extended_baselines_v1/trajopt_canonical_probe_v5
```

The probe deliberately refuses to overwrite its output directory; select a new
path for a repeat. Native source compilation is offline setup and is not counted
as online planning. The parent experiment driver creates fresh queries, applies
the common timing protocol, and reports its own smoke/dev observations separately.
