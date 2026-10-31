# Public iiwa collision-throughput diagnostic

The actual run completed with exit code 0: five timed batches measured
11.63699 ± 0.18358 million configurations/s (sample standard deviation), with
zero disagreements across 2,000 independent Drake checks. The measured model
contained 42 robot–robot and 10 robot–table candidate pairs. Raw data are in
`reproduction/runs/20261025T100718.382125Z_public_iiwa_throughput`, indexed by
`throughput_index.json`. This remains an additional public-model diagnostic,
not a recovered hardware experiment.

To run another separately recorded diagnostic:

```sh
bash local/python.sh -m reproduction.throughput_public_iiwa
```

The driver reuses `probe_public_iiwa.py`'s two hash-pinned public URDFs, 11 robot
boxes, declared gripper weld and declared example table. It records the actual
Drake/native candidate-pair list, separating robot–robot from robot–table pairs.
The existing probe measured 42 robot–robot pairs and 10 robot–table pairs; the new
driver will measure these counts again and must not call the aggregate self-only.
This is not the paper's original 105-self-pair model.

It samples and saves exactly 2,000,000 float32 joint configurations with seed
250410783, performs one full-size warmup and five full-size timed batches through
the synchronous public CUDA wrapper, and retains every returned flag array.
Sampling, file IO and independent checking are outside the measured interval;
allocation, transfers and Python flag conversion are inside it. At least 2,000
evenly spaced indices are checked using Drake signed distances at the exact
float32 inputs, evaluated in float64. Any discrepancy fails the diagnostic and
remains in the saved raw arrays and failure manifest.

Each output directory contains the assembled URDF, source/runtime/native-binary
hashes, model/pair metadata, full configuration array, six flag archives, all
timings, independent comparisons and summary. Runtime/thread flags are recorded;
unavailable installed-binary build flags are explicitly unknown rather than
inferred. Existing Conda packages are used unchanged.

The public assembly has different geometry, filters and hardware from Appendix D.
No speed ratio to the paper's 6.9 million configurations/s will be reported. This
measurement cannot supply the 15 missing physical trajectories, camera streams,
calibration or Table II perception/control timings.
