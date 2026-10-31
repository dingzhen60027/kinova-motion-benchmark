# Original experiment assets needed

**Status: draft only; not sent.** This list follows the public-source audit and
the actual reconstruction failures. It is not an assertion that the authors
must release private assets.

Subject: Reproduction assets for arXiv:2504.10783

We are reproducing *Superfast Configuration-Space Convex Set Computation on
GPUs for Online Motion Planning*. We have implemented the published simulation
parameters, native DRM/EI-ZO, LSCS, LGCS, SCSTO, and Drake NTO, and retained all
failed cases. We also examined the original arXiv TeX and the public repository
at its prepublication revision ee94f8145bb32ef6c08b1e575218335b9daf533f.

Could you share the following experiment assets, or point to a public archive?

1. The original FR3 model with all 33 sphere centers/radii, transforms, attached
   gripper, table, joint/derivative bounds, and collision filtering. Appendix A
   reports 1187 pairs, including 660 dynamic pairs. Our fit has only 640 dynamic
   pairs, and independent official-mesh checks establish a false link5–hand
   collision in our fitted model. Reconstructing the pair count alone cannot
   recover the original geometry.
2. The Forest and FR3 scene generators and input seeds, including obstacle
   distributions, endpoint/target selection, pose-map IK settings, and whether
   any task rejection occurred before the reported ten-by-ten-by-four runs.
3. The exact experiment source revision and dependency versions, including
   trajectory optimization, region recovery, collision checks, and timing. The
   prepublication public revision lacks the later EditRegionsCuda interface.
4. NTO spline degree/order, knot/control-point construction and endpoint
   repetition, exact collision parameter locations and counts, derivative
   bounds, initial duration, solver options and stopping rules. The arXiv TeX
   comment lists k/Nc for k=1,...,Nc-1, while the published text describes Nc
   collision points.
5. The simulation LSCS/LGCS graph and solver settings, SCSTO Bezier degree and
   recovery budgets, and clarification of the printed Eq. (19d) equality
   between adjacent path knots. An executable small reference case would resolve transcription
   ambiguities without requiring all raw data.
6. For Table II, the 11-box iiwa+gripper model and filters, table and sensor
   calibration, depth/voxel logs, the 15 motion inputs and execution records,
   and the breakdown of the reported perception/planning/communication latency.

We can share our command logs, environment lock, fixed inputs, trajectory
validation and minimal counterexamples. We will distinguish reconstructed
experiments from original-asset reproduction in all reporting.
