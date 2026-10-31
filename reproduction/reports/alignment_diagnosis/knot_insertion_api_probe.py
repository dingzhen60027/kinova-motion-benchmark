"""CPU-only check that KTO preserves InsertKnots initial curves and their derivatives."""
import json
from pathlib import Path
import numpy as np
from pydrake.all import BsplineBasis, BsplineTrajectory, KnotVectorType, KinematicTrajectoryOptimization

rows = []
for count in [2, 3, 4, 5]:
    path = np.random.default_rng(192 + count).normal(size=(count, 3))
    controls = np.vstack([path[0], path, path[-1]])
    duration = float(np.linalg.norm(np.diff(controls, axis=0), axis=1).sum() + 2)
    initial = BsplineTrajectory(
        BsplineBasis(4, len(controls), KnotVectorType.kClampedUniform, 0, duration),
        [q[:, None] for q in controls])
    original = initial.Clone()
    times = np.linspace(0, duration, 1001)
    for target in [16, 32]:
        while len(initial.control_points()) < target:
            unique = np.unique(initial.basis().knots())
            i = int(np.argmax(np.diff(unique)))
            initial.InsertKnots([float((unique[i] + unique[i + 1]) / 2)])
        kto = KinematicTrajectoryOptimization(initial)
        prog = kto.prog()
        c = prog.GetInitialGuess(kto.control_points())
        t = float(prog.GetInitialGuess(kto.duration()))
        restored = BsplineTrajectory(
            BsplineBasis(4, [float(k * t) for k in kto.basis().knots()]),
            [q[:, None] for q in c.T])
        row = dict(original_nodes=count, target_controls=target,
                   duration_error=abs(t - duration),
                   normalized_knot_error=float(np.max(np.abs(
                       np.array(kto.basis().knots()) * duration - initial.basis().knots()))),
                   position_error=float(np.abs(restored.vector_values(times) - original.vector_values(times)).max()),
                   velocity_error=float(np.abs(restored.MakeDerivative(1).vector_values(times)
                                                - original.MakeDerivative(1).vector_values(times)).max()),
                   acceleration_error=float(np.abs(restored.MakeDerivative(2).vector_values(times)
                                                    - original.MakeDerivative(2).vector_values(times)).max()))
        rows.append(row)

# Same-input endpoint identities used by the ablation worker.
endpoint_rows = []
root = Path(__file__).resolve().parents[3]
for name, size in [('20261026T025110.685774Z_paper_constructed_forest', 200),
                   ('20261026T025846.407684Z_paper_constructed_fr3', 3000)]:
    for case in sorted((root / 'reproduction/runs' / name / 'cases').glob(f'e*_r00_n{size:05d}')):
        data = json.loads((case / 'input.json').read_text())
        path = np.load(case / 'seed_path.npz')['path']
        endpoint_rows.append(dict(run=name, case=case.name,
                                  start_error=float(np.abs(path[0] - data['start']).max()),
                                  goal_error=float(np.abs(path[-1] - data['goal']).max())))

output = dict(rows=rows, endpoint_rows=endpoint_rows, seed_rule='192+count',
              kind='CPU initial-guess and derivative consistency, no optimization or geometry')
Path(__file__).with_suffix('.json').write_text(json.dumps(output, indent=2, allow_nan=False) + '\n')
print(json.dumps(output, indent=2))
