"""EI-ZO, native counterexample edits and restoration of original-path coverage.

Adapted from the already measured Forest pipeline; original module remains frozen.
"""

import time
import numpy as np
import csdecomp as csd
from pydrake.geometry.optimization import HPolyhedron
from reproduction.common import atomic
from reproduction.pipeline import save_candidate
from .optimization import lscs, lgcs, scsto
from .validation import validate


def native_region(region, seed=None, audit=None):
    a = np.asarray(region.A(), np.float32)
    b = np.asarray(region.b(), np.float32)
    initial_b = b.copy()
    native = csd.HPolyhedron(a, b)
    if seed is not None:
        u, v = [np.asarray(q, np.float32) for q in seed]
        if not (native.PointInSet(u, 1e-9) and native.PointInSet(v, 1e-9)):
            # Bound the rounding of a dimension-n float32 dot product (gamma_n).
            # Native Eigen and NumPy may sum in different orders. Correct only
            # the representational failure and verify with the native predicate.
            n = a.shape[1]
            eps = np.finfo(np.float32).eps
            gamma = n * eps / (1 - n * eps)
            support = np.maximum(a.astype(float) @ u, a.astype(float) @ v)
            error = gamma * np.maximum(abs(a.astype(float)) @ abs(u),
                                       abs(a.astype(float)) @ abs(v))
            b = np.nextafter(np.maximum(b, support + error).astype(np.float32),
                             np.float32(np.inf))
            if float(np.max(b.astype(float) - initial_b)) > 1e-5:
                raise ValueError("Native halfspace representation repair exceeds 1e-5")
            native = csd.HPolyhedron(a, b)
            if not (native.PointInSet(u, 1e-9) and native.PointInSet(v, 1e-9)):
                raise ValueError("Outward-rounded native region still loses its seed")
        if audit is not None:
            audit.append(dict(stage="native_edit_input",
                maximum_rhs_change=float(np.max(b.astype(float) - initial_b)),
                native_seed_containment=True))
    return native


class Pipeline:
    def __init__(self, forest, seed):
        self.forest = forest
        self.cfg = forest.cfg
        options = csd.EizoOptions()
        for key, value in self.cfg["eizo"].items():
            setattr(options, key, value)
        options.seed = seed
        options.verbose = False
        options.track_iteration_information = False
        self.inflator = csd.CudaEdgeInflator(
            forest.minimal, forest.geometry_ids, options, forest.domain
        )
        self.inflation_calls = 0
        self.roundoff = []
        self.region_representation_audit = []

    def preserve_seed(self, a, b, u, v, stage):
        """Represent a float32 native region while retaining its actual seed.

        This changes only halfspaces by their measured representation residual;
        it never changes endpoints or collision validation tolerances. Additional
        path edges must satisfy the resulting halfspaces without a slack test.
        """
        a, b = np.asarray(a, float), np.asarray(b, float)
        correction = np.maximum(np.maximum(a @ u - b, a @ v - b), 0)
        maximum = float(correction.max(initial=0))
        if maximum > 1e-5:
            raise ValueError("Native region lost a seed beyond representation tolerance")
        # nextafter supplies one outward ULP, not a geometric collision margin.
        represented = np.nextafter(b + correction, np.inf)
        self.region_representation_audit.append(
            dict(stage=stage, maximum_measured_residual=maximum,
                 adjusted_rows=int(np.count_nonzero(correction)),
                 maximum_rhs_change=float(np.max(represented - b, initial=0)))
        )
        return HPolyhedron(a, represented)

    @staticmethod
    def contains_edge(region, u, v):
        # SCSTO checks with Drake's default 1e-8. Exact halfspace containment
        # avoids accepting the former 1e-6 gap and failing at its initialization.
        return bool(np.all(region.A() @ u <= region.b()) and
                    np.all(region.A() @ v <= region.b()))

    def inflate(self, u, v):
        result = self.inflator.inflateEdge(
            np.asarray(u, np.float32),
            np.asarray(v, np.float32),
            self.forest.voxels,
            self.forest.voxel_radius,
            False,
        )
        a = np.asarray(result.A(), float)
        b = np.asarray(result.b(), float)
        correction = np.maximum(np.maximum(a @ u - b, a @ v - b), 0)
        if correction.max() > 1e-5:
            raise ValueError(
                "EI-ZO lost a seed endpoint beyond floating-point tolerance"
            )
        self.roundoff.append(float(correction.max()))
        self.inflation_calls += 1
        return self.preserve_seed(a, b, u, v, "inflate")

    def corridor(self, path, pool=None):
        sequence = []
        all_regions = list(pool or [])
        all_seeds = []
        for u, v in zip(path[:-1], path[1:]):
            found = next(
                (
                    i
                    for i, r in enumerate(all_regions)
                    if self.contains_edge(r, u, v)
                ),
                None,
            )
            if found is None:
                found = len(all_regions)
                all_regions.append(self.inflate(u, v))
                all_seeds.append((u, v))
            if not sequence or sequence[-1] != found:
                sequence.append(found)
        return [all_regions[i] for i in sequence], sequence, all_regions, all_seeds

    def initial(self, path):
        pool = []
        pool_seeds = []
        sequence = []
        for u, v in zip(path[:-1], path[1:]):
            found = next(
                (
                    i
                    for i, r in enumerate(pool)
                    if self.contains_edge(r, u, v)
                ),
                None,
            )
            if found is None:
                found = len(pool)
                pool.append(self.inflate(u, v))
                pool_seeds.append((np.array(u), np.array(v)))
            if not sequence or sequence[-1] != found:
                sequence.append(found)
        return [pool[i] for i in sequence], [pool_seeds[i] for i in sequence]

    def solve(self, method, path, drm, out):
        started = time.perf_counter()
        construction = 0.0
        optimization = 0.0
        validation_time = 0.0
        repairs = 0
        history = []
        tic = time.perf_counter()
        regions, seeds = self.initial(path)
        second_path = None
        alternate_status = "not_requested"
        if method == "LGCS":
            eligible = [
                i
                for i, r in enumerate(regions)
                if not r.PointInSet(path[0], 1e-6) and not r.PointInSet(path[-1], 1e-6)
            ]
            if eligible:
                index = np.random.default_rng(0).choice(eligible)
                try:
                    second_path = self.forest.alternate_path(drm, regions[index])
                    alternate_status = "found" if second_path is not None else "unavailable"
                except RuntimeError as exc:
                    if "No collision-free path exists in roadmap" not in str(exc):
                        raise
                    # Paper VI-C terminates the extra-path procedure on failure.
                    # The first path and its convex cover remain a valid input.
                    alternate_status = "native_no_alternate_path"
                    atomic(out / "alternate_path_failure.json", dict(
                        status=alternate_status, exception=repr(exc),
                        first_path_retained=True))
                if second_path is not None:
                    for u, v in zip(second_path[:-1], second_path[1:]):
                        if not any(
                            self.contains_edge(r, u, v)
                            for r in regions
                        ):
                            regions.append(self.inflate(u, v))
                            seeds.append((u, v))
            else:
                alternate_status = "no_eligible_interior_region"
        construction += time.perf_counter() - tic
        for recovery in range(self.cfg["maximum_recovery_iterations"] + 1):
            # Save the complete optimization input before invoking a licensed or
            # fallible solver, so even license/adapter exceptions are replayable.
            for index, region in enumerate(regions):
                np.savez_compressed(out / f"pre_solve_region_{recovery:02d}_{index:02d}.npz",
                                    A=region.A(), b=region.b())
            np.savez_compressed(out / f"pre_solve_seeds_{recovery:02d}.npz",
                                seeds=np.asarray(seeds), path=path,
                                second_path=(np.empty((0, len(path[0])))
                                             if second_path is None else second_path))
            atomic(out / f"pre_solve_{recovery:02d}.json", dict(
                method=method, iteration=recovery, region_count=len(regions),
                alternate_status=alternate_status, history=history,
                region_representation_audit=self.region_representation_audit))
            tic = time.perf_counter()
            if method == "LSCS":
                candidate = lscs(regions, path[0], path[-1])
            elif method == "LGCS":
                candidate = lgcs(regions, path[0], path[-1])
            elif method == "SCSTO":
                candidate = scsto(regions, path[0], path[-1], self.cfg)
            else:
                raise ValueError(method)
            optimization += time.perf_counter() - tic
            candidate["method"] = method
            save_candidate(out, candidate, regions, recovery)
            if not candidate["solver_success"]:
                return dict(
                    status="optimization_failed",
                    candidate=candidate,
                    regions=regions,
                    recovery_iterations=repairs,
                    construction_seconds=construction,
                    optimization_seconds=optimization,
                    total_seconds=construction + optimization,
                    wall_seconds=time.perf_counter() - started,
                    history=history,
                )
            tic = time.perf_counter()
            validation = validate(candidate, self.forest, regions)
            validation_time += time.perf_counter() - tic
            collision_count = len(validation["collision_points"])
            history.append(
                dict(
                    iteration=recovery,
                    collisions=collision_count,
                    passed=validation["passed"],
                )
            )
            atomic(
                out / f"validation_{recovery:02d}.json",
                {k: v for k, v in validation.items() if k != "_samples"},
            )
            if (
                validation["passed"]
                or not collision_count
                or recovery == self.cfg["maximum_recovery_iterations"]
            ):
                break
            tic = time.perf_counter()
            options = csd.EditRegionsOptions()
            options.configuration_margin = self.cfg["eizo"]["configuration_margin"]
            options.bisection_steps = 9
            options.verbose = False
            points = validation["collision_points"]
            indices = validation["collision_regions"]
            selected = np.arange(len(points), dtype=int)
            fixed, _ = csd.EditRegionsCuda(
                np.asarray(points[selected].T, np.float32),
                np.asarray(indices[selected], dtype=np.uint32).tolist(),
                np.asarray([u for u, v in seeds], np.float32).T,
                np.asarray([v for u, v in seeds], np.float32).T,
                [native_region(r, seed, self.region_representation_audit)
                 for r, seed in zip(regions, seeds)],
                self.forest.minimal,
                self.forest.geometry_ids,
                self.forest.voxels,
                self.forest.voxel_radius,
                options,
            )
            regions = [self.preserve_seed(r.A(), r.b(), u, v, "edit")
                       for r, (u, v) in zip(fixed, seeds)]
            # Restore the full original path coverage as required by paper VI-B.
            # When VI-C supplied a second path, retain that full cover too.
            paths = [path] + ([] if second_path is None else [second_path])
            for covered_path in paths:
                for u, v in zip(covered_path[:-1], covered_path[1:]):
                    if not any(self.contains_edge(r, u, v) for r in regions):
                        regions.append(self.inflate(u, v))
                        seeds.append((u, v))
            if method != "LGCS":
                order = []
                for u, v in zip(path[:-1], path[1:]):
                    found = next(
                        i
                        for i, r in enumerate(regions)
                        if self.contains_edge(r, u, v)
                    )
                    if not order or order[-1] != found:
                        order.append(found)
                regions = [regions[i] for i in order]
                seeds = [seeds[i] for i in order]
            atomic(out / f"restored_coverage_{recovery:02d}.json", dict(
                complete=all(any(self.contains_edge(r, u, v) for r in regions)
                             for p in paths for u, v in zip(p[:-1], p[1:])),
                paths=len(paths), regions=len(regions), containment_tolerance=0.0,
                region_representation_audit=self.region_representation_audit))
            construction += time.perf_counter() - tic
            repairs += 1
        return dict(
            status="success_numerical" if validation["passed"] else "validation_failed",
            candidate=candidate,
            regions=regions,
            validation=validation,
            recovery_iterations=repairs,
            inflation_calls=self.inflation_calls,
            roundoff_corrections=self.roundoff,
            construction_seconds=construction,
            optimization_seconds=optimization,
            validation_seconds=validation_time,
            total_seconds=construction + optimization,
            wall_seconds=time.perf_counter() - started,
            history=history,
            second_path=second_path,
            alternate_status=alternate_status,
            region_representation_audit=self.region_representation_audit,
        )
