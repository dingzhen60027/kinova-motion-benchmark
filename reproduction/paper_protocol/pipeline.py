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


def native_region(region):
    return csd.HPolyhedron(
        np.asarray(region.A(), np.float32), np.asarray(region.b(), np.float32)
    )


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
        return HPolyhedron(a, b + correction + 1e-8)

    def corridor(self, path, pool=None):
        sequence = []
        all_regions = list(pool or [])
        all_seeds = []
        for u, v in zip(path[:-1], path[1:]):
            found = next(
                (
                    i
                    for i, r in enumerate(all_regions)
                    if r.PointInSet(u, 1e-7) and r.PointInSet(v, 1e-7)
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
                    if r.PointInSet(u, 1e-7) and r.PointInSet(v, 1e-7)
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
        if method == "LGCS":
            eligible = [
                i
                for i, r in enumerate(regions)
                if not r.PointInSet(path[0], 1e-6) and not r.PointInSet(path[-1], 1e-6)
            ]
            if eligible:
                index = np.random.default_rng(0).choice(eligible)
                second_path = self.forest.alternate_path(drm, regions[index])
                if second_path is not None:
                    for u, v in zip(second_path[:-1], second_path[1:]):
                        if not any(
                            r.PointInSet(u, 1e-7) and r.PointInSet(v, 1e-7)
                            for r in regions
                        ):
                            regions.append(self.inflate(u, v))
                            seeds.append((u, v))
        construction += time.perf_counter() - tic
        for recovery in range(self.cfg["maximum_recovery_iterations"] + 1):
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
                [native_region(r) for r in regions],
                self.forest.minimal,
                self.forest.geometry_ids,
                self.forest.voxels,
                self.forest.voxel_radius,
                options,
            )
            regions = [
                HPolyhedron(np.asarray(r.A(), float), np.asarray(r.b(), float))
                for r in fixed
            ]
            # Restore the full original path coverage as required by paper VI-B.
            for u, v in zip(path[:-1], path[1:]):
                if not any(
                    r.PointInSet(u, 1e-6) and r.PointInSet(v, 1e-6) for r in regions
                ):
                    regions.append(self.inflate(u, v))
                    seeds.append((u, v))
            if method != "LGCS":
                order = []
                for u, v in zip(path[:-1], path[1:]):
                    found = next(
                        i
                        for i, r in enumerate(regions)
                        if r.PointInSet(u, 1e-6) and r.PointInSet(v, 1e-6)
                    )
                    if not order or order[-1] != found:
                        order.append(found)
                regions = [regions[i] for i in order]
                seeds = [seeds[i] for i in order]
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
        )
