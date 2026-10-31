"""Frozen full Kinova model shared by offline DRM and every online method.

The independent checker uses Drake's double precision geometry queries and
forward kinematics. Online sphere-to-box distances are evaluated analytically
from Drake poses; no planner collision code is called by that checker.
"""
from __future__ import annotations

import argparse
import hashlib
import itertools
import json
from pathlib import Path
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
ASSETS = ROOT / 'csdecomp/tests/test_assets'
DIRECTIVE = ASSETS / 'directives/kinova_sens_on_table.yml'
CONTACT_BAND_M = 2e-6


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def _configuration_array(q):
    try:
        q = np.asarray(q, dtype=float)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError('Configurations must be finite N x 7 arrays representable as float32.') from exc
    if q.ndim == 1:
        q = q.reshape(1, -1)
    if (q.ndim != 2 or q.shape[1] != 7 or not np.isfinite(q).all()
            or np.any(np.abs(q) > np.finfo(np.float32).max)):
        raise ValueError('Configurations must be finite N x 7 arrays representable as float32.')
    return q


class Model:
    """Canonical full model; configurations are always rows, in URDF order."""

    dimension = 7
    tcp_name = 'kinova::end_effector_link'
    contact_band_m = CONTACT_BAND_M

    def __init__(self, voxels_xyz=None, voxel_radius=0.04):
        import csdecomp as csd
        from pydrake.all import (RobotDiagramBuilder, LoadModelDirectives,
                                ProcessModelDirectives, Role, Sphere, Box)
        self.csd = csd
        builder = RobotDiagramBuilder()
        parser = builder.parser()
        self.plant = builder.plant()
        self.scene_graph = builder.scene_graph()
        parser.package_map().Add('test_assets', str(ASSETS))
        parser.package_map().Add('kortex_description', str(ASSETS / 'kinova/kortex_description'))
        parser.package_map().Add('robotiq_arg85_description', str(ASSETS / 'robotiq'))
        ProcessModelDirectives(LoadModelDirectives(str(DIRECTIVE)), self.plant, parser)
        self.plant.Finalize()
        self.diagram = builder.Build()
        self.context = self.diagram.CreateDefaultContext()
        self.plant_context = self.plant.GetMyContextFromRoot(self.context)
        self.scene_context = self.scene_graph.GetMyContextFromRoot(self.context)
        inspector = self._query().inspector()
        self.inspector = inspector
        self.csd_plant = csd.convert_drake_plant_to_csd_plant(self.plant, self.plant_context, inspector)
        self.mplant = self.csd_plant.getMinimalPlant()
        self.minimal = self.mplant
        self.robot_geometry_ids = list(self.csd_plant.getRobotGeometryIds())
        self.lower = np.asarray(self.csd_plant.getPositionLowerLimits(), dtype=float)
        self.upper = np.asarray(self.csd_plant.getPositionUpperLimits(), dtype=float)
        self.joint_names = list(self.plant.GetPositionNames())
        active_csd = [joint.name + '_q' for joint in self.csd_plant.getKinematicTree().get_joints() if int(joint.type) != 0]
        if active_csd != self.joint_names:
            raise RuntimeError('CSD/Drake active joint order differs.')
        if self.plant.num_positions() != self.dimension:
            raise RuntimeError('Canonical Kinova must have exactly seven active joints.')
        if not np.allclose(self.lower, self.plant.GetPositionLowerLimits(), atol=2e-7, rtol=0):
            raise RuntimeError('CSD/Drake joint lower limits differ.')
        if not np.allclose(self.upper, self.plant.GetPositionUpperLimits(), atol=2e-7, rtol=0):
            raise RuntimeError('CSD/Drake joint upper limits differ.')
        self._drake_ids = []
        for frame in inspector.GetAllFrameIds():
            self._drake_ids.extend(inspector.GetGeometries(frame, Role.kProximity))
        if len(self._drake_ids) != self.mplant.num_scene_geometries:
            raise RuntimeError('Drake/CSD geometry mapping is incomplete.')
        self.pairs = np.asarray(self.mplant.collision_pairs_flat, dtype=int).reshape(-1, 2).copy()
        drake_pairs = {frozenset(pair) for pair in inspector.GetCollisionCandidates()}
        csd_pairs = {frozenset((self._drake_ids[a], self._drake_ids[b])) for a, b in self.pairs}
        if drake_pairs != csd_pairs:
            raise RuntimeError('Drake/CSD collision filters differ; generation is forbidden.')
        self._robot_geometries = []
        geometry_inventory = []
        for index, geometry_id in enumerate(self._drake_ids):
            shape = inspector.GetShape(geometry_id)
            geometry = self.mplant.scene_collision_geometries[index]
            if isinstance(shape, Box):
                kind, dimensions = 'box', shape.size().tolist()
            elif isinstance(shape, Sphere):
                kind, dimensions = 'sphere', [shape.radius()]
            else:
                raise RuntimeError(f'Unsupported canonical primitive {type(shape).__name__}.')
            if not np.allclose(dimensions, np.asarray(geometry.dimensions).ravel()[:len(dimensions)], atol=2e-7, rtol=0):
                raise RuntimeError('Geometry index/shape mapping differs in the CSD bridge.')
            entry = dict(index=index, name=inspector.GetName(geometry_id), shape=kind,
                         dimensions_m=dimensions,
                         local_transform=inspector.GetPoseInFrame(geometry_id).GetAsMatrix4().tolist(),
                         csd_dimensions_m=np.asarray(geometry.dimensions).tolist(),
                         csd_local_transform=np.asarray(geometry.X_L_B).tolist(),
                         csd_link_index=int(geometry.link_index),
                         moving_robot=index in self.robot_geometry_ids)
            geometry_inventory.append(entry)
            if index in self.robot_geometry_ids:
                self._robot_geometries.append((geometry_id, kind, np.asarray(dimensions)))
        self._tcp_body = self.plant.GetBodyByName('end_effector_link', self.plant.GetModelInstanceByName('kinova'))
        sources = [DIRECTIVE, ASSETS / 'kinova_sens.urdf',
                   ASSETS / 'robotiq_2f85_description_box_welded_fingers.urdf',
                   ASSETS / 'table.urdf', ASSETS / 'camera_and_tripod.urdf', ASSETS / 'box.urdf']
        self.sources = {str(p.relative_to(ROOT)): sha256(p) for p in sources}
        self.robot_sha256 = digest(dict(sources=self.sources, joint_names=self.joint_names,
                                      lower=self.lower.tolist(), upper=self.upper.tolist(),
                                      world_frame='world; right-handed; metres; base_link welded at identity',
                                      tcp=self.tcp_name))
        self.collision_geometry_sha256 = digest(dict(geometries=geometry_inventory,
                                                   checked_pairs=self.pairs.tolist(),
                                                   robot_geometry_ids=self.robot_geometry_ids,
                                                   online_sphere_radius_m=0.04,
                                                   contact_band_m=CONTACT_BAND_M))
        checked = {tuple(pair) for pair in self.pairs.tolist()}
        self.inventory = dict(
            protocol='kinova7_generic_v1_full_primitive_boxes',
            fidelity='RSS-inspired Kinova-7D adaptation; not the paper FR3 model',
            robot_sha256=self.robot_sha256,
            collision_geometry_sha256=self.collision_geometry_sha256,
            source_files_sha256=self.sources,
            canonical_directive=str(DIRECTIVE.relative_to(ROOT)),
            joint_names=self.joint_names, lower=self.lower.tolist(), upper=self.upper.tolist(),
            drake_lower=self.plant.GetPositionLowerLimits().tolist(),
            drake_upper=self.plant.GetPositionUpperLimits().tolist(),
            dimension=self.dimension, tcp=self.tcp_name,
            base_transform_world=np.eye(4).tolist(),
            world_frame='right-handed world; coordinates and distances in metres',
            geometries=geometry_inventory, robot_geometry_ids=self.robot_geometry_ids,
            checked_pairs=self.pairs.tolist(),
            excluded_pairs=[list(p) for p in itertools.combinations(range(len(self._drake_ids)), 2) if p not in checked],
            filter_policy='Unmodified CSD finalize_scene: exclude world-world, same link, adjacent links and fixed-connected links. Exactly equal to native Drake candidates for this model.',
            fixed_base_vs_online_voxels='CSD treats world-welded base as world, excludes it from online sphere tests; generator separately audits obstacle overlap with fixed geometry.',
            independent_checker='Drake signed primitive distances and Drake forward poses; analytic double precision sphere-to-box signed distance for online voxels',
            contact_band_m=CONTACT_BAND_M,
            boundary_policy='Distances within 2 micrometres of contact are numerically ambiguous; keep raw CUDA and independent results, require both free and clearance above band for validation.',
            offline_online_identity='Every builder and runner constructs this same Model; legacy simple-model roadmaps must not be loaded.',
            zero_configuration_geometry_world_poses=[self._query().GetPoseInWorld(g).GetAsMatrix4().tolist() for g in self._drake_ids],
        )
        self.set_voxels(voxels_xyz, voxel_radius)

    def nto_plant(self):
        """Build the same collision model with online spheres for Drake NTO.

        The ordinary validator keeps spheres in an array for efficient batched
        distance queries. Drake's native MinimumDistanceLowerBoundConstraint
        needs them registered in SceneGraph; this explicitly constructs that
        equivalent representation without changing the canonical CSD plant.
        """
        from types import SimpleNamespace
        from pydrake.all import (RobotDiagramBuilder, LoadModelDirectives,
                                ProcessModelDirectives, RigidTransform,
                                Sphere, CoulombFriction)
        builder = RobotDiagramBuilder()
        parser = builder.parser()
        plant = builder.plant()
        scene_graph = builder.scene_graph()
        parser.package_map().Add('test_assets', str(ASSETS))
        parser.package_map().Add('kortex_description', str(ASSETS / 'kinova/kortex_description'))
        parser.package_map().Add('robotiq_arg85_description', str(ASSETS / 'robotiq'))
        ProcessModelDirectives(LoadModelDirectives(str(DIRECTIVE)), plant, parser)
        voxel_ids = []
        for index, centre in enumerate(self.voxels_xyz):
            voxel_ids.append(plant.RegisterCollisionGeometry(
                plant.world_body(), RigidTransform(centre), Sphere(self.voxel_radius),
                f'generic_motion_v1_voxel_{index}', CoulombFriction(.5, .5)))
        plant.Finalize()
        diagram = builder.Build()
        context = diagram.CreateDefaultContext()
        pc = plant.GetMyContextFromRoot(context)
        sc = scene_graph.GetMyContextFromRoot(context)
        inspector = scene_graph.get_query_output_port().Eval(sc).inspector()
        candidates = inspector.GetCollisionCandidates()
        expected = len(self.pairs) + len(self.robot_geometry_ids) * len(self.voxels_xyz)
        if len(candidates) != expected:
            raise RuntimeError('NTO registered-sphere collision filters differ from canonical model.')
        # Cardinality alone is insufficient: verify base pairs by geometry names
        # and all added sphere pairs against the exact moving-robot name set.
        names = lambda pair: frozenset(inspector.GetName(g) for g in pair)
        actual = {names(pair) for pair in candidates}
        expected_pairs = {frozenset(self.inspector.GetName(self._drake_ids[i]) for i in pair) for pair in self.pairs}
        robot_names = [self.inspector.GetName(self._drake_ids[i]) for i in self.robot_geometry_ids]
        expected_pairs.update(frozenset((name, inspector.GetName(voxel))) for name in robot_names for voxel in voxel_ids)
        if actual != expected_pairs:
            raise RuntimeError('NTO pair identities differ from canonical model.')
        return SimpleNamespace(diagram=diagram, context=context, plant=plant,
                               plant_context=pc, scene_graph=scene_graph,
                               scene_context=sc, inspector=inspector,
                               collision_pair_count=len(candidates))

    def _query(self):
        return self.scene_graph.get_query_output_port().Eval(self.scene_context)

    def set_voxels(self, voxels_xyz=None, voxel_radius=0.04):
        message = 'v1 freezes sphere radius at finite scalar 0.04 m; another radius requires a new benchmark version.'
        try:
            radius = np.asarray(voxel_radius, dtype=float)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError(message) from exc
        if radius.ndim != 0 or not np.isfinite(radius) or float(radius) != 0.04:
            raise ValueError(message)
        if voxels_xyz is None:
            xyz = np.empty((0, 3), dtype=float)
        else:
            try:
                xyz = np.asarray(voxels_xyz, dtype=float)
            except (TypeError, ValueError, OverflowError) as exc:
                raise ValueError('Voxel coordinates must be finite N x 3 arrays representable as float32.') from exc
            if xyz.shape == (0,):
                xyz = xyz.reshape(0, 3)
        if (xyz.ndim != 2 or xyz.shape[1] != 3 or not np.isfinite(xyz).all()
                or np.any(np.abs(xyz) > np.finfo(np.float32).max)):
            raise ValueError('Voxel coordinates must be finite N x 3 arrays representable as float32.')
        # Freeze actual float32 coordinates consumed by CSD. Save these values in
        # manifests, so the independent checker sees exactly the same centres.
        quantized = xyz.astype(np.float32)
        if not np.isfinite(quantized).all():
            raise ValueError('Voxel coordinates must remain finite after float32 conversion.')
        voxels = self.csd.Voxels(np.asarray(quantized.T, dtype=np.float32))
        # Commit the new state only after all validation and conversion succeed.
        self.voxels_xyz = quantized.astype(float)
        self.voxel_radius = float(radius)
        self.voxels = voxels
        return self

    def cuda(self, q):
        q = _configuration_array(q)
        if not len(q):
            return np.empty(0, dtype=bool)
        batches = []
        # Native voxel checker allocates O(states * robot_geometries * voxels).
        batch_size = max(1, min(512, 16000000 // max(1, len(self.robot_geometry_ids) * len(self.voxels_xyz))))
        for start in range(0, len(q), batch_size):
            states = np.asarray(q[start:start + batch_size].T, dtype=np.float32, order='F')
            if len(self.voxels_xyz):
                result = self.csd.CheckCollisionFreeVoxelsCuda(states, self.voxels, self.voxel_radius,
                                                             self.mplant, self.robot_geometry_ids)
            else:
                result = self.csd.CheckCollisionFreeCuda(states, self.mplant)
            batches.append(np.asarray(result, dtype=bool))
        return np.concatenate(batches)

    def _voxel_clearance(self, query):
        if not len(self.voxels_xyz):
            return float('inf')
        distance = float('inf')
        for geometry_id, kind, dimensions in self._robot_geometries:
            X = query.GetPoseInWorld(geometry_id)
            centres = (self.voxels_xyz - X.translation()) @ X.rotation().matrix()
            if kind == 'sphere':
                values = np.linalg.norm(centres, axis=1) - dimensions[0] - self.voxel_radius
            else:
                d = np.abs(centres) - dimensions / 2
                values = np.linalg.norm(np.maximum(d, 0), axis=1) + np.minimum(np.max(d, axis=1), 0) - self.voxel_radius
            distance = min(distance, float(np.min(values)))
        return distance

    def clearance(self, q):
        """Minimum signed distance over identical static pairs + online spheres."""
        q = _configuration_array(q)
        values = []
        for configuration in q:
            self.plant.SetPositions(self.plant_context, configuration)
            query = self._query()
            static = min((result.distance for result in query.ComputeSignedDistancePairwiseClosestPoints()), default=float('inf'))
            values.append(min(static, self._voxel_clearance(query)))
        return np.asarray(values)

    def independent(self, q):
        return self.clearance(q) > 0.0

    def tcp(self, q):
        q = _configuration_array(q)
        values = []
        for configuration in q:
            self.plant.SetPositions(self.plant_context, configuration)
            values.append(self.plant.EvalBodyPoseInWorld(self.plant_context, self._tcp_body).translation().copy())
        return np.asarray(values).reshape(-1, 3)

    def fixed_geometry_sphere_clearance(self, xyz, radius=0.04):
        """Distance of each sphere to fixed base/table/devices (input audit)."""
        xyz = np.asarray(xyz, float).reshape(-1, 3)
        result = np.full(len(xyz), np.inf)
        query = self._query()
        for index, geometry_id in enumerate(self._drake_ids):
            if index in self.robot_geometry_ids:
                continue
            entry = self.inventory['geometries'][index]
            X = query.GetPoseInWorld(geometry_id)
            local = (xyz - X.translation()) @ X.rotation().matrix()
            if entry['shape'] == 'sphere':
                values = np.linalg.norm(local, axis=1) - entry['dimensions_m'][0] - radius
            else:
                d = np.abs(local) - np.asarray(entry['dimensions_m']) / 2
                values = np.linalg.norm(np.maximum(d, 0), axis=1) + np.minimum(np.max(d, axis=1), 0) - radius
            result = np.minimum(result, values)
        return result

    def robot_bounds(self, q):
        """World AABB of moving collision primitives, for ROI auditing only."""
        q = _configuration_array(q)
        bounds = []
        for configuration in q:
            self.plant.SetPositions(self.plant_context, configuration)
            query = self._query()
            low, high = [], []
            for geometry_id, kind, dimensions in self._robot_geometries:
                X = query.GetPoseInWorld(geometry_id)
                half = dimensions[0] * np.ones(3) if kind == 'sphere' else np.abs(X.rotation().matrix()) @ (dimensions / 2)
                low.append(X.translation() - half)
                high.append(X.translation() + half)
            bounds.append([np.min(low, axis=0), np.max(high, axis=0)])
        return np.asarray(bounds)


def audit(out=None, seed=861730, random_count=512):
    """Real G0 audit. Persist raw states, boundary probes and all differences."""
    out = Path(out or HERE / 'reports/model_audit')
    out.mkdir(parents=True, exist_ok=True)
    model = Model()
    rng = np.random.Generator(np.random.PCG64(seed))
    states = rng.uniform(model.lower + 0.01, model.upper - 0.01, (random_count, 7))
    edge_endpoints = rng.uniform(model.lower + 0.01, model.upper - 0.01, (16, 2, 7))
    edge_states = np.vstack([np.linspace(a, b, 33) for a, b in edge_endpoints])
    report = dict(seed=seed, rng='numpy.random.PCG64', status='running', model=model.inventory)
    raw = dict(random_states=states, edge_endpoints=edge_endpoints, edge_states=edge_states)
    def check(name, q):
        tic = time.perf_counter()
        native = model.cuda(q)
        cuda_seconds = time.perf_counter() - tic
        tic = time.perf_counter()
        distances = model.clearance(q)
        drake_seconds = time.perf_counter() - tic
        independent = distances > 0
        mismatch = native != independent
        significant = mismatch & (np.abs(distances) > CONTACT_BAND_M)
        raw[name + '_cuda_free'] = native
        raw[name + '_independent_free'] = independent
        raw[name + '_clearance'] = distances
        return dict(count=len(q), cuda_free=int(native.sum()), independent_free=int(independent.sum()),
                    disagreements=int(mismatch.sum()), outside_contact_band=int(significant.sum()),
                    disagreement_indices=np.flatnonzero(mismatch).tolist(),
                    cuda_seconds=cuda_seconds, independent_seconds=drake_seconds)
    report['base_random'] = check('base_random', states)
    report['base_edges'] = check('base_edges', edge_states)
    voxel_centres = rng.uniform([-.25, -.8, .15], [.9, .8, .95], (30, 3))
    model.set_voxels(voxel_centres)
    raw['online_voxels_xyz'] = model.voxels_xyz.copy()
    report['voxel_random'] = check('voxel_random', states)
    report['voxel_edges'] = check('voxel_edges', edge_states)
    # Touch a moving robot face with an online 4 cm sphere at a genuinely
    # free state. This probes float32 classification near a known boundary.
    base_valid = raw['base_random_cuda_free'] & raw['base_random_independent_free'] & (raw['base_random_clearance'] > 1e-4)
    q0 = states[np.flatnonzero(base_valid)[0]]
    model.plant.SetPositions(model.plant_context, q0)
    query = model._query()
    geometry_id, kind, dimensions = model._robot_geometries[-1]
    X = query.GetPoseInWorld(geometry_id)
    boundaries = []
    for axis in range(3):
        for sign in (-1, 1):
            for offset in (-1e-3, -1e-4, -1e-5, -1e-6, -1e-7, 0., 1e-7, 1e-6, 1e-5, 1e-4, 1e-3):
                point = np.zeros(3)
                point[axis] = sign * (dimensions[axis] / 2 + 0.04 + offset)
                world = X.rotation().matrix() @ point + X.translation()
                model.set_voxels([world])
                native = bool(model.cuda(q0)[0])
                clearance = float(model.clearance(q0)[0])
                boundaries.append(dict(axis=axis, sign=sign, nominal_offset_m=offset,
                                       centre=model.voxels_xyz[0].tolist(), cuda_free=native,
                                       independent_free=clearance > 0, clearance_m=clearance))
    report['boundary'] = dict(count=len(boundaries),
                             disagreements=sum(b['cuda_free'] != b['independent_free'] for b in boundaries),
                             outside_contact_band=sum(b['cuda_free'] != b['independent_free'] and abs(b['clearance_m']) > CONTACT_BAND_M for b in boundaries),
                             tested_geometry=model.inspector.GetName(geometry_id), samples=boundaries)
    raw['boundary_configuration'] = q0
    # The actual upstream offline example uses the reduced CSD parser model.
    parser = model.csd.URDFParser()
    parser.register_package('test_assets', str(ASSETS))
    parser.parse_directives(str(ASSETS / 'directives/simple_kinova_sens_on_table.yml'))
    reduced = parser.build_plant()
    reduced_m = reduced.getMinimalPlant()
    model.set_voxels()
    reduced_free = np.asarray(model.csd.CheckCollisionFreeCuda(states.T.astype(np.float32), reduced_m), bool)
    raw['reduced_random_cuda_free'] = reduced_free
    full_free = raw['base_random_cuda_free']
    report['upstream_model_difference'] = dict(
        offline_directive='csdecomp/tests/test_assets/directives/simple_kinova_sens_on_table.yml',
        online_directive=str(DIRECTIVE.relative_to(ROOT)),
        reduced_geometry_count=int(reduced_m.num_scene_geometries),
        full_geometry_count=int(model.mplant.num_scene_geometries),
        reduced_robot_geometry_count=len(reduced.getRobotGeometryIds()),
        full_robot_geometry_count=len(model.robot_geometry_ids),
        reduced_pair_count=int(reduced_m.num_collision_pairs), full_pair_count=len(model.pairs),
        reduced_lower=np.asarray(reduced.getPositionLowerLimits()).tolist(),
        reduced_upper=np.asarray(reduced.getPositionUpperLimits()).tolist(),
        sampled_classification_disagreements=int(np.sum(reduced_free != full_free)),
        reduced_free_full_colliding=int(np.sum(reduced_free & ~full_free)),
        full_free_reduced_colliding=int(np.sum(~reduced_free & full_free)),
        remedy='Build all new roadmaps from Model.csd_plant; never load upstream reduced-model roadmaps.',
        sources={str(p.relative_to(ROOT)): sha256(p) for p in [ASSETS / 'directives/simple_kinova_sens_on_table.yml', ASSETS / 'reduced_kinova_sens.urdf', ASSETS / 'collapsed_robotiq.urdf']})
    # Compare the two actual upstream representations beyond random labels.
    # Fixed-chain collapse changes link indices but preserves collision shapes.
    full_tree = model.csd_plant.getKinematicTree()
    reduced_tree = reduced.getKinematicTree()
    full_geometry = model.mplant.scene_collision_geometries
    reduced_geometry = reduced_m.scene_collision_geometries
    world_errors, tcp_errors, drake_tcp_errors = [], [], []
    for q in states[:64]:
        full_transforms = np.asarray(full_tree.compute_link_frame_to_world_transforms(q))
        reduced_transforms = np.asarray(reduced_tree.compute_link_frame_to_world_transforms(q))
        full_world = np.asarray([full_transforms[g.link_index] @ g.X_L_B for g in full_geometry])
        reduced_world = np.asarray([reduced_transforms[g.link_index] @ g.X_L_B for g in reduced_geometry])
        world_errors.append(np.max(np.abs(full_world - reduced_world), axis=(1, 2)))
        full_tcp = full_transforms[full_tree.get_link_index(model.tcp_name)]
        reduced_tcp = reduced_transforms[reduced_tree.get_link_index(model.tcp_name)]
        tcp_errors.append(np.max(np.abs(full_tcp - reduced_tcp)))
        drake_tcp_errors.append(np.max(np.abs(full_tcp[:3, 3] - model.tcp([q])[0])))
    report['upstream_model_difference'].update(
        full_link_count=model.csd_plant.numLinks(), reduced_link_count=reduced.numLinks(),
        geometry_dimension_max_abs_difference=float(max(np.max(np.abs(a.dimensions - b.dimensions)) for a, b in zip(full_geometry, reduced_geometry))),
        collision_pairs_equal=bool(np.array_equal(model.pairs, np.asarray(reduced_m.collision_pairs_flat).reshape(-1, 2))),
        forward_kinematics_test_configurations=64,
        geometry_world_transform_max_abs_difference_per_geometry=np.max(world_errors, axis=0).tolist(),
        tcp_transform_max_abs_difference=float(max(tcp_errors)),
        full_csd_vs_drake_tcp_position_max_error_m=float(max(drake_tcp_errors)),
        conclusion='The reduced file collapses fixed chains. Both supplied models agree in primitive dimensions, limits and collision pairs, and in sampled poses to float32 precision. Different source files alone are not evidence of a physical-model mismatch; the new suite nevertheless freezes a single full-model construction for offline and online use.')
    valid_states = states[base_valid]
    tcp = model.tcp(valid_states)
    bounds = model.robot_bounds(valid_states)
    raw['base_free_tcp_xyz'] = tcp
    raw['base_free_robot_bounds'] = bounds
    report['workspace'] = dict(sampled_valid_configurations=len(valid_states),
                              tcp_min=tcp.min(axis=0).tolist(), tcp_max=tcp.max(axis=0).tolist(),
                              tcp_quantiles=np.quantile(tcp, [.05, .5, .95], axis=0).tolist(),
                              moving_collision_geometry_min=bounds[:, 0].min(axis=0).tolist(),
                              moving_collision_geometry_max=bounds[:, 1].max(axis=0).tolist(),
                              note='Workspace samples audit the obstacle ROI; they do not prove reachability of every point.')
    report['passed'] = all(report[name]['outside_contact_band'] == 0 for name in ('base_random', 'base_edges', 'voxel_random', 'voxel_edges', 'boundary'))
    report['status'] = 'passed_with_explicit_contact_band' if report['passed'] else 'failed'
    np.savez_compressed(out / 'raw_samples.npz', **raw)
    report['raw_samples_sha256'] = sha256(out / 'raw_samples.npz')
    report['model_source_sha256'] = sha256(__file__)
    (out / 'audit.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    (HERE / 'configs/robot.json').write_text(json.dumps(model.inventory, indent=2, allow_nan=False) + '\n')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', type=Path, default=HERE / 'reports/model_audit')
    parser.add_argument('--seed', type=int, default=861730)
    args = parser.parse_args()
    result = audit(args.out, args.seed)
    print(json.dumps({k: result[k] for k in ('status', 'passed', 'base_random', 'base_edges', 'voxel_random', 'voxel_edges', 'upstream_model_difference', 'workspace')}, indent=2))
    raise SystemExit(0 if result['passed'] else 1)
