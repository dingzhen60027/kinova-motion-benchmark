"""Signed clearance and a local derivative of the frozen collision geometry.

The minimum feature can change, so this is a piecewise derivative, not a claim
of global smoothness. Distances use exactly Model.clearance's static candidate
pairs and full-size online spheres. No collision primitive is approximated.
"""
from __future__ import annotations

import time
import numpy as np


def _point_jacobian(model, geometry_id, point_geometry):
    from pydrake.multibody.tree import JacobianWrtVariable
    inspector = model.inspector
    body = model.plant.GetBodyFromFrameId(inspector.GetFrameId(geometry_id))
    point_body = inspector.GetPoseInFrame(geometry_id) @ np.asarray(point_geometry)
    return model.plant.CalcJacobianTranslationalVelocity(
        model.plant_context, JacobianWrtVariable.kQDot, body.body_frame(),
        point_body, model.plant.world_frame(), model.plant.world_frame())


def clearance_gradient(model, configurations, deadline=float("inf"), finite_difference_step=1e-5):
    """Return identical signed minimum clearance and active-feature derivative."""
    configurations = np.asarray(configurations, float)
    if configurations.ndim != 2 or configurations.shape[1] != 7 or not np.isfinite(configurations).all():
        raise ValueError("Canonical gradient requires finite N x 7 configurations")
    distances = np.empty(len(configurations))
    gradients = np.empty_like(configurations)
    for index, q in enumerate(configurations):
        if time.perf_counter() >= deadline:
            raise TimeoutError("Canonical geometry derivative exhausted shared deadline")
        model.plant.SetPositions(model.plant_context, q)
        query = model._query()
        pairs = query.ComputeSignedDistancePairwiseClosestPoints()
        competing = [float(pair.distance) for pair in pairs]
        closest = min(pairs, key=lambda pair: pair.distance, default=None)
        best = float(closest.distance) if closest is not None else float("inf")
        feature = ("static", closest)
        if len(model.voxels_xyz):
            for geometry, kind, dimensions in model._robot_geometries:
                pose = query.GetPoseInWorld(geometry)
                rotation = pose.rotation().matrix()
                centres = (model.voxels_xyz-pose.translation()) @ rotation
                if kind == "sphere":
                    norm = np.linalg.norm(centres, axis=1)
                    values = norm-dimensions[0]-model.voxel_radius
                else:
                    delta = np.abs(centres)-dimensions/2
                    values = (np.linalg.norm(np.maximum(delta, 0), axis=1)
                              + np.minimum(np.max(delta, axis=1), 0)-model.voxel_radius)
                voxel = int(np.argmin(values))
                competing.extend(np.partition(values, min(1, len(values)-1))[:2].tolist())
                if values[voxel] < best:
                    best = float(values[voxel])
                    feature = ("voxel", geometry, kind, dimensions,
                               centres[voxel], rotation)
        nonsmooth = (len(competing) > 1 and np.partition(competing, 1)[1]-best < 1e-9)
        if feature[0] == "static":
            pair = feature[1]
            if pair is None:
                raise ValueError("Canonical model has no finite distance features")
            ja = _point_jacobian(model, pair.id_A, pair.p_ACa)
            jb = _point_jacobian(model, pair.id_B, pair.p_BCb)
            gradient = pair.nhat_BA_W @ (ja-jb)
        else:
            _, geometry, kind, dimensions, centre, rotation = feature
            if kind == "sphere":
                norm = np.linalg.norm(centre)
                nonsmooth |= norm <= 1e-15
                # At coincident centres the signed distance has no unique
                # derivative. A zero subgradient records that degeneracy.
                normal = centre/norm if norm > 1e-15 else np.zeros(3)
                point = np.zeros(3)
            else:
                half = dimensions/2
                point = np.clip(centre, -half, half)
                outside = centre-point
                norm = np.linalg.norm(outside)
                if norm > 1e-15:
                    normal = outside/norm
                else:
                    axis = int(np.argmax(np.abs(centre)-half))
                    faces = np.sort(np.abs(centre)-half)
                    nonsmooth |= faces[-1]-faces[-2] < 1e-9
                    normal = np.zeros(3)
                    normal[axis] = 1. if centre[axis] >= 0 else -1.
                    point[axis] = normal[axis]*half[axis]
            gradient = -(rotation @ normal) @ _point_jacobian(model, geometry, point)
        if nonsmooth:
            # At ties a selected-feature derivative depends on feature ordering.
            # Use the symmetric finite-difference policy explicitly, retaining
            # the exact original clearance rather than smoothing geometry.
            perturb = np.eye(7)*finite_difference_step
            sample = np.r_[q[None, :]+perturb, q[None, :]-perturb]
            values = model.clearance(sample)
            gradient = (values[:7]-values[7:])/(2*finite_difference_step)
        distances[index], gradients[index] = best, gradient
    if not np.isfinite(distances).all() or not np.isfinite(gradients).all():
        raise ValueError("Nonfinite canonical signed distance/derivative")
    return distances, gradients
