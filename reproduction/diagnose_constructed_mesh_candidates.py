"""Numerical original-triangle diagnostic for the three fixed link5/hand cases.

No installs, GPU, solver, model fitting or main-result edits. Run only after the
full timed grid stops. Includes synthetic controls, unsigned surface distance,
transverse/coplanar intersection checks and closed-mesh containment diagnostics.
All predicates are float64, not exact arithmetic or a formal mesh certificate.
"""
from __future__ import annotations

import argparse
from collections import defaultdict
import hashlib
from pathlib import Path
import sys
import time
import traceback

import numpy as np

from reproduction.common import ROOT, atomic, read
from reproduction.validate_constructed_geometry import (
    DEFAULT_OUT, SOURCE, GeometryOracle, bare_chain, description, prepare,
)

DISTANCE_TOL = 1e-8  # Only an unresolved band; never changes the benchmark flags.
AREA_TOL = 1e-16
PARAMETER_TOL = 1e-10
WINDING_TOL = 1e-6
LIMITATION = (
    "Custom float64 numerical diagnostic, not exact predicates or a formal certificate. "
    "AABB pruning, triangle distances, topology/self-intersection screens and winding "
    "containment tests are all recorded. Near contact, invalid topology or ambiguous "
    "winding remain unresolved. Meshes are not repaired, welded by tolerance, reoriented, "
    "inflated, decomposed or refitted. No benchmark collision flags are changed."
)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_stl(path):
    raw = Path(path).read_bytes()
    count = int.from_bytes(raw[80:84], "little")
    if len(raw) != 84 + 50 * count:
        raise ValueError("Expected binary STL: " + str(path))
    dtype = np.dtype([("normal", "<f4", 3), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])
    result = np.frombuffer(raw, dtype=dtype, offset=84)["vertices"].astype(float)
    if not np.all(np.isfinite(result)):
        raise ValueError("Nonfinite STL coordinates")
    return result


def closest_segment(point, first, second):
    delta = second - first
    length2 = float(delta @ delta)
    u = np.clip(float((point - first) @ delta) / length2, 0, 1) if length2 > 0 else 0.0
    return first + u * delta


def barycentric(point, tri):
    # Dominant-axis projected oriented areas avoid the cancellation of the
    # Gram determinant ||u||²||v||²-(u·v)² on slender triangles.
    normal = np.cross(tri[1] - tri[0], tri[2] - tri[0])
    axis = int(np.argmax(abs(normal)))
    projected = np.delete(tri, axis, axis=1)
    q = np.delete(point, axis)
    cross2 = lambda u, v: float(u[0] * v[1] - u[1] * v[0])
    denominator = cross2(projected[1] - projected[0], projected[2] - projected[0])
    if denominator == 0:
        return None
    return np.array([cross2(projected[1] - q, projected[2] - q),
                     cross2(projected[2] - q, projected[0] - q),
                     cross2(projected[0] - q, projected[1] - q)]) / denominator


def closest_triangle(point, tri):
    normal = np.cross(tri[1] - tri[0], tri[2] - tri[0])
    norm2 = float(normal @ normal)
    candidates = [closest_segment(point, tri[i], tri[(i + 1) % 3]) for i in range(3)]
    if norm2 > 0:
        projected = point - normal * float((point - tri[0]) @ normal) / norm2
        weights = barycentric(projected, tri)
        if weights is not None and np.all(weights >= 0):
            candidates.append(projected)
    return min(candidates, key=lambda other: float((point - other) @ (point - other)))


def closest_edges(a, b, c, d):
    # A convex quadratic on [0,1]^2 reaches its minimum at an interior
    # stationary point or one of four boundaries. Explicit boundary candidates
    # also cover parallel and zero-length segments without fragile division by
    # ||u||²||v||²-(u·v)². SVD rank handling remains floating-point numerical.
    candidates = [(a, closest_segment(a, c, d)), (b, closest_segment(b, c, d)),
                  (closest_segment(c, a, b), c), (closest_segment(d, a, b), d)]
    u, v = b - a, d - c
    parameters, _, _, _ = np.linalg.lstsq(np.column_stack((u, -v)), c - a, rcond=None)
    if np.all(parameters >= 0) and np.all(parameters <= 1):
        candidates.append((a + parameters[0] * u, c + parameters[1] * v))
    return min(candidates, key=lambda pair: float((pair[0] - pair[1]) @ (pair[0] - pair[1])))


def segment_face_witnesses(a, b):
    """Collect every edge/face witness, including endpoints and coplanar edges.

    Retain near-boundary observations for conservative self-screening; these
    observations are not automatically zero-distance intersection claims.
    """
    witnesses = []
    for edge_tri, face_tri, direction in ((a, b, "A_edge_B_face"), (b, a, "B_edge_A_face")):
        normal = np.cross(face_tri[1] - face_tri[0], face_tri[2] - face_tri[0])
        length = np.linalg.norm(normal)
        if length == 0:
            continue
        normal /= length
        signed = (edge_tri - face_tri[0]) @ normal
        for i in range(3):
            j = (i + 1) % 3
            if (signed[i] < 0 < signed[j]) or (signed[j] < 0 < signed[i]):
                t = float(signed[i] / (signed[i] - signed[j]))
                point = edge_tri[i] + t * (edge_tri[j] - edge_tri[i])
                weights = barycentric(point, face_tri)
                if weights is not None and np.all(weights >= -PARAMETER_TOL):
                    robust = (min(abs(signed[i]), abs(signed[j])) > DISTANCE_TOL
                              and min(t, 1 - t, float(weights.min())) > PARAMETER_TOL)
                    witnesses.append(dict(kind="transverse_intersection" if robust else "boundary_intersection_candidate",
                                          point=point, edge=i, direction=direction, edge_parameter=t,
                                          face_barycentric=weights, endpoint_plane_distances=[signed[i], signed[j]]))
            if abs(signed[i]) <= DISTANCE_TOL:
                projected = edge_tri[i] - signed[i] * normal
                weights = barycentric(projected, face_tri)
                if weights is not None and np.all(weights >= -PARAMETER_TOL):
                    witnesses.append(dict(kind="endpoint_face_proximity", point=edge_tri[i],
                                          projected_point=projected, plane_distance=float(signed[i]),
                                          edge=i, direction=direction, face_barycentric=weights))
            if max(abs(signed[i]), abs(signed[j])) <= DISTANCE_TOL:
                for k in range(3):
                    first, second = closest_edges(edge_tri[i], edge_tri[j], face_tri[k], face_tri[(k + 1) % 3])
                    gap = float(np.linalg.norm(first - second))
                    if gap <= DISTANCE_TOL:
                        witnesses.append(dict(kind="coplanar_edge_proximity", point=first,
                                              second_point=second, distance=gap, edge=i,
                                              target_edge=k, direction=direction))
    return witnesses


def transverse_witness(a, b):
    witnesses = segment_face_witnesses(a, b)
    return next((row for row in witnesses if row["kind"] == "transverse_intersection"),
                next((row for row in witnesses if row["kind"] == "boundary_intersection_candidate"), None))


def coplanar_overlap_area(a, b):
    normal = np.cross(a[1] - a[0], a[2] - a[0])
    length = np.linalg.norm(normal)
    if length == 0 or np.max(abs((b - a[0]) @ (normal / length))) > 1e-12:
        return 0.0
    axis = int(np.argmax(abs(normal)))
    first, second = np.delete(a, axis, axis=1), np.delete(b, axis, axis=1)
    cross2 = lambda u, v: float(u[0] * v[1] - u[1] * v[0])
    orientation = np.sign(cross2(second[1] - second[0], second[2] - second[0]))
    if orientation == 0:
        return 0.0
    polygon = list(first)
    for i in range(3):
        start, end = second[i], second[(i + 1) % 3]
        clipped = []
        if not polygon:
            break
        previous = polygon[-1]
        previous_side = orientation * cross2(end - start, previous - start)
        for current in polygon:
            current_side = orientation * cross2(end - start, current - start)
            if (previous_side >= 0) != (current_side >= 0):
                fraction = previous_side / (previous_side - current_side)
                clipped.append(previous + fraction * (current - previous))
            if current_side >= 0:
                clipped.append(current)
            previous, previous_side = current, current_side
        polygon = clipped
    if len(polygon) < 3:
        return 0.0
    projected_area = abs(sum(cross2(polygon[i], polygon[(i + 1) % len(polygon)]) for i in range(len(polygon)))) / 2
    return projected_area * length / abs(normal[axis])


def triangle_distance(a, b):
    witness = transverse_witness(a, b)
    if witness is not None:
        return dict(distance=0.0, **witness)
    area = coplanar_overlap_area(a, b)
    if area > AREA_TOL:
        return dict(distance=0.0, kind="coplanar_overlap_candidate", overlap_area=area)
    best = None
    for tri, other, reverse in ((a, b, False), (b, a, True)):
        for point in tri:
            closest = closest_triangle(point, other)
            distance = float(np.linalg.norm(point - closest))
            if best is None or distance < best["distance"]:
                best = dict(distance=distance, kind="vertex_face_or_edge_distance",
                            point_A=closest if reverse else point, point_B=point if reverse else closest)
    for i in range(3):
        for j in range(3):
            p, q = closest_edges(a[i], a[(i + 1) % 3], b[j], b[(j + 1) % 3])
            distance = float(np.linalg.norm(p - q))
            if distance < best["distance"]:
                best = dict(distance=distance, kind="edge_edge_distance", point_A=p, point_B=q)
    return best


def box_lower_bounds(a, b):
    separation = np.maximum(a.min(axis=1)[:, None] - b.max(axis=1)[None],
                            b.min(axis=1)[None] - a.max(axis=1)[:, None])
    return np.linalg.norm(np.maximum(0, separation), axis=2)


def surface_distance(a, b):
    bounds = box_lower_bounds(a, b)
    best, tested = None, 0
    for flat in np.argsort(bounds, axis=None):
        i, j = np.unravel_index(flat, bounds.shape)
        if best is not None and bounds[i, j] > best["distance"] + DISTANCE_TOL:
            break
        result = triangle_distance(a[i], b[j])
        tested += 1
        if best is None or result["distance"] < best["distance"] or result["kind"] == "transverse_intersection":
            best = dict(result, triangle_A=int(i), triangle_B=int(j))
        if result["kind"] == "transverse_intersection":
            break  # Zero distance witnessed; exhaustive intersection enumeration not needed.
    return dict(best, total_triangle_pairs=int(bounds.size), tested_triangle_pairs=tested,
                search="AABB distance lower-bound pruning; early exit only after proper transverse witness")


def topology(triangles):
    vertices, inverse = np.unique(triangles.reshape(-1, 3), axis=0, return_inverse=True)
    faces = inverse.reshape(-1, 3)
    edges = defaultdict(list)
    vertex_links = defaultdict(list)
    for i, face in enumerate(faces):
        for k in range(3):
            a, b = int(face[k]), int(face[(k + 1) % 3])
            edges[tuple(sorted((a, b)))].append((i, 1 if a < b else -1))
            vertex_links[int(face[k])].append(tuple(int(x) for x in (face[(k + 1) % 3], face[(k + 2) % 3])))
    adjacency = [set() for _ in faces]
    for incident in edges.values():
        for first, _ in incident:
            adjacency[first].update(second for second, _ in incident if second != first)
    components, remaining = [], set(range(len(faces)))
    while remaining:
        stack, component = [min(remaining)], []
        while stack:
            i = stack.pop()
            if i not in remaining:
                continue
            remaining.remove(i); component.append(i); stack.extend(adjacency[i] & remaining)
        components.append(sorted(component))
    bad_vertex_links = []
    for vertex, link_edges in vertex_links.items():
        graph = defaultdict(set)
        for a, b in link_edges:
            graph[a].add(b); graph[b].add(a)
        reached, stack = set(), [next(iter(graph))]
        while stack:
            current = stack.pop()
            if current not in reached:
                reached.add(current); stack.extend(graph[current] - reached)
        if len(reached) != len(graph) or any(len(neighbors) != 2 for neighbors in graph.values()):
            bad_vertex_links.append(vertex)
    double_area = np.linalg.norm(np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0]), axis=1)
    degenerate = np.flatnonzero(double_area <= 2 * AREA_TOL).tolist()
    edge_scale2 = np.max(np.sum((triangles - np.roll(triangles, 1, axis=1)) ** 2, axis=2), axis=1)
    shape_quality = double_area / np.maximum(edge_scale2, 1e-300)
    ill_conditioned = np.flatnonzero(shape_quality <= 1e-12).tolist()
    bad_edges = [dict(vertices=edge, incident_faces=[i for i, _ in incident]) for edge, incident in edges.items() if len(incident) != 2]
    bad_orientation = [edge for edge, incident in edges.items() if len(incident) == 2 and sum(sign for _, sign in incident) != 0]
    duplicate_faces = len(faces) - len(np.unique(np.sort(faces, axis=1), axis=0))
    volumes = [float(np.einsum('ij,ij->i', triangles[c, 0], np.cross(triangles[c, 1], triangles[c, 2])).sum() / 6) for c in components]
    valid = not (degenerate or ill_conditioned or bad_edges or bad_orientation or duplicate_faces or bad_vertex_links) and all(abs(v) > 1e-12 for v in volumes)
    return dict(triangles=len(faces), exact_unique_vertices=len(vertices), degenerate_faces=degenerate,
                ill_conditioned_faces=ill_conditioned, minimum_area_to_squared_edge_ratio=float(shape_quality.min()),
                nonmanifold_or_boundary_edges=bad_edges, inconsistent_oriented_edges=bad_orientation,
                nonmanifold_vertex_links=bad_vertex_links, duplicate_faces=duplicate_faces,
                connected_components=components, signed_component_volumes=volumes,
                closed_oriented_manifold_checks_pass=bool(valid)), faces


def self_intersection_screen(triangles, faces):
    bounds = box_lower_bounds(triangles, triangles)
    issues, tested = [], 0
    for i in range(len(triangles)):
        for j in range(i + 1, len(triangles)):
            if bounds[i, j] > DISTANCE_TOL:
                continue
            shared = sorted(set(faces[i]) & set(faces[j]))
            result = triangle_distance(triangles[i], triangles[j])
            witnesses = segment_face_witnesses(triangles[i], triangles[j])
            tested += 1
            outside_shared = []
            if shared:
                shared_points = [triangles[i, int(np.flatnonzero(faces[i] == index)[0])] for index in shared]
                for witness in witnesses:
                    points = [witness[key] for key in ("point", "projected_point", "second_point") if key in witness]
                    def gap(point):
                        if len(shared_points) == 1:
                            return float(np.linalg.norm(point - shared_points[0]))
                        if len(shared_points) == 2:
                            return float(np.linalg.norm(point - closest_segment(point, *shared_points)))
                        return float(np.linalg.norm(point - closest_triangle(point, np.array(shared_points))))
                    distance = max(gap(point) for point in points)
                    # Compatibility with the known shared simplex is much
                    # tighter than the 1e-8 unresolved contact band. Near the
                    # simplex but beyond roundoff is retained as unresolved.
                    if distance > 1e-12:
                        outside_shared.append(dict(witness, distance_to_shared_simplex=distance))
            if (result["kind"] in ("transverse_intersection", "coplanar_overlap_candidate")
                    or (not shared and result["distance"] <= DISTANCE_TOL)
                    or outside_shared):
                issues.append(dict(triangle_A=i, triangle_B=j, shared_vertex_indices=shared,
                                   outside_shared_simplex_witnesses=outside_shared,
                                   all_boundary_witnesses=witnesses, **result))
    return dict(passed=not issues, tested_pairs=tested, issues=issues,
                limitation="All edge/face crossing, endpoint and coplanar-edge witnesses are checked. Shared-simplex-compatible contact alone is allowed; any other or ambiguous contact is unresolved. Numerical screen, not exact self-intersection certification.")


def winding(point, triangles):
    a, b, c = (triangles[:, i] - point for i in range(3))
    la, lb, lc = (np.linalg.norm(x, axis=1) for x in (a, b, c))
    if min(la.min(), lb.min(), lc.min()) <= DISTANCE_TOL:
        return dict(status="unresolved_near_vertex", winding_number=None)
    numerator = np.einsum('ij,ij->i', a, np.cross(b, c))
    denominator = la * lb * lc + np.einsum('ij,ij->i', a, b) * lc + np.einsum('ij,ij->i', b, c) * la + np.einsum('ij,ij->i', c, a) * lb
    angle = 2 * np.arctan2(numerator, denominator)
    value = float(angle.sum() / (4 * np.pi))
    if not np.isfinite(value):
        return dict(status="unresolved_nonfinite_winding", winding_number=None)
    nearest = int(round(value))
    status = "outside" if nearest == 0 else "inside" if abs(nearest) == 1 else "unresolved_multiple_winding"
    if abs(value - nearest) > WINDING_TOL:
        status = "unresolved_noninteger_winding"
    return dict(status=status, winding_number=value, nearest_integer=nearest,
                integer_residual=abs(value - nearest), minimum_vertex_distance=float(min(la.min(), lb.min(), lc.min())))


def containment(a, b, topology_a, topology_b):
    component_counts = [len(topo["connected_components"]) for topo in (topology_a, topology_b)]
    if component_counts != [1, 1]:
        return [dict(status="unresolved_component_solid_semantics", component_counts=component_counts,
                     reason="Multiple independently oriented closed components may cancel total winding; union/cavity semantics are not inferred")]
    rows = []
    for first, second, label, source_topology in ((a, b, "A_in_B", topology_a), (b, a, "B_in_A", topology_b)):
        for i, component in enumerate(source_topology["connected_components"]):
            # A component's first face centroid, before any test result is known.
            triangle = component[0]
            point = first[triangle].mean(axis=0)
            rows.append(dict(direction=label, component=i, representative_triangle=triangle,
                             representative_point=point, **winding(point, second)))
    return rows


def tetrahedron(scale=1., offset=None):
    vertices = np.array([[0., 0., 0.], [scale, 0., 0.], [0., scale, 0.], [0., 0., scale]])
    if offset is not None:
        vertices += offset
    return vertices[np.array([[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]])]


def self_tests():
    tests = []
    a = np.array([[0., 0., 0.], [1., 0., 0.], [0., 1., 0.]])
    b = a + [0, 0, .5]
    distance = triangle_distance(a, b)["distance"]
    tests.append(dict(name="parallel_triangle_distance", expected=.5, actual=distance, passed=abs(distance - .5) < 1e-12))
    for name, endpoints, expected in (
        ("skew_interior_edge_distance", ([-1., 0., 0.], [1., 0., 0.], [0., -1., .25], [0., 1., .25]), .25),
        ("parallel_edge_distance", ([0., 0., 0.], [1., 0., 0.], [.2, .5, 0.], [.8, .5, 0.]), .5),
        ("zero_length_edge_distance", ([0., 0., 0.], [0., 0., 0.], [1., -1., 0.], [1., 1., 0.]), 1.),
    ):
        p, q = closest_edges(*(np.array(point) for point in endpoints))
        measured = float(np.linalg.norm(p - q))
        tests.append(dict(name=name, expected=expected, actual=measured, passed=abs(measured - expected) < 1e-12))
    cross = np.array([[.25, .25, -1.], [.25, .25, 1.], [.75, .25, .5]])
    crossing = triangle_distance(a, cross)
    tests.append(dict(name="proper_segment_face_crossing", actual=crossing, passed=crossing["kind"] == "transverse_intersection"))
    coplanar = triangle_distance(a, a * .5 + [.1, .1, 0])
    tests.append(dict(name="coplanar_positive_area_overlap", actual=coplanar, passed=coplanar["kind"] == "coplanar_overlap_candidate"))
    outer, inner = tetrahedron(2.), tetrahedron(.1, [.1, .1, .1])
    outer_topology, outer_faces = topology(outer)
    inner_topology, _ = topology(inner)
    checks = containment(inner, outer, inner_topology, outer_topology)
    tests.append(dict(name="closed_tetrahedron_topology_and_self_screen", passed=outer_topology["closed_oriented_manifold_checks_pass"] and self_intersection_screen(outer, outer_faces)["passed"]))
    tests.append(dict(name="positive_surface_distance_with_containment", surface=surface_distance(inner, outer), checks=checks, passed=surface_distance(inner, outer)["distance"] > DISTANCE_TOL and checks[0]["status"] == "inside" and checks[1]["status"] == "outside"))
    outside = winding(np.array([3., 3., 3.]), outer)
    tests.append(dict(name="outside_winding", actual=outside, passed=outside["status"] == "outside"))
    open_topology, _ = topology(outer[:-1])
    tests.append(dict(name="open_mesh_rejected", passed=not open_topology["closed_oriented_manifold_checks_pass"]))
    nested_reverse = np.concatenate((outer, tetrahedron(1., [.1, .1, .1])[:, ::-1]))
    multiple_topology, _ = topology(nested_reverse)
    multiple_check = containment(inner, nested_reverse, inner_topology, multiple_topology)
    tests.append(dict(name="oppositely_oriented_components_remain_unresolved", actual=multiple_check,
                      passed=multiple_check[0]["status"] == "unresolved_component_solid_semantics"))
    shared_vertex = np.array([a, [[0., 0., 0.], [.5, 0., 1.], [.5, 0., -1.]]])
    _, shared_faces = topology(shared_vertex)
    shared_screen = self_intersection_screen(shared_vertex, shared_faces)
    tests.append(dict(name="shared_vertex_does_not_hide_distant_boundary_intersection", actual=shared_screen,
                      passed=not shared_screen["passed"] and any(issue["outside_shared_simplex_witnesses"] for issue in shared_screen["issues"])))
    legitimate_adjacent = np.array([a, [[0., 0., 0.], [0., 1., 0.], [0., 0., 1.]]])
    _, adjacent_faces = topology(legitimate_adjacent)
    adjacent_screen = self_intersection_screen(legitimate_adjacent, adjacent_faces)
    tests.append(dict(name="legitimate_shared_edge_not_a_self_intersection", actual=adjacent_screen,
                      passed=adjacent_screen["passed"]))
    slender = np.array([[0., 0., 0.], [1., 0., 0.], [0., 1e-8, 0.]])
    slender_crossing = np.array([[.2, 2e-9, -1.], [.2, 2e-9, 1.], [.6, 2e-9, .5]])
    slender_result = triangle_distance(slender, slender_crossing)
    tests.append(dict(name="slender_triangle_interior_crossing", actual=slender_result,
                      passed=slender_result["kind"] == "transverse_intersection"))
    ill_conditioned_mesh = np.array([[[0., 0., 0.], [1., 0., 0.], [0., 1e-14, 0.]]])
    ill_topology, _ = topology(ill_conditioned_mesh)
    tests.append(dict(name="ill_conditioned_triangle_is_not_silently_reliable", actual=ill_topology,
                      passed=ill_topology["ill_conditioned_faces"] == [0]))
    return dict(passed=all(test["passed"] for test in tests), tests=tests)


def run(args, output):
    initial_snapshot = {str(Path(__file__).resolve()): sha(Path(__file__)),
                        str(Path(__file__).with_name("validate_constructed_geometry.py").resolve()): sha(Path(__file__).with_name("validate_constructed_geometry.py"))}
    atomic(output / "initial_implementation_inventory.json", initial_snapshot)
    controls = self_tests()
    atomic(output / "synthetic_controls.json", controls)
    if any(sha(path) != expected for path, expected in initial_snapshot.items()):
        raise ValueError("Implementation changed during synthetic controls")
    if not controls["passed"]:
        raise ValueError("Synthetic geometry controls failed; real diagnosis not performed")
    if args.self_test_only:
        return dict(status="synthetic_controls_only", real_candidate_evaluation=False)
    if args.validation_dir is None:
        raise ValueError("--validation-dir is required for actual candidates")
    validation = args.validation_dir.resolve()
    identity = read(validation / "input_identity.json")
    asset_path = args.asset or Path(identity["asset_path"])
    if sha(asset_path) != identity["asset_sha256"]:
        raise ValueError("Expected the exact saved validation asset")
    prereg = prepare(args.oracle_inputs)
    if sha(args.oracle_inputs / "preregistration.json") != identity["preregistration_sha256"]:
        raise ValueError("Original oracle registration changed")
    desc = description(args.oracle_inputs)
    asset = read(asset_path)
    if asset["kinematics"] != desc["kinematics"] or asset["limits"] != desc["limits"]:
        raise ValueError("Saved model differs from official preparation kinematics")
    candidates = {}
    row_sources = []
    for condition in ("static_table", "online_20_spheres"):
        path = validation / (condition + "_rows.json")
        row_sources.append(dict(path=str(path), sha256=sha(path)))
        count = 0
        for row in read(path):
            if row["classification"] != "reference_false_negative_candidate":
                continue
            if any({pair["first_link"], pair["second_link"]} != {"fr3_link5", "fr3_hand"} for pair in row["reference_collisions"]):
                raise ValueError("Unexpected body pair in fixed candidate set")
            count += 1
            entry = candidates.setdefault(row["sample_id"], dict(configuration=row["configuration"], occurrences=[]))
            if entry["configuration"] != row["configuration"]:
                raise ValueError("Candidate configuration differs across conditions")
            entry["occurrences"].append(dict(condition=condition, reference_collisions=row["reference_collisions"], sphere_minimum_distance=row["sphere_minimum_distance"]))
        if count != (3 if condition == "static_table" else 2):
            raise ValueError("Expected exactly the original 3 static / 2 online candidates")
    if set(candidates) != {"joint_0608", "joint_0673", "workspace_left_38"}:
        raise ValueError("Fixed candidate IDs changed")
    paths = [SOURCE / "meshes/robots/fr3/collision/link5.stl", SOURCE / "meshes/robot_ee/franka_hand_white/collision/hand.stl"]
    snapshot_paths = {Path(__file__), Path(__file__).with_name("validate_constructed_geometry.py"), asset_path,
                      validation / "input_identity.json", args.oracle_inputs / "preregistration.json",
                      args.oracle_inputs / "official_description.json", *paths,
                      *(Path(row["path"]) for row in row_sources),
                      *(Path(row["path"]) for row in prereg["sources"])}
    input_snapshot = dict(initial_snapshot)
    for path in sorted(snapshot_paths):
        key, current = str(path.resolve()), sha(path)
        if key in input_snapshot and input_snapshot[key] != current:
            raise ValueError("Implementation changed before real geometry calculations")
        input_snapshot[key] = current
    atomic(output / "frozen_input_inventory.json", input_snapshot)
    atomic(output / "source_inputs_before_geometry.json", dict(asset_path=str(asset_path), asset_sha256=sha(asset_path),
           row_sources=row_sources, official_sources=prereg["sources"],
           mesh_files=[dict(path=str(path), sha256=sha(path)) for path in paths], candidates=candidates))
    meshes = [read_stl(path) for path in paths]
    topologies, screens = [], []
    for mesh in meshes:
        topo, faces = topology(mesh)
        topologies.append(topo); screens.append(self_intersection_screen(mesh, faces))
    atomic(output / "mesh_topology_and_self_screens.json", dict(link5=topologies[0], hand=topologies[1], self_intersection_screens=screens))
    atomic(output / "input_identity.json", dict(command=sys.argv, script_sha256=sha(Path(__file__)), geometry_helper_sha256=sha(Path(__file__).with_name("validate_constructed_geometry.py")), asset_sha256=sha(asset_path), validation_identity_sha256=sha(validation / "input_identity.json"), preregistration_sha256=sha(args.oracle_inputs / "preregistration.json"), row_sources=row_sources, official_sources=prereg["sources"], mesh_files=[dict(path=str(path), sha256=sha(path)) for path in paths], numeric_parameters=dict(distance_unresolved_band_m=DISTANCE_TOL, area_tolerance_m2=AREA_TOL, barycentric_tolerance=PARAMETER_TOL, winding_tolerance=WINDING_TOL), limitation=LIMITATION))
    plant = GeometryOracle(desc, args.oracle_inputs, bare_chain(desc))
    records = []
    topology_valid = all(topo["closed_oriented_manifold_checks_pass"] for topo in topologies) and all(screen["passed"] for screen in screens)
    for sample_id, entry in candidates.items():
        dest = output / sample_id
        dest.mkdir()
        try:
            plant.plant.SetPositions(plant.pc, entry["configuration"])
            transforms = [plant.plant.CalcRelativeTransform(plant.pc, plant.plant.world_frame(), plant.plant.GetBodyByName(name, plant.instance).body_frame()).GetAsMatrix4() for name in ("fr3_link5", "fr3_hand")]
            posed = [mesh @ transform[:3, :3].T + transform[:3, 3] for mesh, transform in zip(meshes, transforms)]
            distance = surface_distance(*posed)
            contained = containment(*posed, *topologies) if distance["distance"] > DISTANCE_TOL else []
            if distance["kind"] == "transverse_intersection":
                conclusion = "numerical_transverse_surface_intersection"
            elif distance["distance"] <= DISTANCE_TOL:
                conclusion = "unresolved_contact_or_near_contact"
            elif any(row["status"] == "unresolved_component_solid_semantics" for row in contained):
                conclusion = "unresolved_component_solid_semantics"
            elif not topology_valid:
                conclusion = "unresolved_containment_due_to_topology_or_self_screen"
            elif any(row["status"].startswith("unresolved") for row in contained):
                conclusion = "unresolved_winding_classification"
            elif any(row["status"] == "inside" for row in contained):
                conclusion = "numerical_closed_mesh_containment"
            else:
                conclusion = "numerically_separated_original_triangle_meshes"
            record = dict(sample_id=sample_id, **entry, status="evaluated", conclusion=conclusion,
                          surface_distance=distance, containment=contained, mesh_topology_and_self_screen_pass=topology_valid,
                          link5_world_transform=transforms[0], hand_world_transform=transforms[1], limitation=LIMITATION)
            np.savez_compressed(dest / "posed_original_meshes.npz", link5=posed[0], hand=posed[1], configuration=np.array(entry["configuration"]), link5_world_transform=transforms[0], hand_world_transform=transforms[1])
        except Exception as error:
            record = dict(sample_id=sample_id, **entry, status="unresolved_exception", exception=repr(error), traceback=traceback.format_exc())
        atomic(dest / "result.json", record)
        records.append(record)
    drift = [path for path, expected in input_snapshot.items() if not Path(path).exists() or sha(path) != expected]
    atomic(output / "input_reverification.json", dict(passed=not drift, changed_or_missing=drift))
    if drift:
        raise ValueError("Inputs changed during mesh diagnosis; saved per-case conclusions are unverified: " + repr(drift))
    unresolved = sum(row["status"] != "evaluated" or row.get("conclusion", "").startswith("unresolved") for row in records)
    return dict(status="completed_with_unresolved" if unresolved else "completed_numerical_diagnostic", unresolved_configurations=unresolved, unique_configurations=len(records), original_candidate_occurrences=sum(len(row["occurrences"]) for row in records), results=records, limitation=LIMITATION, benchmark_flags_modified=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--validation-dir", type=Path)
    parser.add_argument("--oracle-inputs", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--asset", type=Path, help="Original asset copy, must match saved hash")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--self-test-only", action="store_true")
    args = parser.parse_args()
    output = args.output or ROOT / "reproduction/reports/paper_constructed" / ("mesh_candidate_diagnostic_" + time.strftime("%Y%m%dT%H%M%SZ", time.gmtime()))
    output.mkdir(parents=True, exist_ok=False)
    atomic(output / "execution.json", dict(command=sys.argv, script_sha256=sha(Path(__file__)),
           numpy_version=np.__version__, started_utc=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
           limitation=LIMITATION))
    try:
        result = run(args, output)
        atomic(output / "summary.json", result)
        print(output / "summary.json", flush=True)
    except BaseException as error:
        atomic(output / "failure.json", dict(command=sys.argv, exception=repr(error), traceback=traceback.format_exc(), limitation=LIMITATION))
        raise


if __name__ == "__main__":
    main()
