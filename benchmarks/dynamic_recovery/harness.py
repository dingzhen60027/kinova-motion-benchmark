"""Closed-loop episode execution with measured planning latency.

Time semantics. The simulation clock advances only by physics steps. A method
is called with an observation taken at simulation time ``t``. Its wall-clock
computation time ``L`` (measured by the harness around the inter-process call)
is charged to the simulation: the robot keeps tracking the current reference
and obstacles keep moving while the call is outstanding, and the response can
take effect only from ``t + L``. A submitted trajectory names its own start time
``t_s``; it is accepted only if ``t_s >= t + L``, it matches the commanded
reference at ``t_s`` in position and velocity, ends at rest and respects joint,
velocity and acceleration limits. The physical robot state is assigned once,
at episode start, and never reset.

Safety. Collisions are penetrations beyond the canonical 2 um contact band
reported by MuJoCo contacts during the rollout or by the canonical Drake
checker over every recorded state afterwards. An external supervisor applies a
protective stop when speed and separation monitoring against moving obstacles
is violated; each stop is an intervention.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import importlib
import json
import multiprocessing
import os
from pathlib import Path
import platform
import time
import traceback

import numpy as np

from . import BENCHMARK
from .trajectory import Plan, Segment, braking, check_submission, serializable
from .world import GeometryChecker, World, build_simulation, VOXEL_RADIUS_M

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def load_config():
    return json.loads((HERE / "config.json").read_text())


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def source_hashes():
    files = sorted(HERE.glob("*.py")) + [HERE / "config.json", ROOT / "configs/mujoco.json"]
    return {str(p.relative_to(ROOT)): sha(p) for p in files}


def clean(value):
    if isinstance(value, np.ndarray):
        return clean(value.tolist())
    if isinstance(value, np.generic):
        return clean(value.item())
    if isinstance(value, float) and not np.isfinite(value):
        return None
    if isinstance(value, dict):
        return {str(k): clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(v) for v in value]
    return value


def write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(clean(value), indent=2, allow_nan=False) + "\n")
    os.replace(temporary, path)


def motion_limits(cfg, model):
    robot, protocol = cfg["robot"], cfg["protocol"]
    return dict(lower=np.asarray(model.lower, float), upper=np.asarray(model.upper, float),
                velocity=np.full(7, robot["velocity_limit_rad_s"]),
                acceleration=np.full(7, robot["acceleration_limit_rad_s2"]),
                tolerance=protocol["splice_tolerance"],
                maximum_duration_s=protocol["maximum_trajectory_duration_s"],
                maximum_pieces=protocol["maximum_trajectory_pieces"])


def public_problem(task, world, cfg, timestep):
    """Everything a method may know before the episode: no obstacle motion, no witness, no labels."""
    public = task["public"]
    return dict(benchmark=BENCHMARK, task_id=task["task_id"],
                robot=dict(task["public"]["robot"], velocity_limit_rad_s=cfg["robot"]["velocity_limit_rad_s"],
                           acceleration_limit_rad_s2=cfg["robot"]["acceleration_limit_rad_s2"],
                           tool_offset_m=cfg["robot"]["tool_offset_m"]),
                scene=dict(static_primitives=public["scene"]["static_primitives"],
                           static_voxels_xyz=world.static_xyz.copy(), cover_radius_m=VOXEL_RADIUS_M),
                goal=public["goal"], initial_plan=public["initial_plan"],
                nominal_waypoints=public["nominal"]["waypoints"],
                protocol=dict(cfg["protocol"], supervisor=cfg["supervisor"], timestep_s=timestep))


# ----------------------------------------------------------------------------- supervisor

def protective_separation(proximity, obstacle_velocity, robot_velocity, cfg, acceleration):
    """ISO/TS 15066-style protective separation distance (simplified)."""
    normal = proximity["normal"]
    approach_obstacle = max(0.0, -float(np.dot(obstacle_velocity, normal)))
    approach_robot = max(0.0, float(np.dot(proximity["point_velocity"], normal)))
    stopping = float(np.max(np.abs(robot_velocity) / acceleration))
    reaction = cfg["reaction_time_s"]
    required = (approach_obstacle * (reaction + stopping) + approach_robot * reaction
                + 0.5 * approach_robot * stopping + cfg["intrusion_distance_m"])
    return required, approach_obstacle, approach_robot, stopping


class Supervisor:
    """External protective stop on speed and separation monitoring violations."""

    def __init__(self, cfg, acceleration, checker):
        self.cfg, self.acceleration, self.checker = cfg, acceleration, checker
        self.stop_until = -np.inf
        self.interventions = []

    def active(self, t):
        return t < self.stop_until

    def measure(self, t, q, v, world):
        xyz = world.dynamic_xyz(t)
        if not len(xyz):
            return None
        proximity = self.checker.proximity(q, v, xyz)
        owner = int(world.owner[proximity["sphere"]])
        obstacle_velocity = world.obstacles[owner].motion.at(t)[1]
        required, vo, vr, ts = protective_separation(proximity, obstacle_velocity, v, self.cfg, self.acceleration)
        return dict(distance=proximity["distance"], required=required, obstacle_approach=vo,
                    robot_approach=vr, stopping_time=ts, obstacle=world.obstacles[owner].id,
                    geometry=self.checker.names[proximity["geometry"]])

    def check(self, t, q, v, plan, world):
        """Return (measurement, new plan or None)."""
        measurement = self.measure(t, q, v, world)
        if (measurement is None or self.active(t) or np.max(np.abs(v)) <= self.cfg["moving_threshold_rad_s"]
                or measurement["distance"] >= measurement["required"]):
            return measurement, None
        start = t + self.cfg["reaction_time_s"]
        qs, vs, _ = plan.at(start)
        segment = Segment(start, braking(qs, vs, self.acceleration))
        self.stop_until = segment.end
        self.interventions.append(dict(type="intervention", time_s=t, brake_start_s=start,
                                       stop_until_s=segment.end, **clean(measurement)))
        return measurement, plan.spliced(segment)


# ----------------------------------------------------------------------------- planners

def _serve(connection, target, options, workdir):
    """Worker process loop: the evaluated method sees only setup and observation messages."""
    os.chdir(workdir)
    try:
        module, name = target.split(":")
        policy = getattr(importlib.import_module(module), name)(**options)
    except Exception:
        connection.send(("error", traceback.format_exc()))
        return
    connection.send(("ok", None))
    while True:
        try:
            kind, payload = connection.recv()
        except EOFError:
            return
        if kind == "close":
            return
        try:
            result = policy.setup(payload) if kind == "setup" else policy.step(payload)
            connection.send(("ok", result))
        except Exception:
            connection.send(("error", traceback.format_exc()))


class WorkerPlanner:
    """Evaluated method in an isolated process; latency includes message transfer."""

    def __init__(self, target, options=None, workdir=None):
        context = multiprocessing.get_context("spawn")
        self.connection, child = context.Pipe()
        workdir = Path(workdir or ".").resolve()
        workdir.mkdir(parents=True, exist_ok=True)
        self.process = context.Process(target=_serve, args=(child, target, dict(options or {}), str(workdir)),
                                       daemon=True)
        self.process.start()
        child.close()
        self.alive = True
        try:
            status, value = self.connection.recv()
        except EOFError:
            status, value = "error", f"worker exited during start-up (exit code {self.process.exitcode})"
        if status != "ok":
            self.close()
            raise RuntimeError("Method construction failed:\n" + str(value))

    def _call(self, kind, payload, timeout):
        if not self.alive:
            return None, 0.0, "planner_unavailable"
        start = time.perf_counter()
        try:
            self.connection.send((kind, payload))
            if not self.connection.poll(timeout):
                latency = time.perf_counter() - start
                self.close(force=True)
                return None, latency, "timeout"
            status, value = self.connection.recv()
        except (EOFError, BrokenPipeError, OSError):
            self.alive = False
            return None, time.perf_counter() - start, "planner_unavailable"
        latency = time.perf_counter() - start
        return (None, latency, value) if status == "error" else (value, latency, None)

    def setup(self, problem, timeout):
        return self._call("setup", problem, timeout)

    def step(self, observation, timeout):
        return self._call("step", observation, timeout)

    def close(self, force=False):
        if self.alive and not force:
            try:
                self.connection.send(("close", None))
            except (BrokenPipeError, OSError):
                pass
        self.alive = False
        self.process.join(timeout=0 if force else 5)
        if self.process.is_alive():
            self.process.terminate()
            self.process.join(timeout=5)


class InProcessPlanner:
    """Policy executed in the harness process; reserved for privileged witness validation."""

    def __init__(self, policy):
        self.policy = policy

    def setup(self, problem, timeout):
        start = time.perf_counter()
        return self.policy.setup(problem), time.perf_counter() - start, None

    def step(self, observation, timeout):
        start = time.perf_counter()
        response = self.policy.step(observation)
        return response, time.perf_counter() - start, None

    def close(self):
        pass


class RecordedPlanner:
    """Replays recorded responses and latencies; any observation-time mismatch is an error."""

    def __init__(self, calls):
        self.calls, self.index = list(calls), 0

    def setup(self, problem, timeout):
        return None, 0.0, None

    def step(self, observation, timeout):
        if self.index >= len(self.calls):
            raise RuntimeError("Replay requested more planner calls than were recorded")
        call = self.calls[self.index]
        self.index += 1
        if abs(call["time_s"] - observation["time_s"]) > 1e-9:
            raise RuntimeError("Replay observation time differs from the recorded call")
        return call["response"], call["latency_s"], call["error"]

    def close(self):
        pass


# ----------------------------------------------------------------------------- episode

@dataclass
class Pending:
    call: int
    observed_s: float
    ready_s: float
    response: object
    error: object


def observation(t, data, ids, plan, world, supervisor, feedback, interventions, time_limit):
    qi, vi = ids
    reference = plan.at(t)
    obstacles = []
    for obstacle in world.obstacles:
        state = obstacle.state(t)
        state["voxels_xyz"] = obstacle.voxels(t)
        obstacles.append(state)
    return dict(time_s=t, time_limit_s=time_limit,
                robot=dict(q=data.qpos[qi].copy(), v=data.qvel[vi].copy(),
                           reference=dict(q=reference[0], v=reference[1], a=reference[2])),
                plan=plan.to_list(t),
                supervisor=dict(protective_stop=supervisor.active(t), stop_until_s=float(supervisor.stop_until),
                                interventions=interventions),
                obstacles=obstacles, feedback=feedback)


def run_episode(task, planner, output, cfg=None, method=None):
    """Simulate one task with one method and write its complete evidence directory."""
    import mujoco
    from benchmarks.generic_motion_v1.model import Model
    from benchmarks.generic_motion_v1.simulation.execution import _torque, configuration

    cfg = cfg or load_config()
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    physics = configuration()
    protocol, sup_cfg = cfg["protocol"], cfg["supervisor"]
    started = time.perf_counter()
    events, calls = [], []
    result = dict(benchmark=BENCHMARK, task_id=task["task_id"], method=method or {}, status="infrastructure_error",
                  source_hashes=source_hashes(), task_sha256=digest(task), host=platform.node(),
                  python=platform.python_version())
    try:
        world = World.from_task(task)
        metadata = build_simulation(world, output / "model", physics,
                                    dict(problem_id=task["task_id"], scene_id=task["layout_id"]))
        model = mujoco.MjModel.from_xml_path(metadata["xml_path"])
        data = mujoco.MjData(model)
        qi = np.asarray(metadata["qpos_ids"], int)
        vi = np.asarray(metadata["dof_ids"], int)
        ui = np.asarray(metadata["actuator_ids"], int)
        effort = np.asarray(metadata["effort_limits_nm"], float)
        mocap = metadata["obstacle_mocap_ids"]
        checker = GeometryChecker(Model())
        limits = motion_limits(cfg, checker.model)
        plan = Plan.from_list(task["public"]["initial_plan"])
        q0, v0, _ = plan.at(0.0)
        initial = task["public"]["initial_state"]
        if not (np.allclose(q0, initial["q"], atol=1e-12, rtol=0) and np.allclose(v0, initial["v"], atol=1e-12, rtol=0)):
            raise ValueError("Initial plan does not pass through the frozen initial state")
        goal = np.asarray(task["public"]["goal"]["q"], float)
        dt = float(model.opt.timestep)
        time_limit = float(protocol["time_limit_s"])
        steps = int(round(time_limit / dt))
        supervisor_every = max(1, int(round(sup_cfg["check_period_s"] / dt)))
        supervisor = Supervisor(sup_cfg, limits["acceleration"], checker)
        problem = public_problem(task, world, cfg, dt)
        _, setup_seconds, setup_error = planner.setup(problem, protocol["setup_timeout_s"])
        events.append(dict(type="setup", wall_s=setup_seconds, error=setup_error))
        planner_enabled = setup_error is None
        n_obs = len(world.obstacles)
        log = dict(time=np.zeros(steps + 1), qpos=np.zeros((steps + 1, 7)), qvel=np.zeros((steps + 1, 7)),
                   qacc=np.zeros((steps + 1, 7)), qref=np.zeros((steps + 1, 7)), vref=np.zeros((steps + 1, 7)),
                   aref=np.zeros((steps + 1, 7)), torque=np.zeros((steps + 1, 7)),
                   torque_requested=np.zeros((steps + 1, 7)), obstacles=np.zeros((steps + 1, n_obs, 3)),
                   penetration_m=np.zeros(steps + 1), protective_stop=np.zeros(steps + 1, bool))
        monitor = dict(time=[], distance=[], required=[], obstacle_approach=[], robot_approach=[])
        data.qpos[qi] = q0
        data.qvel[vi] = v0
        for k, obstacle in enumerate(world.obstacles):
            data.mocap_pos[mocap[k]] = obstacle.motion.at(0.0)[0]
        mujoco.mj_forward(model, data)
        pending, feedback = None, None
        next_call, goal_since, completion, collision = 0.0, None, None, None
        last = -1
        for step in range(steps + 1):
            t = step * dt
            if not np.isclose(data.time, t, rtol=0, atol=1e-9):
                raise FloatingPointError("Physical clock drift")
            if pending is not None and t >= pending.ready_s - 1e-12:
                plan, feedback = _apply(pending, plan, limits, supervisor, events)
                pending = None
            for k, obstacle in enumerate(world.obstacles):
                data.mocap_pos[mocap[k]] = obstacle.motion.at(t)[0]
            mujoco.mj_step1(model, data)
            q, v = data.qpos[qi].copy(), data.qvel[vi].copy()
            if step % supervisor_every == 0 and n_obs:
                measurement, stopped = supervisor.check(t, q, v, plan, world)
                if measurement is not None:
                    monitor["time"].append(t)
                    for key in ("distance", "required", "obstacle_approach", "robot_approach"):
                        monitor[key].append(measurement[key])
                if stopped is not None:
                    plan = stopped
                    events.append(supervisor.interventions[-1])
            target = plan.at(t)
            requested = _torque(mujoco, model, data, target, (qi, vi), physics, False)
            command = np.clip(requested, -effort, effort)
            data.ctrl[ui] = command
            mujoco.mj_forward(model, data)
            if any(int(w.number) for w in data.warning):
                raise FloatingPointError("MuJoCo integration warnings")
            depth = 0.0
            hits = []
            for i in range(data.ncon):
                contact = data.contact[i]
                if contact.dist < -protocol["contact_band_m"]:
                    depth = min(depth, float(contact.dist))
                    hits.append(sorted((int(contact.geom1), int(contact.geom2))))
            log["time"][step] = t
            log["qpos"][step], log["qvel"][step], log["qacc"][step] = data.qpos[qi], data.qvel[vi], data.qacc[vi]
            log["qref"][step], log["vref"][step], log["aref"][step] = target
            log["torque"][step], log["torque_requested"][step] = command, requested
            if n_obs:
                log["obstacles"][step] = data.mocap_pos[mocap]
            log["penetration_m"][step] = depth
            log["protective_stop"][step] = supervisor.active(t)
            last = step
            if hits:
                collision = dict(time_s=t, engine="mujoco", depth_m=depth,
                                 geoms=[[mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g) for g in pair] for pair in hits[:5]])
                events.append(dict(type="collision", **collision))
                break
            at_goal = (np.max(np.abs(q - goal)) <= protocol["goal_position_tolerance_rad"]
                       and np.max(np.abs(v)) <= protocol["goal_velocity_tolerance_rad_s"])
            goal_since = (goal_since if goal_since is not None else t) if at_goal else None
            if goal_since is not None and t - goal_since >= protocol["goal_dwell_s"] - 1e-12:
                completion = goal_since
                events.append(dict(type="completion", time_s=goal_since, confirmed_s=t))
                break
            if planner_enabled and pending is None and t >= next_call - 1e-12:
                obs = observation(t, data, (qi, vi), plan, world, supervisor, feedback,
                                  len(supervisor.interventions), time_limit)
                response, latency, error = planner.step(obs, max(1.0, time_limit - t + 1.0))
                index = len(calls)
                calls.append(dict(index=index, time_s=t, latency_s=latency, error=error,
                                  response=_serial_response(response)))
                if error is not None:
                    events.append(dict(type="planner_error", call=index, time_s=t, latency_s=latency, error=error))
                    planner_enabled = error != "planner_unavailable"
                pending = Pending(index, t, t + latency, response, error)
                next_call = t + protocol["observation_period_s"]
            if step < steps:
                mujoco.mj_step2(model, data)
        planner.close()
        result.update(_finalize(task, world, cfg, limits, log, last, monitor, events, calls, collision, completion,
                                supervisor, effort, setup_seconds, setup_error, output))
        result["model_xml_sha256"] = metadata["xml_sha256"]
    except Exception:
        try:
            planner.close()
        except Exception:
            pass
        result.update(status="infrastructure_error", error=traceback.format_exc())
        (output / "failure.log").write_text(result["error"])
    result["wall_seconds"] = time.perf_counter() - started
    write_json(output / "events.json", events)
    write_json(output / "calls.json", calls)
    result["artifacts"] = {str(p.relative_to(output)): sha(p) for p in sorted(output.rglob("*"))
                           if p.is_file() and p.name not in ("result.json", "receipt.json") and "model" not in p.parts}
    write_json(output / "result.json", result)
    write_json(output / "receipt.json", dict(result_sha256=sha(output / "result.json")))
    return result


def _serial_response(response):
    if not isinstance(response, dict) or "trajectory" not in response:
        return None
    try:
        return dict(start_time_s=float(response["start_time_s"]), trajectory=serializable(dict(response["trajectory"])))
    except Exception:
        return dict(unserializable=repr(response)[:200])


def _apply(pending, plan, limits, supervisor, events):
    """Validate a ready response and splice it into the commanded plan."""
    response = pending.response
    if pending.error is not None or response is None or response == {}:
        return plan, dict(call=pending.call, status="no_change" if pending.error is None else "planner_error")
    record = dict(type="submission", call=pending.call, observed_s=pending.observed_s, ready_s=pending.ready_s)
    if not isinstance(response, dict) or "trajectory" not in response or "start_time_s" not in response:
        record.update(status="malformed", details=dict(error="Response must contain start_time_s and trajectory"))
    elif supervisor.active(float(response["start_time_s"])):
        record.update(status="protective_stop_active", details=dict(stop_until_s=supervisor.stop_until))
    else:
        segment, status, details = check_submission(plan, response["start_time_s"], response["trajectory"],
                                                    limits, pending.ready_s)
        record.update(status=status, details=details)
        if segment is not None:
            plan = plan.spliced(segment)
            record["trajectory_sha256"] = digest(serializable(segment.candidate))
    events.append(clean(record))
    return plan, dict(call=pending.call, status=record["status"], start_time_s=response.get("start_time_s")
                      if isinstance(response, dict) else None)


def independent_check(task, world, log, last, cutoff):
    """Canonical Drake clearance of every recorded state against the world at that time.

    Values are exact below ``cutoff`` (spheres farther than ``cutoff`` from the
    robot's box hull are omitted and the value saturates there).
    """
    from benchmarks.generic_motion_v1.model import Model
    canonical = Model()
    checker = GeometryChecker(Model())
    count = last + 1
    combined, dynamic, static = np.zeros(count), np.full(count, cutoff), np.zeros(count)
    for k in range(count):
        t, q = log["time"][k], log["qpos"][k]
        moving = world.dynamic_xyz(t)
        rotations, translations, query = checker.poses(q)
        near_static = checker.nearby(rotations, translations, world.static_xyz, cutoff)
        near_moving = checker.nearby(rotations, translations, moving, cutoff)
        canonical.set_voxels(np.vstack([near_static, near_moving]) if len(near_moving) else near_static)
        combined[k] = min(float(canonical.clearance(q)[0]), cutoff)
        if len(near_moving):
            dynamic[k] = min(float(checker.sphere_distances(rotations, translations, near_moving).min()), cutoff)
        fixed = checker.fixed_clearance(query)
        spheres = checker.sphere_distances(rotations, translations, near_static)
        static[k] = min(fixed, float(spheres.min()) if spheres.size else cutoff, cutoff)
    return combined, dynamic, static


def _finalize(task, world, cfg, limits, log, last, monitor, events, calls, collision, completion, supervisor,
              effort, setup_seconds, setup_error, output):
    protocol, tolerances = cfg["protocol"], cfg["metrics"]
    count = last + 1
    arrays = {k: v[:count] for k, v in log.items()}
    combined, dynamic, static = independent_check(task, world, log, last, protocol["independent_cutoff_m"])
    band = protocol["contact_band_m"]
    independent_hits = np.flatnonzero(combined < -band)
    mujoco_hits = np.flatnonzero(arrays["penetration_m"] < -band)
    disagreement = int(np.count_nonzero(((combined < -band) != (arrays["penetration_m"] < -band))
                                        & (np.abs(combined) > band)))
    first = [arrays["time"][i[0]] for i in (independent_hits, mujoco_hits) if len(i)]
    collision_time = min(first) if first else None
    if collision_time is not None and collision is None:
        collision = dict(time_s=collision_time, engine="independent_drake")
        events.append(dict(type="collision", **collision))
    elif collision is not None and len(independent_hits):
        collision["independent_time_s"] = float(arrays["time"][independent_hits[0]])
    horizon = count if collision_time is None else int(np.searchsorted(arrays["time"], collision_time, "right"))
    completed = completion is not None and (collision_time is None or completion < collision_time)
    velocity = np.max(np.abs(arrays["qvel"][:horizon]), axis=0)
    acceleration = np.max(np.abs(arrays["qacc"][:horizon]), axis=0)
    lower, upper = limits["lower"], limits["upper"]
    q = arrays["qpos"][:horizon]
    excess = dict(position_rad=float(max(0.0, np.max(lower - q), np.max(q - upper))),
                  velocity_rad_s=float(max(0.0, np.max(velocity - cfg["robot"]["velocity_limit_rad_s"]))),
                  acceleration_rad_s2=float(max(0.0, np.max(acceleration - cfg["robot"]["acceleration_limit_rad_s2"]))))
    violated = (excess["position_rad"] > tolerances["position_tolerance_rad"]
                or excess["velocity_rad_s"] > tolerances["velocity_tolerance_rad_s"]
                or excess["acceleration_rad_s2"] > tolerances["acceleration_tolerance_rad_s2"])
    interventions = supervisor.interventions
    submissions = [e for e in events if e.get("type") == "submission"]
    reasons = {}
    for e in submissions:
        reasons[e["status"]] = reasons.get(e["status"], 0) + 1
    latencies = np.asarray([c["latency_s"] for c in calls], float)
    trajectory_latencies = np.asarray([c["latency_s"] for c in calls if c["response"] is not None], float)
    if collision_time is not None:
        status = "collision"
    elif violated:
        status = "limit_violation"
    elif not completed:
        status = "timeout"
    elif interventions:
        status = "completed_with_intervention"
    else:
        status = "success"
    end = completion if completed else (collision_time if collision_time is not None else float(arrays["time"][count - 1]))
    path = np.linalg.norm(np.diff(arrays["qpos"][:horizon], axis=0), axis=1).sum() if horizon > 1 else 0.0
    error = arrays["qpos"][:horizon] - arrays["qref"][:horizon]
    np.savez_compressed(output / "rollout.npz", **arrays, independent_clearance_m=combined,
                        dynamic_clearance_m=dynamic, static_clearance_m=static,
                        monitor_time=np.asarray(monitor["time"]), monitor_distance_m=np.asarray(monitor["distance"]),
                        monitor_required_m=np.asarray(monitor["required"]),
                        monitor_obstacle_approach_m_s=np.asarray(monitor["obstacle_approach"]),
                        monitor_robot_approach_m_s=np.asarray(monitor["robot_approach"]))
    percentiles = lambda x: (dict(mean=float(x.mean()), p50=float(np.percentile(x, 50)), p95=float(np.percentile(x, 95)),
                                  max=float(x.max())) if len(x) else None)
    return dict(
        status=status, success=status == "success", completed=bool(completed),
        completion_time_s=float(completion) if completed else None, end_time_s=float(end),
        time_limit_s=protocol["time_limit_s"],
        collision=dict(occurred=collision_time is not None, time_s=collision_time, details=collision,
                       mujoco_penetration_steps=int(len(mujoco_hits)),
                       independent_penetration_steps=int(len(independent_hits)),
                       checker_disagreement_steps=disagreement),
        clearance=dict(minimum_m=float(combined[:horizon].min()), minimum_dynamic_m=float(dynamic[:horizon].min()),
                       minimum_static_m=float(static[:horizon].min()), cutoff_m=protocol["independent_cutoff_m"],
                       checker="canonical Drake Model.clearance on every simulation step; values saturate at cutoff"),
        interventions=dict(count=len(interventions), first_time_s=interventions[0]["time_s"] if interventions else None),
        limits=dict(violated=bool(violated), excess=excess, maximum_velocity_rad_s=velocity.tolist(),
                    maximum_acceleration_rad_s2=acceleration.tolist(),
                    torque_saturation_steps=int(np.count_nonzero(np.any(np.abs(arrays["torque_requested"][:horizon]) > effort + 1e-10, axis=1)))),
        planner=dict(setup_seconds=setup_seconds, setup_error=setup_error, calls=len(calls),
                     errors=sum(c["error"] is not None for c in calls),
                     submissions=len(submissions), submission_outcomes=reasons,
                     deadline_misses=reasons.get("deadline_miss", 0),
                     deadline_miss_rate=reasons.get("deadline_miss", 0) / len(submissions) if submissions else 0.0,
                     latency_s=percentiles(latencies), trajectory_latency_s=percentiles(trajectory_latencies)),
        tracking=dict(maximum_error_rad=float(np.max(np.abs(error))) if horizon else 0.0,
                      rmse_rad=float(np.sqrt(np.mean(error ** 2))) if horizon else 0.0),
        executed=dict(path_length_rad=float(path), steps=int(count)))


def replay(task, episode, output, cfg=None):
    """Re-simulate a recorded episode from its saved calls; return the comparison."""
    episode = Path(episode)
    calls = json.loads((episode / "calls.json").read_text())
    original = json.loads((episode / "result.json").read_text())
    repeated = run_episode(task, RecordedPlanner(calls), output, cfg, original.get("method"))
    with np.load(episode / "rollout.npz") as a, np.load(Path(output) / "rollout.npz") as b:
        identical = all(np.array_equal(a[k], b[k]) for k in ("time", "qpos", "qvel", "qref"))
    keys = ("status", "completion_time_s", "end_time_s")
    return dict(identical_states=bool(identical),
                identical_outcome=all(original.get(k) == repeated.get(k) for k in keys),
                original={k: original.get(k) for k in keys}, replayed={k: repeated.get(k) for k in keys})
