"""Dynamic-fabrics worker (B6), run by the separate fabrics environment (``.fabrics.venv``).

This file must not import the benchmark package (that environment has no Drake); it speaks JSON lines over
stdin/stdout with ``fabrics_bridge.FabricsBridge``:

* ``{"cmd": "setup", ...}``: builds a URDF from the benchmark's own kinematics (joint points and axes at the zero
  configuration, the product-of-exponentials model the benchmark uses) with one fixed child link per collision
  sphere, then builds (or loads from ``cache``) the fabric planner of Spahn et al. (2023): a joint-space attractor,
  joint-limit avoidance, ``static`` spherical obstacles and ``dynamic`` spherical obstacles with known velocity and
  acceleration (dynamic fabrics). Replies ``{"ok": true, ...}``.
* ``{"cmd": "rollout", ...}``: integrates the fabric from (q, qdot) for ``steps`` x ``dt`` with the dynamic
  obstacles moving at their reported velocity, clipping accelerations and velocities to the robot limits; replies
  the applied accelerations.
"""
import json
import os
import sys
import time

import numpy as np


def urdf(description):
    points, axes = np.asarray(description["points"]), np.asarray(description["axes"])
    lower, upper = description["lower"], description["upper"]
    parts = ['<?xml version="1.0"?>', '<robot name="kinova_fabrics">', '  <link name="base_link"/>']
    previous, parent = np.zeros(3), "base_link"
    for j in range(7):
        x, y, z = points[j] - previous
        ax, ay, az = axes[j]
        parts += [f'  <joint name="joint_{j + 1}" type="revolute">', f'    <parent link="{parent}"/>',
                  f'    <child link="link_{j + 1}"/>', f'    <origin xyz="{x:.9f} {y:.9f} {z:.9f}" rpy="0 0 0"/>',
                  f'    <axis xyz="{ax:.9f} {ay:.9f} {az:.9f}"/>',
                  f'    <limit lower="{lower[j]}" upper="{upper[j]}" effort="100" velocity="1"/>', '  </joint>',
                  f'  <link name="link_{j + 1}"/>']
        previous, parent = points[j], f"link_{j + 1}"
    for i, sphere in enumerate(description["spheres"]):
        link = "base_link" if sphere["link"] == 0 else f"link_{sphere['link']}"
        x, y, z = sphere["offset"]
        parts += [f'  <joint name="sphere_joint_{i}" type="fixed">', f'    <parent link="{link}"/>',
                  f'    <child link="sphere_{i}"/>', f'    <origin xyz="{x:.9f} {y:.9f} {z:.9f}" rpy="0 0 0"/>',
                  '  </joint>', f'  <link name="sphere_{i}"/>']
    parts.append("</robot>")
    return "\n".join(parts)


class Worker:
    def setup(self, message):
        from forwardkinematics.urdfFks.generic_urdf_fk import GenericURDFFk
        from fabrics.planner.parameterized_planner import ParameterizedFabricPlanner
        from fabrics.planner.serialized_planner import SerializedFabricPlanner
        from mpscenes.goals.goal_composition import GoalComposition
        started = time.perf_counter()
        self.spheres = message["spheres"]
        self.links = [f"sphere_{i}" for i in range(len(self.spheres))]
        self.n_static, self.n_dynamic = int(message["static_obstacles"]), int(message["dynamic_obstacles"])
        cache = message.get("cache")
        if cache and os.path.exists(cache):
            self.planner = SerializedFabricPlanner(cache)
            loaded = True
        else:
            fk = GenericURDFFk(urdf(message), root_link="base_link", end_links=self.links + ["link_7"])
            planner = ParameterizedFabricPlanner(7, fk)
            goal = GoalComposition(name="goal", content_dict={"subgoal0": dict(
                weight=1.0, is_primary_goal=True, indices=list(range(7)), desired_position=[0.0] * 7,
                epsilon=0.05, type="staticJointSpaceSubGoal")})
            limits = [[float(lo), float(hi)] for lo, hi in zip(message["lower"], message["upper"])]
            planner.set_components(collision_links=self.links, goal=goal, limits=limits,
                                   number_obstacles=self.n_static, number_dynamic_obstacles=self.n_dynamic)
            planner.concretize()
            if cache:
                os.makedirs(os.path.dirname(cache), exist_ok=True)
                tmp = f"{cache}.{os.getpid()}.tmp"
                planner.serialize(tmp)
                os.replace(tmp, cache)
            self.planner = planner
            loaded = False
        self.radii = {f"radius_body_{link}": float(s["radius"]) for link, s in zip(self.links, self.spheres)}
        return dict(ok=True, seconds=time.perf_counter() - started, loaded=loaded)

    def fk(self, message):
        """Sphere-link positions for configurations (consistency test against the benchmark's kinematics)."""
        from forwardkinematics.urdfFks.generic_urdf_fk import GenericURDFFk
        links = [f"sphere_{i}" for i in range(len(message["spheres"]))]
        fk = GenericURDFFk(urdf(message), root_link="base_link", end_links=links + ["link_7"])
        out = [[np.asarray(fk.numpy(np.asarray(q, float), link, position_only=True)).reshape(-1).tolist()
                for link in links] for q in message["q"]]
        return dict(ok=True, positions=out)

    def rollout(self, message):
        q, qd = np.asarray(message["q"], float), np.asarray(message["qdot"], float)
        goal, weight = np.asarray(message["goal"], float), float(message["goal_weight"])
        dt, steps = float(message["dt"]), int(message["steps"])
        vmax, amax = np.asarray(message["vmax"], float), np.asarray(message["amax"], float)
        static = np.asarray(message["static"], float).reshape(-1, 4)
        dynamic = np.asarray(message["dynamic"], float).reshape(-1, 10)
        arguments = dict(self.radii, x_goal_0=goal, weight_goal_0=weight)
        for i in range(self.n_static):
            arguments[f"x_obst_{i}"], arguments[f"radius_obst_{i}"] = static[i, :3], static[i, 3]
        out = []
        for k in range(steps):
            for i in range(self.n_dynamic):
                p, v, a, r = dynamic[i, :3], dynamic[i, 3:6], dynamic[i, 6:9], dynamic[i, 9]
                tau = k * dt
                arguments[f"x_obst_dynamic_{i}"] = p + v * tau
                arguments[f"xdot_obst_dynamic_{i}"] = v
                arguments[f"xddot_obst_dynamic_{i}"] = a
                arguments[f"radius_obst_dynamic_{i}"] = r
            action = np.asarray(self.planner.compute_action(q=q, qdot=qd, **arguments), float).reshape(-1)
            if not np.all(np.isfinite(action)):
                action = -qd / dt
            v_next = np.clip(qd + np.clip(action, -amax, amax) * dt, -vmax, vmax)
            applied = (v_next - qd) / dt
            q = q + qd * dt + 0.5 * applied * dt * dt
            qd = v_next
            out.append(applied.tolist())
        return dict(ok=True, acc=out)


def main():
    import logging
    import warnings
    logging.disable(logging.WARNING)
    warnings.simplefilter("ignore")
    # Libraries print to stdout (e.g. the URDF parser); keep the protocol on a private copy of the stream.
    protocol = os.fdopen(os.dup(1), "w")
    os.dup2(2, 1)
    sys.stdout = sys.stderr
    worker = Worker()
    for line in sys.stdin:
        message = json.loads(line)
        try:
            if message["cmd"] == "setup":
                reply = worker.setup(message)
            elif message["cmd"] == "rollout":
                reply = worker.rollout(message)
            elif message["cmd"] == "fk":
                reply = worker.fk(message)
            elif message["cmd"] == "close":
                break
            else:
                reply = dict(ok=False, error=f"unknown command {message['cmd']}")
        except Exception as exc:  # reported to the bridge, which treats it as a planner failure
            import traceback
            reply = dict(ok=False, error=f"{type(exc).__name__}: {exc}", trace=traceback.format_exc()[-2000:])
        protocol.write(json.dumps(reply) + "\n")
        protocol.flush()


if __name__ == "__main__":
    main()
