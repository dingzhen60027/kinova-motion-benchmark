# Dynamic recovery benchmark

[Project overview](../../README.md) · [Static benchmark](../generic_motion_v1/README.md) · [Configuration](config.json)

Can a Kinova Gen3 that is **already moving** recover safely and efficiently when the environment changes, and still finish its original pick-and-place task? Each task starts from a predefined in-motion state with non-zero joint velocities. Independently moving obstacles invalidate the motion the robot is executing. The evaluated method replans online while the simulation keeps running, and its measured computation time is charged to the task.

The benchmark measures **task completion under disturbance**, not isolated planning queries. It does not prescribe a strategy: waiting, braking, retreating and rerouting are all scored only by their outcome.

## What is reused

| Component | Source | Use here |
|---|---|---|
| Robot and collision model | `generic_motion_v1/model.py` (`Model`) | Kinova Gen3 with welded Robotiq 2F-85, table, tripods and wall; canonical primitive boxes and collision pairs |
| Obstacle representation | `generic_motion_v1/generator/voxelize.py` | Every workspace and moving obstacle is a conservative cover of 0.04 m spheres, so CSDecomp (GPU or CPU), Drake and MuJoCo consume identical geometry |
| Independent collision check | `Model.clearance` | Canonical Drake signed distance at every simulation step |
| Trajectory format | `simulation/reference.py`, `runners/validate.py` | Timed piecewise Bézier or clamped B-spline candidates and derivative control-hull bounds |
| Physics and control | `simulation/mujoco_model.py`, `simulation/execution.py`, `configs/mujoco.json` | Canonical MuJoCo export extended with covers and mocap obstacles; computed-torque PD control at 500 Hz under URDF torque limits |
| Time-optimal line motions | `runners/phase_search.py` (`line_curve`) | Phase-synchronized rest-to-rest motions for nominal plans, witnesses and reference behaviours |
| Sampling-based planner | `runners/ompl_baselines.py` | OMPL 2.0.1 RRTConnect inside the `brake_replan` reference behaviour |

No existing file of `generic_motion_v1` was changed, and its frozen inputs and results are untouched.

## Closed-loop protocol

**Clock and latency.** Physics advances in 2 ms steps. The harness calls the method with an observation every 0.1 s of simulation time while the method is idle. The method runs in a separate process. The harness measures the wall-clock duration `L` of each call, including message transfer, and charges it to simulation time: the robot keeps executing its current reference and the obstacles keep moving for `L` seconds. The response can only take effect from `t + L`. A slower method therefore reacts later and finishes later. A call that never returns is treated as an unanswered call: the episode simply runs to the time limit.

**Responses and splicing.** A method returns `None` (keep the commanded plan) or a trajectory with an explicit start time `t_s`. A submission is accepted only if all of the following hold:
- `t_s >= t + L`; otherwise it is a **deadline miss**.
- Position and velocity at `t_s` match the commanded reference within 1e-6.
- It ends at rest.
- Its control hulls stay within the joint limits, the velocity limit (1 rad/s) and the acceleration limit (2.5 rad/s²).

An accepted submission replaces the plan from `t_s` onward. A rejected one never changes the reference, and its reason is recorded. Continuity is checked against the commanded reference, which the method knows exactly. The physical MuJoCo state is assigned once, at the start of the episode, and is never reset; tracking error is reported.

**Observations.** At setup, before `t = 0` and not timed (up to 600 s), the method receives:
- the robot limits and model identity;
- the static scene, as primitives and sphere cover;
- the goal configuration;
- the nominal waypoints;
- the plan executing at `t = 0`.

Each step then adds:
- the time;
- the measured joint state;
- the commanded reference and the future plan;
- the supervisor state;
- each moving obstacle's current shape, pose, velocity and world-frame sphere cover;
- feedback on the previous submission.

Obstacle motion definitions, the witness and task labels are never sent. `public_problem` and `observation` in `harness.py` are the complete interface.

**External safety supervisor.** Every 10 ms the harness applies a simplified ISO/TS 15066 speed and separation monitor against moving obstacles:

`S_p = v_o (T_r + T_s) + v_r T_r + v_r T_s / 2 + C`

- `v_o`, `v_r`: obstacle and robot-point approach speeds along the closest-point normal.
- `T_r = 0.05 s`: reaction time.
- `T_s`: the robot's joint-space stopping time.
- `C = 0.02 m`: intrusion distance.

If a moving robot comes closer than `S_p`, the supervisor stops it along its current velocity direction at the acceleration limit. Submissions that would start during the stop are rejected. Each stop counts as one **intervention**. The supervisor does not guarantee safety: an obstacle can still hit a stopped robot.

**Collisions and completion.**
- **Collision:** penetration deeper than the canonical 2 µm contact band, reported either by MuJoCo contacts during the rollout or by the canonical Drake checker over every recorded step afterwards. The episode ends at the first collision.
- **Completion:** all joints within 0.02 rad of the goal and slower than 0.05 rad/s for 0.2 s.
- **Time limit:** 30 s.

## Metrics

Each episode has one status:
- `success`: safe completion, meaning completed with no collision, no intervention and no executed-limit violation.
- `completed_with_intervention`
- `timeout`
- `limit_violation`
- `collision`
- `infrastructure_error`: a harness fault, which invalidates a run rather than counting against a method.

`summary.json` reports per method, per disturbance family and per workspace:
- **Rates:** safe completion (with a 95% interval from resampling layouts), completion, collision, timeout and intervention.
- **Time:** median end-to-end completion time, and its ratio to the witness's.
- **Clearance:** median minimum clearance to moving obstacles.
- **Motion constraints:** executed violations (velocity +0.02 rad/s, acceleration +0.5 rad/s² tolerance for discrete-time control at acceleration switches), torque saturation steps, and rejected submissions by reason.
- **Planning:** call latency percentiles and deadline-miss rate.

Failures stay in every denominator. Each episode directory keeps:
- `rollout.npz`: executed and commanded states, torques, obstacle positions, independent clearances and supervisor measurements;
- `events.json`: submissions, interventions, collisions and completion;
- `calls.json`: every call with its measured latency and response;
- `result.json` and its receipt.

`verify` replays recorded calls and requires bit-identical physical states.

## Tasks

Three workspaces, each built from boxes on the canonical table:
- **Shelf:** a two-level shelving unit on a plinth. The task places a part, picked from a side tote, into a compartment.
- **Cluttered workstation:** a pick bin and an assembly fixture on opposite sides of a table with clutter. The task moves from one to the other.
- **Machine opening:** an enclosure with a confined front window. The task places a part, picked from a side tote, inside through the window.

Endpoints come from IK of semantic tool poses. The nominal motion consists of rest-to-rest joint-space segments: pick, straight lift, transfer, then a straight insertion or descent. It runs at a 50% velocity and 40% acceleration override; recovery may use the full limits. The episode starts while the robot is lifting the part.

| Family | Workspaces | Disturbance |
|---|---|---|
| `transient_crossing` | all | A box or sphere crosses the robot's corridor once (mean 0.25–0.45 m/s) and leaves |
| `persistent_blockage` | all | An object standing on the table or held in place moves into the route and stays; another route remains |
| `entrance_blockage` | machine | A guard door drops in front of the window (or a shutter slides across it), blocks it for 1.5–3 s, then clears |
| `distractor` (control) | all | An obstacle moves nearby without conflicting; the unchanged nominal is safe and never triggers the supervisor |

**Method-independent acceptance.** A task is accepted only through geometry, the nominal motion and a privileged witness; no evaluated method is ever consulted. The rules:
- **Plausible obstacles:** obstacles stay clear of furniture and fixed equipment.
- **No sudden appearance:** every obstacle is at least 0.25 m from the robot at `t = 0`.
- **Warning time:** it first comes within 5 cm of the nominal motion at least 1.0 s after its observable onset: its departure, or `t = 0` if it is already moving.
- **A real disturbance:** executing the nominal motion unchanged must collide with it.
- **Stable completion:** final obstacle poses stay at least 2 cm from the robot at its goal pose.
- **Usable layout:** a layout is used only if some nominal motion exists.

Rejections are counted by reason in each task's `generation` record, and a task that exhausts its attempts stays registered as a generation failure.

**Completion witness.** For every accepted task a privileged search, which knows the obstacle motions, finds a completion.
- **Strategies, in order:** continue; brake, wait, resume; brake, retreat to the last waypoint, wait, resume; brake and detour through a via configuration; brake and follow an OMPL RRTConnect path planned against the obstacles' rest poses.
- **Timing:** each strategy follows the nominal motion until at least the disturbance onset.
- **Screening:** candidates are screened kinematically with the supervisor rule.
- **Closed-loop validation:** the chosen witness is then executed in the same harness and must reach `success`.

Its validation summary is stored in the task's `oracle` section; its evidence stays under `runs/`. The witness proves each task feasible. It is not a baseline and is never visible to methods.

**Splits.** `smoke` (1 layout per workspace), `dev` (5) and `test` (20) use disjoint, SHA256-derived PCG64 streams; `check` also rejects any layout shared between splits. Tune methods on `dev` only. `test` is frozen and should be run only for final evaluation of a fixed method.

| Split | Layouts | Accepted / registered | Transient | Persistent | Entrance | Distractor |
|---|---:|---:|---:|---:|---:|---:|
| `smoke` | 3 | 8 / 10 | 3 | 1 | 1 | 3 |
| `dev` | 15 | 42 / 50 | 15 | 8 | 4 | 15 |
| `test` | 60 | 162 / 200 | 57 | 33 | 14 | 58 |

On `test`:
- **Persistent blockages are the hardest to place.** They are accepted in 18/20 workstation, 13/20 shelf and 2/20 machine layouts. In many machine layouts no stretch of the transfer is clear of both the start and goal arm poses for long enough to meet the warning rule.
- **Witness strategies:**

  | Strategy | Tasks |
  |---|---:|
  | continue (all distractors) | 58 |
  | brake–wait–resume | 30 |
  | brake–retreat–resume | 9 |
  | detour through a via configuration | 39 |
  | OMPL-planned detour | 26 |

- **Witness completion times:** medians run from 5.7 s (distractor) to 9.1 s (entrance blockage).
- **Disturbance speeds:** peaks of 0.47–1.12 m/s.
- **Without reaction:** executing the nominal motion unchanged reaches a median minimum clearance of −7.6 cm, i.e. it collides.

## Reference behaviours on `dev`

These check that the benchmark separates behaviours; they are not proposed methods.
- **Behaviours:** `nominal` never replans. `brake_wait` stops on a constant-velocity predicted conflict and resumes the nominal route once it is clear. `brake_replan` additionally plans an OMPL RRTConnect detour around obstacles that stay put.
- **Run:** 42 accepted `dev` tasks, 168 episodes in total, on an Intel i9-14900HX (32 threads), CPU only, with 3 concurrent episodes.
- **Verification:** all 168 episodes passed integrity checks, and 9 replays (3 per method) were bit-identical.
- **Test split:** deliberately not run.

Safe completion by family; the overall rate's interval comes from resampling layouts:

| Behaviour | Overall (95% CI) | Distractor (15) | Transient crossing (15) | Persistent blockage (8) | Entrance blockage (4) |
|---|---|---:|---:|---:|---:|
| `nominal` | 36% (33–38) | 100% | 0%, all collide | 0%, all time out | 0%, all time out |
| `brake_wait` | 45% (38–53) | 100% | 0%, all collide | 0%, all time out | 100% |
| `brake_replan` | 57% (50–64) | 100% | 0%, all collide | 62%, rest complete with intervention | 100% |
| witness (privileged) | 100% | 100% | 100% | 100% | 100% |

What the results show:
- **External supervisor:** it stopped the blind `nominal` motion in 64% of episodes. That prevented collisions with the blockers, but not with crossing obstacles, which hit the stopped arm.
- **Persistent blockages:** waiting never completes them. Geometric replanning does: its detours took 9.0 s median, 1.6× the witness, and its OMPL calls cost 0.9 s median (3.2 s max) of charged latency. Its 5 cm clearance margin is not speed-aware, so 3 of 8 detours triggered a protective stop.
- **Transient crossings:** none of the reference behaviours solves them, although every witness completes. Constant-velocity prediction reacts too late to accelerating obstacles, and stopping in place is not enough when the obstacle sweeps through the stopping position. This is the open problem the benchmark is built to measure.
- **Latency:** no submission missed its deadline or was rejected; 95th-percentile call latency stayed below 20 ms except during OMPL replanning.

## Commands

```bash
export CSDECOMP_CONDA_PREFIX="$PWD/.venv"   # or the Conda prefix of local/README.md
bash local/python.sh scripts/dynamic_recovery.py check
bash local/python.sh scripts/dynamic_recovery.py run --split smoke --methods nominal brake_wait witness --output runs/dynamic_recovery/smoke
bash local/python.sh scripts/dynamic_recovery.py verify --run runs/dynamic_recovery/smoke --replays 3
bash local/python.sh scripts/dynamic_recovery.py summarize --run runs/dynamic_recovery/smoke
bash local/python.sh -m pytest -q benchmarks/dynamic_recovery/tests
```

`run` is resumable: finished episodes, failures included, are reused, and an interrupted episode directory is kept aside. `generate --splits smoke dev test --workers N` rebuilds the frozen data deterministically.

The CPU path needs Drake 1.51.1, MuJoCo 3.3.7, the CSDecomp 0.1.2 wheel and OSQP from `local/requirements.lock.txt`; `brake_replan` also needs OMPL 2.0.1 in `.local_setup/solver_probes/ompl`. No GPU is required. Methods may use CSDecomp's CUDA checker on a GPU machine, since obstacles arrive as the same sphere covers.

## Adding a method

```python
class MyMethod:
    def setup(self, problem):          # untimed; static scene, goal, limits, initial plan
        ...
    def step(self, observation):       # timed; return None or a spliced trajectory
        t = observation["time_s"]
        return dict(start_time_s=t + 0.05, trajectory=dict(kind="bezier", controls=..., durations=...))
```

Register `"module:Class"` in `policies.REGISTRY`, or pass it to `harness.WorkerPlanner`. `trajectory.Plan.from_list(observation["plan"])` evaluates the commanded reference, and `braking`, `point_to_point` and `world.GeometryChecker` are available helpers.

The bundled `nominal`, `brake_wait` and `brake_replan` behaviours are reference compositions used to check that the benchmark separates behaviours. They are not proposed methods.

## Limitations

- Collision checks are finite-resolution numerical samples, not continuous certificates.
- Sphere covers inflate obstacles by up to about 3 cm.
- Obstacles are kinematic, observed perfectly and without delay.
- Torque limits come from the canonical URDF and are not calibrated hardware data.
- Latency is wall-clock time and depends on the machine; record hardware and keep concurrent episodes low for formal comparisons.
