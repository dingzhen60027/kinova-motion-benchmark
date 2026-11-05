"""Render saved MuJoCo states without confusing playback with simulation.

The NPZ must contain ``time`` and actual ``qpos`` emitted by the dynamics
rollout, optionally ``qvel``.  Reference/desired joint angles are never used as
the displayed robot state.  Rendering calls ``mj_forward``, not ``mj_step``.
"""
from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import time as wall_time
from typing import Any, Mapping

import numpy as np


def _sha(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _states(arrays_npz: str | Path, nq: int, nv: int) -> dict[str, np.ndarray]:
    """Reject malformed state logs instead of substituting reference poses."""
    with np.load(arrays_npz, allow_pickle=False) as source:
        missing = {"time", "qpos"} - set(source.files)
        if missing:
            raise ValueError(f"Missing actual simulation state arrays: {sorted(missing)}")
        states = {key: np.asarray(source[key], dtype=float).copy()
                  for key in ("time", "qpos", "qvel") if key in source.files}
    timestamps = states["time"]
    if timestamps.ndim != 1 or not len(timestamps):
        raise ValueError("time must be a nonempty one-dimensional array")
    if not np.all(np.isfinite(timestamps)) or np.any(np.diff(timestamps) <= 0):
        raise ValueError("time must be finite and strictly increasing")
    if states["qpos"].shape != (len(timestamps), nq):
        raise ValueError(f"qpos must have shape ({len(timestamps)}, {nq})")
    if "qvel" in states and states["qvel"].shape != (len(timestamps), nv):
        raise ValueError(f"qvel must have shape ({len(timestamps)}, {nv})")
    if any(not np.all(np.isfinite(value)) for value in states.values()):
        raise ValueError("Actual simulation state arrays must be finite")
    return states


def _frame_indices(timestamps: np.ndarray, fps: float) -> np.ndarray:
    """Nearest saved state at each video frame; no fabricated interpolation."""
    if not math.isfinite(fps) or fps <= 0:
        raise ValueError("fps must be positive and finite")
    duration = float(timestamps[-1] - timestamps[0])
    count = max(1, int(math.ceil(duration * fps)) + 1)
    requested = np.minimum(timestamps[0] + np.arange(count) / fps, timestamps[-1])
    right = np.searchsorted(timestamps, requested, side="left").clip(0, len(timestamps)-1)
    left = np.maximum(0, right-1)
    use_left = np.abs(timestamps[left]-requested) <= np.abs(timestamps[right]-requested)
    return np.where(use_left, left, right)


def _camera(mujoco: Any, model: Any, camera: str | Mapping[str, Any] | None):
    if isinstance(camera, str):
        identifier = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera)
        if identifier < 0:
            raise ValueError(f"MuJoCo model has no camera named {camera!r}")
        return camera, {"kind": "named", "name": camera}
    settings = {"lookat": [0.0, 0.0, 0.65], "distance": 2.4,
                "azimuth": 135.0, "elevation": -22.0}
    if camera is not None:
        unknown = set(camera) - set(settings)
        if unknown:
            raise ValueError(f"Unknown free camera settings: {sorted(unknown)}")
        settings.update(camera)
    lookat = np.asarray(settings["lookat"], dtype=float)
    if lookat.shape != (3,) or not np.all(np.isfinite(lookat)):
        raise ValueError("camera lookat must contain three finite coordinates")
    if not all(math.isfinite(float(settings[key])) for key in ("distance", "azimuth", "elevation")):
        raise ValueError("camera settings must be finite")
    if float(settings["distance"]) <= 0:
        raise ValueError("camera distance must be positive")
    view = mujoco.MjvCamera()
    view.type = mujoco.mjtCamera.mjCAMERA_FREE
    view.lookat[:] = lookat
    for key in ("distance", "azimuth", "elevation"):
        setattr(view, key, float(settings[key]))
    return view, {"kind": "free", **settings, "lookat": lookat.tolist()}


def _assign_state(mujoco: Any, model: Any, data: Any,
                  states: dict[str, np.ndarray], index: int) -> None:
    data.qpos[:] = states["qpos"][index]
    data.qvel[:] = states["qvel"][index] if "qvel" in states else 0
    data.time = states["time"][index]
    mujoco.mj_forward(model, data)


def render_rollout(model_xml: str | Path, arrays_npz: str | Path,
                   output_dir: str | Path, fps: float = 30, width: int = 960,
                   height: int = 720,
                   camera: str | Mapping[str, Any] | None = None,
                   collision_debug: bool = False) -> dict[str, Any]:
    """Save actual-state playback as MP4 and a representative PNG.

    EGL is selected only if the caller has not chosen ``MUJOCO_GL``.  ffmpeg
    encodes RGB frames directly, retaining stderr and failure metadata. Existing
    output files are never overwritten. Paths are resolved independently of cwd.
    """
    if not math.isfinite(fps) or fps <= 0:
        raise ValueError("fps must be positive and finite")
    if isinstance(width, bool) or isinstance(height, bool) or width != int(width) or height != int(height) or min(width, height) <= 0:
        raise ValueError("width and height must be positive integers")
    width, height = int(width), int(height)
    xml_path, state_path = Path(model_xml).resolve(), Path(arrays_npz).resolve()
    output = Path(output_dir).resolve()
    output.mkdir(parents=True, exist_ok=True)
    targets = [output / name for name in ("overview.png", "rollout.mp4", "render.json", "ffmpeg.log")]
    if any(path.exists() for path in targets):
        raise FileExistsError("Render outputs already exist; use a new output directory")
    metadata: dict[str, Any] = {
        "kind": "saved_actual_state_playback", "dynamics_resimulated": False,
        "displayed_state_array": "qpos", "reference_state_displayed": False,
        "model_xml": str(xml_path), "model_xml_sha256": _sha(xml_path),
        "arrays_npz": str(state_path), "arrays_npz_sha256": _sha(state_path),
        "fps": float(fps), "width": width, "height": height,
        "passed": False,
    }
    start = wall_time.perf_counter()
    try:
        ffmpeg = shutil.which("ffmpeg")
        if ffmpeg is None:
            raise RuntimeError("ffmpeg executable is required for MP4 encoding")
        os.environ.setdefault("MUJOCO_GL", "egl")
        import mujoco
        import mujoco.gl_context
        from PIL import Image

        context_type = mujoco.gl_context.GLContext
        metadata.update(mujoco_version=mujoco.__version__, gl_backend_requested=os.environ["MUJOCO_GL"],
                        gl_context_class=f"{context_type.__module__}.{context_type.__qualname__}")
        model = mujoco.MjModel.from_xml_path(str(xml_path))
        model.vis.global_.offwidth = max(width, model.vis.global_.offwidth)
        model.vis.global_.offheight = max(height, model.vis.global_.offheight)
        data = mujoco.MjData(model)
        states = _states(state_path, model.nq, model.nv)
        indices = _frame_indices(states["time"], fps)
        view, camera_metadata = _camera(mujoco, model, camera)
        scene_option = mujoco.MjvOption()
        # Exporter groups 3/4 are canonical collision primitives and conservative
        # obstacle voxels. Keep the mesh/source-shape visualization readable.
        scene_option.geomgroup[:] = [1, 1, 1, int(collision_debug), int(collision_debug), 0]
        metadata.update(camera=camera_metadata, saved_state_count=len(states["time"]),
                        simulation_start_time_seconds=float(states["time"][0]),
                        simulation_end_time_seconds=float(states["time"][-1]),
                        simulation_duration_seconds=float(states["time"][-1]-states["time"][0]),
                        video_nominal_duration_seconds=len(indices)/float(fps),
                        frame_count=len(indices), frame_state_indices=indices.tolist(),
                        frame_state_times_seconds=states["time"][indices].tolist(),
                        sampling="nearest_saved_actual_state_without_interpolation",
                        collision_proxy_overlay=bool(collision_debug),
                        rendered_geometry_groups=[0, 1, 2, 3, 4] if collision_debug else [0, 1, 2],
                        hidden_geometry_groups=[5] if collision_debug else [3, 4, 5])
        command = [ffmpeg, "-nostdin", "-hide_banner", "-loglevel", "warning", "-n",
                   "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{width}x{height}",
                   "-r", str(float(fps)), "-i", "pipe:0", "-an", "-vf",
                   "pad=ceil(iw/2)*2:ceil(ih/2)*2", "-c:v", "libx264", "-preset",
                   "fast", "-crf", "20", "-pix_fmt", "yuv420p", "-movflags",
                   "+faststart", str(output / "rollout.mp4")]
        metadata["ffmpeg_command"] = command
        with (output / "ffmpeg.log").open("wb") as log:
            with mujoco.Renderer(model, height=height, width=width) as renderer:
                process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=log)
                try:
                    assert process.stdin is not None
                    for frame_index, state_index in enumerate(indices):
                        _assign_state(mujoco, model, data, states, int(state_index))
                        renderer.update_scene(data, camera=view, scene_option=scene_option)
                        frame = np.ascontiguousarray(renderer.render(), dtype=np.uint8)
                        if frame.shape != (height, width, 3):
                            raise RuntimeError(f"Unexpected MuJoCo RGB shape: {frame.shape}")
                        if frame_index == len(indices)//2:
                            Image.fromarray(frame).save(output / "overview.png")
                            metadata["overview_state_index"] = int(state_index)
                            metadata["overview_simulation_time_seconds"] = float(data.time)
                        process.stdin.write(frame.tobytes())
                    process.stdin.close()
                    returncode = process.wait(timeout=120)
                    if returncode:
                        raise RuntimeError(f"ffmpeg failed with exit code {returncode}; see ffmpeg.log")
                except BaseException:
                    if process.stdin is not None and not process.stdin.closed:
                        try:
                            process.stdin.close()
                        except BrokenPipeError:
                            pass
                    if process.poll() is None:
                        process.terminate()
                        try:
                            process.wait(timeout=10)
                        except subprocess.TimeoutExpired:
                            process.kill()
                            process.wait(timeout=10)
                    raise
        metadata.update(passed=True, outputs={name: {"path": str(output / name), "sha256": _sha(output / name)}
                                              for name in ("overview.png", "rollout.mp4", "ffmpeg.log")})
    except Exception as error:
        metadata.update(error_type=type(error).__name__, error=str(error))
        raise
    finally:
        metadata["render_wall_seconds"] = wall_time.perf_counter() - start
        (output / "render.json").write_text(json.dumps(metadata, indent=2, allow_nan=False) + "\n")
    return metadata


def play_rollout(model_xml: str | Path, arrays_npz: str | Path,
                 speed: float = 1.0, loop: bool = False,
                 camera: str | Mapping[str, Any] | None = None,
                 collision_debug: bool = False) -> dict[str, Any]:
    """Interactively replay saved actual states; this does not rerun dynamics.

    Space pauses/resumes, Home returns to the first recorded state, and Escape
    closes the official MuJoCo viewer. A graphical desktop is required.
    """
    if not math.isfinite(speed) or speed <= 0:
        raise ValueError("Playback speed must be positive and finite")
    import mujoco
    import mujoco.viewer

    model = mujoco.MjModel.from_xml_path(str(Path(model_xml).resolve()))
    data = mujoco.MjData(model)
    states = _states(Path(arrays_npz).resolve(), model.nq, model.nv)
    view, camera_metadata = _camera(mujoco, model, camera)
    control = {"paused": False, "restart": False}

    def key_callback(keycode: int) -> None:
        if keycode == 32:
            control["paused"] = not control["paused"]
        elif keycode == 268:
            control["restart"] = True

    _assign_state(mujoco, model, data, states, 0)
    index = 0
    timestamp = float(states["time"][0])
    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
        with viewer.lock():
            viewer.opt.geomgroup[:] = [1, 1, 1, int(collision_debug), int(collision_debug), 0]
            if isinstance(view, str):
                viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
                viewer.cam.fixedcamid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, view)
            else:
                viewer.cam.type = view.type
                viewer.cam.lookat[:] = view.lookat
                viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = view.distance, view.azimuth, view.elevation
        previous_wall = wall_time.perf_counter()
        while viewer.is_running():
            current_wall = wall_time.perf_counter()
            if control["restart"]:
                timestamp = float(states["time"][0])
                control["restart"] = False
            elif not control["paused"]:
                timestamp += (current_wall - previous_wall) * speed
            previous_wall = current_wall
            if timestamp > states["time"][-1]:
                if loop and len(states["time"]) > 1:
                    timestamp = float(states["time"][0]) + (timestamp - states["time"][0]) % (states["time"][-1] - states["time"][0])
                else:
                    timestamp = float(states["time"][-1])
            index = max(0, int(np.searchsorted(states["time"], timestamp, side="right"))-1)
            with viewer.lock():
                _assign_state(mujoco, model, data, states, index)
            viewer.sync()
            wall_time.sleep(1/120)
    return {"kind": "saved_actual_state_playback", "dynamics_resimulated": False,
            "displayed_state_array": "qpos", "last_displayed_state_index": index,
            "camera": camera_metadata, "collision_proxy_overlay": bool(collision_debug)}
