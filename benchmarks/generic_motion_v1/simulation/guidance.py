"""Display-only endpoint poses and actual TCP trace for saved MuJoCo rollouts."""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from ..common import recorded_path, sha


START = (.16, .38, .77, 1.)
GOAL = (.08, .54, .38, 1.)
TRACE = (.12, .43, .64, .85)


class PlaybackGuide:
    """Use original task endpoints, never relabel an attained pose as the goal.

    Decorations live in MjvScene only. They cannot change model geometry,
    contacts, saved states, simulation outcomes or planning scores.
    """

    def __init__(self, mujoco, model, states, model_xml, arrays_npz):
        self.mj = mujoco
        root = Path(arrays_npz).resolve().parent
        metadata = json.loads(Path(model_xml).with_name('model_metadata.json').read_text())
        if sha(Path(model_xml)) != metadata['xml_sha256']:
            raise ValueError('Endpoint guidance model hash differs from its metadata')
        result = json.loads((root / 'result.json').read_text())
        source = recorded_path(result['source_attempt']) / 'input.json'
        if sha(source) != result['source_input_sha256']:
            raise ValueError('Endpoint guidance task input hash changed')
        case = json.loads(source.read_text())
        self.result, self.case = result, case
        self.start_time, self.end_time = map(float, states['time'][[0, -1]])
        self.tcp_body = int(metadata['tcp_body_id'])
        self.local_tcp = np.asarray(metadata['tcp_local_transform'], dtype=float)[:3, 3]
        self.joints = np.asarray(metadata['joint_ids'], dtype=int)
        self.qpos_ids = np.asarray(metadata['qpos_ids'], dtype=int)
        scratch = mujoco.MjData(model)

        def positions(qpos):
            scratch.qpos[:] = qpos
            mujoco.mj_kinematics(model, scratch)
            tcp = scratch.xpos[self.tcp_body] + scratch.xmat[self.tcp_body].reshape(3, 3) @ self.local_tcp
            return tcp.copy(), np.vstack([scratch.xanchor[self.joints], tcp])

        endpoints = []
        self.endpoint_qpos = []
        for key in ('q_start', 'q_goal'):
            q = np.asarray(case[key], dtype=float)
            if q.shape != self.qpos_ids.shape or not np.isfinite(q).all():
                raise ValueError('Invalid task endpoint for playback guidance')
            qpos = model.qpos0.copy()
            qpos[self.qpos_ids] = q
            self.endpoint_qpos.append(qpos.copy())
            endpoints.append(positions(qpos))
        (self.start_tcp, self.start_pose), (self.goal_tcp, self.goal_pose) = endpoints
        # This coarse trace is only an orientation aid, never collision evidence.
        self.indices = np.unique(np.linspace(0, len(states['time']) - 1,
                                            min(181, len(states['time'])), dtype=int))
        self.trace = np.asarray([positions(states['qpos'][i])[0] for i in self.indices])
        self._times = states['time'].copy()
        self._trace_times = states['time'][self.indices]
        self.metadata = dict(endpoint_source='original_task_q_start_q_goal',
                             input_sha256=result['source_input_sha256'],
                             start_tcp_world_m=self.start_tcp.tolist(),
                             goal_tcp_world_m=self.goal_tcp.tolist(),
                             trace_source='actual_saved_qpos_forward_kinematics',
                             trace_samples=len(self.indices), display_only=True,
                             start_color='blue', goal_color='green')

    def tcp(self, data):
        return data.xpos[self.tcp_body] + data.xmat[self.tcp_body].reshape(3, 3) @ self.local_tcp

    def decorate(self, scene, data, show_poses=False, compact=False):
        """Append to the renderer scene or an emptied viewer.user_scn."""
        mj = self.mj

        def geom(kind, pos, color, size=(.025, .025, .025), label=''):
            if scene.ngeom >= scene.maxgeom:
                raise RuntimeError('Insufficient MuJoCo scene capacity for playback guidance')
            g = scene.geoms[scene.ngeom]
            mj.mjv_initGeom(g, kind, np.asarray(size), np.asarray(pos),
                           np.eye(3).reshape(-1), np.asarray(color, dtype=np.float32))
            g.category = mj.mjtCatBit.mjCAT_DECOR
            g.objid = -1
            g.label = label
            scene.ngeom += 1
            return g

        def link(a, b, color, radius=.003, arrow=False):
            if np.linalg.norm(b - a) < 1e-8:
                return
            kind = mj.mjtGeom.mjGEOM_ARROW if arrow else mj.mjtGeom.mjGEOM_CAPSULE
            g = geom(kind, a, color)
            mj.mjv_connector(g, kind, radius, a, b)

        for i, (a, b) in enumerate(zip(self.trace[:-1], self.trace[1:])):
            # Future recording is pale; the portion already played is emphasized.
            played = self.indices[i+1] <= np.searchsorted(self._times, data.time, side='right') - 1
            color = TRACE if played else (*TRACE[:3], .22)
            link(a, b, color, .0035 if played else .002)
        # Direction arrows span multiple samples so they remain visible.
        current = np.searchsorted(self._trace_times, data.time, side='right') - 1
        if 0 <= current < len(self.trace) - 1:
            link(self.trace[current], self.trace[min(current + 3, len(self.trace) - 1)], TRACE, .007, True)
        for tcp, pose, color, text, shape in (
            (self.start_tcp, self.start_pose, START, 'START [1]', mj.mjtGeom.mjGEOM_SPHERE),
            (self.goal_tcp, self.goal_pose, GOAL, 'GOAL [2]', mj.mjtGeom.mjGEOM_BOX),
        ):
            if show_poses:
                faint = (*color[:3], .42)
                for a, b in zip(pose[:-1], pose[1:]):
                    link(a, b, faint, .010)
                for point in pose:
                    geom(mj.mjtGeom.mjGEOM_SPHERE, point, faint, (.014,) * 3)
            geom(shape, tcp, color)
            label_pos = tcp + [0, 0, .10 if compact else .17]
            link(tcp, label_pos, color, .002)
            geom(mj.mjtGeom.mjGEOM_LABEL, label_pos, color, label=text[0] if compact else text)
        geom(mj.mjtGeom.mjGEOM_SPHERE, self.tcp(data), TRACE, (.013,) * 3)

    def text(self, timestamp, paused=False, speed=1.):
        elapsed = float(np.clip(timestamp - self.start_time, 0, self.end_time - self.start_time))
        duration = self.end_time - self.start_time
        fraction = elapsed / duration if duration else 1.
        phase = 'END OF RECORDING' if fraction >= 1 else 'PAUSED' if paused else 'PLAYING'
        bar = '#' * int(20 * fraction) + '-' * (20 - int(20 * fraction))
        failed = [k for k, v in self.result.get('checks', {}).items() if v is False]
        verdict = ('CHECK FAILED: ' + ', '.join(failed)) if failed else self.result['status']
        return [f"{self.result['method']} | {self.case['problem_id']}",
                'BLUE sphere = START | GREEN box = GOAL',
                'Colored arm outlines = fixed endpoint poses',
                'White robot = actual state | Cyan arrows = TCP trace',
                f'{phase}   {elapsed:.2f} / {duration:.2f} s   {fraction:.0%}   {speed:g}x',
                f'[{bar}]', verdict,
                'SPACE Play/Pause | HOME Replay | 1 Start | 2 End | G Guides']


def load_guide(mujoco, model, states, model_xml, arrays_npz):
    if not Path(model_xml).with_name('model_metadata.json').exists():
        return None
    return PlaybackGuide(mujoco, model, states, model_xml, arrays_npz)
