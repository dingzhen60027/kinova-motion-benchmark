"""Actual-state rendering contract and a small real EGL/ffmpeg integration."""
import json
import shutil

import numpy as np
import pytest

from benchmarks.generic_motion_v1.simulation.render import (
    _frame_indices, _states, render_rollout,
)


def test_frame_sampling_uses_physical_time_and_exact_saved_states():
    timestamps = np.array([0., .01, .03, .08, .1])
    indices = _frame_indices(timestamps, 20)
    assert indices.tolist() == [0, 2, 4]
    assert np.array_equal(timestamps[indices], [0., .03, .1])


@pytest.mark.parametrize("fps", [0, -1, float("nan"), float("inf")])
def test_invalid_video_rate_rejected(fps):
    with pytest.raises(ValueError, match="fps"):
        _frame_indices(np.array([0., 1.]), fps)


@pytest.mark.parametrize("arrays,match", [
    ({"time": [0., 1.], "qref": [[0.], [1.]]}, "Missing actual"),
    ({"time": [0., 0.], "qpos": [[0.], [1.]]}, "strictly increasing"),
    ({"time": [1., 0.], "qpos": [[0.], [1.]]}, "strictly increasing"),
    ({"time": [], "qpos": []}, "nonempty"),
    ({"time": [0., 1.], "qpos": [[0., 0.], [1., 1.]]}, "qpos must"),
    ({"time": [0., 1.], "qpos": [[0.], [float("nan")]]}, "finite"),
    ({"time": [0., 1.], "qpos": [[0.], [1.]], "qvel": [[0., 1.], [0., 1.]]}, "qvel must"),
])
def test_bad_actual_state_logs_rejected(tmp_path, arrays, match):
    source = tmp_path / "states.npz"
    np.savez(source, **arrays)
    with pytest.raises(ValueError, match=match):
        _states(source, 1, 1)


@pytest.mark.parametrize("collision_debug", [False, True])
def test_renderer_uses_actual_qpos_and_writes_real_egl_video(tmp_path, collision_debug):
    pytest.importorskip("mujoco")
    if shutil.which("ffmpeg") is None:
        pytest.skip("ffmpeg is not installed")
    model = tmp_path / "model.xml"
    model.write_text('''<mujoco><visual><global offwidth="320" offheight="240"/></visual>
    <worldbody><light pos="0 0 3"/><geom type="plane" size="2 2 .1" rgba=".5 .5 .5 1"/>
    <body pos="0 0 .3"><joint type="hinge" axis="0 0 1"/>
    <geom type="capsule" fromto="0 0 0 .5 0 0" size=".06" rgba=".1 .3 .9 1"/></body>
    </worldbody></mujoco>''')
    arrays = tmp_path / "rollout.npz"
    np.savez(arrays, time=[0., .1, .2], qpos=[[0.], [.4], [.8]],
             qvel=[[0.], [4.], [4.]], qref=[[9.], [9.], [9.]])
    output = tmp_path / "render"
    result = render_rollout(model, arrays, output, fps=10, width=320, height=240,
                            collision_debug=collision_debug)
    assert result["passed"]
    assert result["frame_count"] == 3
    assert result["frame_state_indices"] == [0, 1, 2]
    assert result["displayed_state_array"] == "qpos"
    assert result["reference_state_displayed"] is False
    assert result["dynamics_resimulated"] is False
    assert result["collision_proxy_overlay"] is collision_debug
    assert result["rendered_geometry_groups"] == ([0, 1, 2, 3, 4] if collision_debug else [0, 1, 2])
    assert (output / "rollout.mp4").stat().st_size > 1000
    from PIL import Image
    image = np.asarray(Image.open(output / "overview.png"))
    assert image.shape == (240, 320, 3)
    assert image.std() > 5
    assert json.loads((output / "render.json").read_text())["passed"]
    with pytest.raises(FileExistsError):
        render_rollout(model, arrays, output, fps=10, width=320, height=240)


def test_failed_render_keeps_diagnostic_metadata(tmp_path):
    pytest.importorskip("mujoco")
    model = tmp_path / "model.xml"
    model.write_text('<mujoco><worldbody/></mujoco>')
    arrays = tmp_path / "rollout.npz"
    np.savez(arrays, time=[0., 1.], qref=[[0.], [0.]])
    output = tmp_path / "failed"
    with pytest.raises(ValueError, match="Missing actual"):
        render_rollout(model, arrays, output)
    result = json.loads((output / "render.json").read_text())
    assert result["passed"] is False
    assert result["error_type"] == "ValueError"
    assert "Missing actual" in result["error"]


def test_endpoint_guides_use_task_goal_not_attained_end_and_do_not_change_state(tmp_path):
    mujoco = pytest.importorskip("mujoco")
    from benchmarks.generic_motion_v1.common import sha
    from benchmarks.generic_motion_v1.simulation.guidance import load_guide

    xml = tmp_path / "model.xml"
    xml.write_text('''<mujoco><worldbody><body pos="0 0 .3">
        <joint type="hinge" axis="0 0 1"/>
        <geom type="capsule" fromto="0 0 0 .5 0 0" size=".06"/>
        </body></worldbody></mujoco>''')
    source = tmp_path / 'attempt'
    source.mkdir()
    (source / 'input.json').write_text(json.dumps(dict(problem_id='test-goal', q_start=[0.], q_goal=[1.])))
    (tmp_path / 'result.json').write_text(json.dumps(dict(source_attempt=str(source),
        source_input_sha256=sha(source / 'input.json'), method='EI_ZO',
        status='execution_failed', checks=dict(endpoint=False))))
    (tmp_path / 'model_metadata.json').write_text(json.dumps(dict(xml_sha256=sha(xml),
        tcp_body_id=1, tcp_local_transform=[[1, 0, 0, .5], [0, 1, 0, 0],
        [0, 0, 1, 0], [0, 0, 0, 1]], joint_ids=[0], qpos_ids=[0])))
    arrays = tmp_path / 'rollout.npz'
    np.savez(arrays, time=[0., .1, .2], qpos=[[0.], [.2], [.4]])
    model = mujoco.MjModel.from_xml_path(str(xml))
    states = _states(arrays, model.nq, model.nv)
    before = states['qpos'].copy()
    guide = load_guide(mujoco, model, states, xml, arrays)
    np.testing.assert_allclose(guide.goal_tcp, [.5*np.cos(1.), .5*np.sin(1.), .3])
    np.testing.assert_allclose(guide.trace[-1], [.5*np.cos(.4), .5*np.sin(.4), .3])
    assert not np.allclose(guide.goal_tcp, guide.trace[-1])
    data = mujoco.MjData(model)
    data.qpos[:] = .2
    mujoco.mj_forward(model, data)
    scene = mujoco.MjvScene(model, maxgeom=500)
    guide.decorate(scene, data)
    assert {'START [1]', 'GOAL [2]'} <= {g.label for g in scene.geoms[:scene.ngeom]}
    assert all(g.objid == -1 for g in scene.geoms[:scene.ngeom])
    np.testing.assert_array_equal(states['qpos'], before)
    np.testing.assert_array_equal(data.qpos, [.2])
    assert model.ngeom == 1 and model.npair == 0
    assert 'END OF RECORDING' in guide.text(.2)[4]
    assert 'CHECK FAILED: endpoint' in guide.text(.2)[6]
    assert 'PAUSED' in guide.text(0, paused=True)[4]
    if shutil.which('ffmpeg'):
        result = render_rollout(xml, arrays, tmp_path / 'guided', fps=10, width=640, height=480)
        assert result['passed'] and result['visual_guidance']['display_only']
        assert result['composition'] == 'actual_motion_with_original_start_goal_pose_panels'
        assert set(result['render_sources_sha256']) == {'render.py', 'guidance.py', 'presentation.py'}
        assert sha(xml) == json.loads((tmp_path / 'model_metadata.json').read_text())['xml_sha256']
    (source / 'input.json').write_text('{}')
    with pytest.raises(ValueError, match='input hash changed'):
        load_guide(mujoco, model, states, xml, arrays)
