"""CPU regressions for an isolated native host patch; no CUDA queries here."""
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from benchmarks.generic_motion_v1.runners import native_drm_query as safe


def test_patch_is_exactly_the_two_reviewed_failure_exits():
    upstream=safe.SOURCE.read_text()
    patched=safe.patched_source(upstream)
    for old,new in safe.PATCHES:
        assert new in patched and old not in patched
        patched=patched.replace(new,old,1)
    assert patched==upstream


@pytest.mark.parametrize('kind',['missing','duplicated'])
def test_refuse_unreviewed_upstream_changes(kind):
    source=safe.SOURCE.read_text()
    old=safe.PATCHES[0][0]
    source=source.replace(old,'',1) if kind=='missing' else source+old
    with pytest.raises(ValueError,match='exact reviewed'):
        safe.patched_source(source)


def test_build_refuses_existing_output(tmp_path):
    with pytest.raises(FileExistsError,match='Preserve'):
        safe.build(tmp_path)


@pytest.fixture
def fake(monkeypatch):
    planner=SimpleNamespace(num_joints=7,drm=SimpleNamespace(id_to_node_map={1:[0]*7}),
        collision_set=set(),drm_loaded=True,is_initialized=True,
        options=SimpleNamespace(max_number_planning_attempts=20,max_iterations_steering_to_node=10,
        max_nodes_to_expand=10000,online_edge_step_size=.1,voxel_padding=.01,
        max_num_blocked_edges_before_discard_node=10))
    def layout(p):
        return dict(num_joints=p.num_joints,nodes=len(p.drm.id_to_node_map),collision_nodes=len(p.collision_set),
                    loaded=p.drm_loaded,initialized=p.is_initialized,
                    **{k:getattr(p.options,k) for k in ('max_number_planning_attempts',
                       'max_iterations_steering_to_node','max_nodes_to_expand','online_edge_step_size')})
    calls=[]
    def plan(p,start,goal,voxels,radius):
        calls.append((p,start.copy(),goal.copy(),voxels.copy(),radius))
        return True,[start,goal]
    module=SimpleNamespace(layout=layout,plan=plan)
    receipt=dict(binary_sha256='b'*64,native_library_sha256='c'*64,
                 patched_source_sha256='d'*64,upstream_source_sha256='e'*64)
    monkeypatch.setattr(safe,'load',lambda _: (module,receipt))
    return planner,module,receipt,calls


def invoke(fake,**kwargs):
    return safe.query(fake[0],kwargs.pop('start',np.zeros(7)),kwargs.pop('goal',np.ones(7)),
                      kwargs.pop('voxels',[[.1,.2,.3]]),kwargs.pop('radius',.04),build_dir='/synthetic/build',**kwargs)


def test_normal_query_preserves_endpoints_radius_options_and_provenance(fake,tmp_path):
    before=fake[0].options.__dict__.copy()
    result,path,evidence=invoke(fake,out=tmp_path)
    assert result and np.array_equal(path,np.array([np.zeros(7),np.ones(7)]))
    assert len(fake[3])==1 and np.array_equal(fake[3][0][1],np.zeros(7))
    assert np.array_equal(fake[3][0][2],np.ones(7)) and fake[3][0][3].shape==(3,1)
    assert fake[3][0][4]==.04 and fake[0].options.__dict__==before
    assert evidence['query_success'] and not evidence['endpoint_change']
    assert json.loads((tmp_path/'query.json').read_text())==evidence
    assert json.loads((tmp_path/'build_receipt.json').read_text())==fake[2]
    assert evidence['status']=='returned' and evidence['elapsed_seconds']>=0


def test_evidence_saved_before_native_exception(fake,tmp_path):
    def fail(*args):
        pending=json.loads((tmp_path/'query.json').read_text())
        assert pending['status']=='running' and (tmp_path/'build_receipt.json').is_file()
        raise RuntimeError('No collision-free path exists in roadmap.')
    fake[1].plan=fail
    with pytest.raises(RuntimeError,match='No collision-free path'):
        invoke(fake,out=tmp_path)
    record=json.loads((tmp_path/'query.json').read_text())
    assert record['status']=='native_exception' and not record['query_success']
    assert record['exception_type']=='RuntimeError' and record['elapsed_seconds']>=0
    assert record['layout_after']==record['layout_before']


def test_returned_false_is_preserved_not_promoted(fake,tmp_path):
    fake[1].plan=lambda *args:(False,[])
    result,path,evidence=invoke(fake,out=tmp_path)
    assert not result and path.size==0 and not evidence['query_success']


def test_preserve_existing_query_receipt(fake,tmp_path):
    (tmp_path/'query.json').write_text('{"old":true}')
    with pytest.raises(FileExistsError):invoke(fake,out=tmp_path)
    assert not fake[3] and (tmp_path/'query.json').read_text()=='{"old":true}'


@pytest.mark.parametrize('change',[{'start':[0]*6},{'goal':[float('nan')]*7},
                                  {'voxels':[[0,1]]},{'voxels':[[0,1,float('inf')]]},
                                  {'radius':-1},{'radius':float('nan')}])
def test_invalid_inputs_never_reach_native(fake,change):
    with pytest.raises(ValueError,match='Malformed'):invoke(fake,**change)
    assert not fake[3]


@pytest.mark.parametrize('name,value',[('max_number_planning_attempts',1),
    ('max_num_blocked_edges_before_discard_node',1),('max_iterations_steering_to_node',0),
    ('max_nodes_to_expand',0),('online_edge_step_size',0.),('online_edge_step_size',float('inf')),
    ('voxel_padding',-1.),('voxel_padding',float('inf'))])
def test_native_assert_preconditions_checked(fake,name,value):
    setattr(fake[0].options,name,value)
    with pytest.raises(ValueError,match='preconditions'):invoke(fake)
    assert not fake[3]


def test_unloaded_planner_never_reaches_native(fake):
    fake[0].drm_loaded=False
    with pytest.raises(ValueError,match='initialized and loaded'):invoke(fake)
    assert not fake[3]


def test_layout_disagreement_is_fatal(fake):
    original=fake[1].layout
    fake[1].layout=lambda p:dict(original(p),nodes=99)
    with pytest.raises(RuntimeError,match='ABI/layout'):invoke(fake)
    assert not fake[3]


def test_native_closest_empty_and_normal_nearest_cpu():
    build=safe.BASE/'reports/baseline_repair_v2/drm_safe_build_v2'
    if not (build/'build.json').is_file():pytest.skip('Explicit isolated sidecar build not present')
    # Constructing/parsing a model and DrmPlanner makes no CUDA query. Do NOT
    # LoadRoadmap/BuildCollisionSet/Plan in this CPU test.
    from benchmarks.generic_motion_v1.model import Model
    import csdecomp as csd
    model=Model()
    planner=csd.DrmPlanner(model.csd_plant,csd.DrmPlannerOptions(),True)
    module,_=safe.load(build)
    assert safe.layout_guard(module,planner)['nodes']==0
    assert module.closest_node(planner,np.zeros(7))==(-1,None)
    planner.drm.id_to_node_map={2:np.ones(7),1:np.zeros(7)}
    native_id,point=module.closest_node(planner,np.full(7,.01))
    assert native_id==1 and np.array_equal(point,np.zeros(7))


@pytest.fixture
def relocated(monkeypatch,tmp_path):
    checkout=tmp_path/'moved_checkout';checkout.mkdir()
    build=checkout/'sidecar';build.mkdir()
    source=checkout/'source.cpp';source.write_text('unchanged original source')
    binding=checkout/'binding.cpp';binding.write_text('unchanged original binding')
    native=tmp_path/'native.so';native.write_bytes(b'unchanged installed CUDA')
    binary=build/'_drm_safe_query.so';binary.write_bytes(b'unchanged compiled sidecar')
    generated=build/'generated_binding.cpp';generated.write_text('generated binding')
    receipt=dict(status='ready',binary='/old/machine/sidecar/_drm_safe_query.so',
        native_library_sha256=safe.sha(native),binary_sha256=safe.sha(binary),
        upstream_source_sha256=safe.sha(source),binding_sha256=safe.sha(binding),source_files={},
        compiled_binding='/old/machine/sidecar/generated_binding.cpp',compiled_binding_sha256=safe.sha(generated))
    (build/'build.json').write_text(json.dumps(receipt))
    monkeypatch.setattr(safe,'SOURCE',source);monkeypatch.setattr(safe,'BINDING',binding)
    monkeypatch.setattr(safe,'native_library',lambda:native)
    module=SimpleNamespace(__file__=str(binary))
    monkeypatch.setitem(sys.modules,'_drm_safe_query',module)
    return build,receipt,module


def test_copied_build_uses_current_directory_without_rewriting_receipt(relocated):
    folder,receipt,module=relocated
    before=(folder/'build.json').read_bytes()
    loaded,evidence=safe.load(folder)
    assert loaded is module and evidence==receipt
    assert (folder/'build.json').read_bytes()==before


@pytest.mark.parametrize('name',['_drm_safe_query.so','generated_binding.cpp'])
def test_copied_build_tampering_still_rejected(relocated,name):
    folder,_,_=relocated
    (folder/name).write_bytes(b'tampered')
    with pytest.raises(ValueError,match='changed'):safe.load(folder)


def test_copied_legacy_build_resolves_current_original_binding(relocated):
    folder,receipt,_=relocated
    receipt['compiled_binding']='/old/machine/native/binding.cpp'
    receipt['compiled_binding_sha256']=receipt['binding_sha256']
    (folder/'build.json').write_text(json.dumps(receipt))
    assert safe.load(folder)[1]==receipt


def test_moved_build_does_not_reuse_an_old_loaded_binary(relocated):
    folder,_,module=relocated
    module.__file__='/old/machine/sidecar/_drm_safe_query.so'
    with pytest.raises(ValueError,match='different safe DRM'):safe.load(folder)


def test_portable_build_loads_only_hash_verified_current_native(relocated,monkeypatch):
    folder,receipt,module=relocated
    receipt['native_linkage']='verified_runtime_global'
    (folder/'build.json').write_text(json.dumps(receipt))
    calls=[];handle=object()
    def native_load(path,mode):
        calls.append((path,mode));return handle
    monkeypatch.setattr(safe.ctypes,'CDLL',native_load)
    safe.load(folder)
    assert calls==[(str(safe.native_library()),safe.ctypes.RTLD_GLOBAL)]
    assert module._verified_native_library_handle is handle
    safe.native_library().write_bytes(b'changed native')
    calls.clear()
    with pytest.raises(ValueError,match='changed'):safe.load(folder)
    assert not calls
