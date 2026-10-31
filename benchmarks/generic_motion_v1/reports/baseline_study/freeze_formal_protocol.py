"""Freeze the user's approved v1 experiment before any test planner is run.

This records the actual 2026-10-28 conversation decision; it is not a generic
authorization bypass. Refuses replacement and does not launch planners.
"""
from collections import Counter
from datetime import datetime, timezone
from importlib import metadata as distribution_metadata
from pathlib import Path

from ...common import BASE, ROOT, cases, code_inventory, head, read, sha, write
from ...roadmap import load_config
from ...study_protocol import (authorize, current_model_identity, method_pairs,
                              _dataset_files, _roadmap_files)

OUT = Path(__file__).resolve().parent


def main():
    target = OUT / 'formal_protocol_v1.json'
    if target.exists():
        raise FileExistsError('The formal protocol is frozen; never overwrite it')
    manifest = BASE / 'manifests/test.jsonl'
    data = cases(manifest)
    assert len(data) == 420 and sum(c['validation_status']=='valid_input' for c in data)==395
    assert sha(manifest)=='9fcbe5ccae28d8681586b1ac6c9986f51f7cc7b5cdc19352c101910e3b6ed03b'
    audit = OUT / 'test_full_reaudit.json'
    evaluation = load_config(BASE / 'configs/rss2025_eizo.json')
    selected = [c['problem_id'] for c in data if c['family']!='open_control'
                and int(c['problem_id'].rsplit('-p',1)[1]) == int(c['scene_id'].rsplit('-',1)[1]) % 5]
    assert len(selected)==80
    selected_cases=[c for c in data if c['problem_id'] in selected]
    assert len({c['scene_id'] for c in selected_cases})==80
    assert set(Counter(c['family'] for c in selected_cases).values())=={20}
    assert sum(c['validation_status']=='valid_input' for c in selected_cases)==75
    pairs=method_pairs(['EI_ZO','LGCS','NTO_PAPER'],['minimum_distance','minimum_time'])
    subset_path=OUT/'formal_size_effect_ids.json'
    if subset_path.exists():
        raise FileExistsError('Prespecified subset already exists')
    write(subset_path,selected)
    blocks={}
    dependencies={}
    for package in ('csdecomp','drake','scsplanning','pybezier','clarabel','gurobipy'):
        distribution=distribution_metadata.distribution(package)
        for item in distribution.files or []:
            path=Path(distribution.locate_file(item)).resolve()
            if path.is_file() and (path.suffix=='.py' or '.so' in path.name):
                dependencies[str(path)]=sha(path)
    assert dependencies
    requests=[]
    # Small immutable run manifests keep individual evidence files practical
    # for Git and let completed blocks be independently checked while later
    # blocks run. This partitions the matrix; it never changes its population.
    for seed in range(3):
        for family in ('sparse_clutter','dense_clutter','bottleneck','route_choice','open_control'):
            requests.append((f'main_s{seed:03d}_{family}','main',6000,seed,
                             [c['problem_id'] for c in data if c['family']==family]))
    for nodes in (3000,9000,12000):
        for seed in range(3):
            requests.append((f'size_n{nodes}_s{seed:03d}','size_effect',nodes,seed,selected))
    for name,phase,nodes,seed,ids in requests:
        directories=[BASE/f'roadmaps/rss2025_n{nodes}_s{seed:03d}']
        blocks[name]=dict(selection='fixed_subset',phase=phase,problem_count=len(ids),problem_ids=ids,
            roadmap_dirs=list(map(str,directories)),roadmap_files=_roadmap_files(directories),
            method_pairs=pairs,evaluation_config=evaluation,timeout_seconds=60.,
            requested_node_budgets=[nodes],roadmap_seeds=[seed],
            planned_jobs=len(ids)*len(directories)*len(pairs))
    assert len(blocks)==24 and sum(b['planned_jobs'] for b in blocks.values())==9900
    assert sum(b['planned_jobs'] for b in blocks.values() if b['phase']=='main')==6300
    protocol=dict(schema_version=1,state='approved',
        title='RSS 2025 paper-method baselines on frozen Kinova generic v1',
        frozen_utc=datetime.now(timezone.utc).isoformat(),source_code_commit=head(),
        runtime_source_hashes=code_inventory(),
        external_dependency_files=dependencies,
        external_dependency_scope='Installed code/native library files for CSDecomp, Drake, SCSPlanning, pybezier, Clarabel and gurobipy; coordinator checks before and after each block. Other package versions and hardware are recorded separately.',
        authorization=dict(text='保留 v1，评测395合法题（推荐）',
            source='User response in this Codex task on 2026-10-28 to request_user_input_async call_8c1443d3b7da4ea79f2556eb11aebf63 question 0',
            question='全量复核已确认：395 题输入合法，25 条是已保存的生成失败；模型交叉检查、哈希和跨集合检查未发现错误。按原先“冻结后不改题”的要求，我需要确定正式实验的数据口径：使用原 v1 的 395 道合法题，还是另建包含 420 道合法题的新版本？',
            prior_request='重点是论文的方法呀！！我现在先需要建立一些baseline 确定好评价指标！并充分完整的做完实验！'),
        manifest=dict(path=str(manifest),sha256=sha(manifest)),
        input_audit=dict(path=str(audit),sha256=sha(audit)),
        model=current_model_identity(),data_files=_dataset_files(manifest,data),blocks=blocks,
        metrics_contract=dict(path=str(BASE/'BASELINE_PROTOCOL.md'),sha256=sha(BASE/'BASELINE_PROTOCOL.md')),
        dataset_accounting=dict(registered=420,legal=395,input_generation_failures=25,
            legal_obstacle_tasks=375,legal_control_tasks=20,
            legal_random_obstacle_worlds=75,physical_control_worlds=1,
            invalid_inputs_are_not_planner_failures=True),
        size_selection=dict(rule='For each of the 80 obstacle scene IDs, endpoint index = scene index modulo 5. No optimizer outcomes used.',
            registered=80,legal=75,invalid=5,
            problem_ids_file=dict(path=str(subset_path),sha256=sha(subset_path)),
            distance_bins=dict(Counter(c['endpoint_sampling']['distance_bin'] for c in selected_cases)),
            reuse_6000_node_observations_from_main=True),
        execution=dict(gpu_workers=1,total_distinct_job_slots=9900,
            total_legal_planner_jobs=9300,input_invalid_job_slots=600,
            no_test_tuning=True,no_success_only_reruns=True,
            reuse_roadmaps=True,offline_time_reported_separately=True),
        limitations=['Kinova adaptation, not exact original FR3 reproduction',
            'NTO and LGCS are declared mathematical reconstructions',
            'LGCS restricted Gurobi license can reject large models',
            'NTO Nc=150 parameter yields 149 interior collision bindings',
            'Finite collision validation is numerical evidence, not a continuous proof',
            'Conservative voxel obstacles can remove original-geometry free space'])
    write(target,protocol)
    try:
        for name,block in blocks.items():
            authorize(target,name,manifest,block['roadmap_dirs'],evaluation,60.,pairs)
    except Exception as exc:
        write(OUT/'formal_protocol_freeze_failure.json',dict(error=repr(exc),protocol_sha256=sha(target)))
        raise
    write(OUT/'formal_protocol_freeze_receipt.json',dict(protocol=str(target),sha256=sha(target),
        source_commit=head(),blocks={n:b['planned_jobs'] for n,b in blocks.items()},
        test_planner_jobs_executed=0))
    print(f'Frozen {target}: {sha(target)}; no planner executed',flush=True)


if __name__=='__main__':
    main()
