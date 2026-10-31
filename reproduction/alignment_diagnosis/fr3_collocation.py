"""Read-only Drake audit of all saved FR3 NTO collision constraints.

No optimization, roadmap construction, native collision query, or GPU call.
The zero-threshold signed-distance query determines negative distances only;
when it returns none, the positive clearance is unknown, not zero.
"""
import argparse
from collections import Counter
import hashlib
import importlib.metadata
from pathlib import Path
import time
import traceback

import numpy as np
from pydrake.all import BsplineBasis, BsplineTrajectory, MinimumDistanceLowerBoundConstraint

from reproduction.common import ROOT, atomic, read
from reproduction.paper_constructed.model import drake_fr3


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def negative_distances(parts, configurations):
    """Return negative minima, with NaN explicitly meaning no negative pair."""
    _, _, plant, context, scene_graph, inspector, _ = parts
    diagram, root_context = parts[:2]
    del diagram
    query = scene_graph.get_query_output_port().Eval(scene_graph.GetMyContextFromRoot(root_context))
    minima = np.full(len(configurations), np.nan)
    pair_counts = np.zeros(len(configurations), dtype=np.int32)
    names = np.full(len(configurations), '', dtype='<U160')
    for i, q in enumerate(configurations):
        plant.SetPositions(context, q)
        # Re-evaluate the output after position mutation; never cache distances.
        query = scene_graph.get_query_output_port().Eval(scene_graph.GetMyContextFromRoot(root_context))
        distances = query.ComputeSignedDistancePairwiseClosestPoints(0.0)
        negative = [p for p in distances if p.distance < 0.0]
        pair_counts[i] = len(negative)
        if negative:
            closest = min(negative, key=lambda p: p.distance)
            minima[i] = closest.distance
            names[i] = inspector.GetName(closest.id_A) + ' | ' + inspector.GetName(closest.id_B)
    return dict(min_negative_distance=minima, negative_pair_counts=pair_counts,
                nonnegative=~np.isfinite(minima), deepest_pair_names=names)


def minimum_negative(a):
    values = np.asarray(a)[np.isfinite(a)]
    return float(values.min()) if len(values) else None


def stats(values):
    a = np.array([v for v in values if v is not None], dtype=float)
    if not len(a):
        return dict(count=0)
    return dict(count=len(a), minimum=float(a.min()), median=float(np.median(a)),
                mean=float(a.mean()), maximum=float(a.max()))


def summarize(records, missing, errors, identities, verified, elapsed):
    groups = {}
    for method in ['NTO_distance', 'NTO_time']:
        rows = [r for r in records if r['method'] == method]
        subsets = {
            'all_candidates': rows,
            'solver_success': [r for r in rows if r['solver_success']],
            'solver_failed': [r for r in rows if not r['solver_success']],
            'strict_success': [r for r in rows if r['strict_success']],
            'solver_success_validation_failed': [r for r in rows if r['solver_success'] and not r['saved_validation_passed']],
        }
        groups[method] = {}
        for key, selected in subsets.items():
            groups[method][key] = dict(
                candidates=len(selected),
                categories=dict(Counter(r['geometry_category'] for r in selected)),
                smooth_residual_max=stats([r['max_smooth_constraint_residual'] for r in selected]),
                collocation_penetration_depth_m=stats([-r['collocation_min_negative_distance']
                                                      for r in selected if r['collocation_min_negative_distance'] is not None]),
                saved_dense_witness_penetration_depth_m=stats([-r['dense_witness_min_negative_distance']
                                                             for r in selected if r['dense_witness_min_negative_distance'] is not None]),
                candidates_residual_above_diagnostic_1e_6=sum(r['max_smooth_constraint_residual'] > 1e-6 for r in selected),
                dense_witness_deeper_than_collocation=sum(r['dense_witness_deeper_than_collocation'] for r in selected),
                saved_collision_free_but_collocation_negative=sum(r['saved_collision_free'] and r['collocation_negative_points'] > 0 for r in selected),
            )
    return dict(candidates=len(records), collocation_points=sum(r['collocation_points'] for r in records),
                saved_dense_witness_points=sum(r['dense_witness_points'] for r in records),
                missing_candidate_combinations=len(missing), errors=len(errors),
                input_files=len(identities), input_reverification_passed=verified,
                elapsed_seconds=elapsed, groups=groups)


def make_report(output, summary):
    lines = ['# FR3 NTO 离散碰撞约束定位', '',
             '本报告只复核 v5 冻结候选。没有重新优化、规划或调用 GPU。'
             '采用原 FR3 球/盒模型与过滤规则，实际计算每条候选的全部 150 个碰撞约束点，并复查保存的密集验证碰撞见证。', '',
             f"完成 {summary['candidates']} 条候选、{summary['collocation_points']} 个约束点、{summary['saved_dense_witness_points']} 个保存见证点；"
             f"缺失 {summary['missing_candidate_combinations']} 个案例/方法组合（原因单列），错误 {summary['errors']}。"
             f"输入哈希复核：{summary['input_reverification_passed']}。", '',
             '| 方法 / 子集 | 条数 | 约束点有负距离 | 约束点非负、保存见证有穿透 | 无保存或约束点碰撞 |',
             '|---|---:|---:|---:|---:|']
    for method, groups in summary['groups'].items():
        for subset in ['solver_success', 'solver_failed', 'solver_success_validation_failed']:
            data = groups[subset]
            categories = data['categories']
            lines.append(f"| {method} / {subset} | {data['candidates']} | "
                         f"{categories.get('collocation_penetration', 0)} | "
                         f"{categories.get('between_collocation_penetration_at_saved_dense_witness', 0)} | "
                         f"{categories.get('no_saved_or_collocation_collision', 0)} |")
    lines += ['', '若存在“保存碰撞标志、当前见证未重现负距离”类别，会独立列在 JSON 中，不并入无碰撞。', '',
              '## 残差与物理距离必须分开', '',
              '`MinimumDistanceLowerBoundConstraint` 的标量上界残差是无量纲平滑约束量。'
              '`1e-6` 只作为诊断分组线，既不是 1 µm，也不是对 SNOPT/IPOPT 内部缩放后停止准则的等价检验。'
              '报告保存原连续残差，不把略超过该线自动解释为求解器错误。', '']
    for method, groups in summary['groups'].items():
        for subset in ['solver_success', 'solver_success_validation_failed', 'solver_failed']:
            data = groups[subset]
            lines.append(f"- {method} / {subset}：最大平滑残差统计 {data['smooth_residual_max']}；"
                         f"约束点穿透深度（m）{data['collocation_penetration_depth_m']}；"
                         f"保存密集见证穿透深度（m）{data['saved_dense_witness_penetration_depth_m']}。")
    lines += ['', '## 判定范围', '',
              '- 先独立记录求解成功/失败，再标记几何碰撞位置；求解失败候选即使无负距离也不计成功。',
              '- 约束点全部非负、保存密集见证有负距离，支持“点间碰撞”定位。约束点已经有负距离时，可能还同时存在更深的点间碰撞；这两者并不互斥。',
              '- `ComputeSignedDistancePairwiseClosestPoints(0.0)` 只用于查负距离。未返回负对时保存 `nonnegative=true` 和未知距离 NaN/JSON null，不宣称真实最小正距离等于零。',
              '- 密集见证仅复查原验证器保存的点，没有额外搜索连续轨迹最深穿透；结论不是连续安全证明，也不是官方网格几何的审计。',
              '- 原始约束点配置、残差、负距离、负碰撞对数量与最深对名称，以及保存见证的复查数组见 `samples/`；完整逐候选记录、输入哈希和源码快照一并保存。', '',
              '执行命令：', '', '```bash',
              'bash local/python.sh -m reproduction.alignment_diagnosis.fr3_collocation --output reproduction/reports/alignment_diagnosis/fr3_collocation',
              '```', '', '为保留历史，重复运行必须选择新的输出目录。']
    (output / 'report.md').write_text('\n'.join(lines) + '\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    run = Path(read(ROOT / 'reproduction/reports/paper_constructed/run_index.json')['fr3']['run_dir'])
    manifest = read(run / 'manifest.json')
    identities = {'manifest.json': sha(run / 'manifest.json')}
    source_names = ['reproduction/paper_constructed/model.py', 'reproduction/paper_constructed/assets/fr3_33.json',
                    'reproduction/paper_constructed/config.json', 'reproduction/common.py']
    source_hashes = {name: sha(ROOT / name) for name in source_names}
    source_matches = {name: source_hashes[name] == manifest['source_files'].get(name) for name in source_names}
    if not all(source_matches.values()):
        raise RuntimeError(f'Model source differs from frozen run: {source_matches}')
    source_hashes[str(Path(__file__).relative_to(ROOT))] = sha(__file__)
    for name in source_hashes:
        dest = output / 'source_snapshot' / name
        dest.parent.mkdir(parents=True, exist_ok=True)
        dest.write_bytes((ROOT / name).read_bytes())
    atomic(output / 'scope.json', dict(input_run=run, selection='all 400 cases, both NTO objectives including failed solvers',
           kind='posthoc CPU Drake only, no optimization, no GPU, no reselection',
           distance_query_threshold=0.0, nonnegative_distance_representation='NaN in arrays and null in JSON; positive magnitude unknown',
           residual_diagnostic_threshold=1e-6, residual_is_not_physical_distance=True,
           source_hashes=source_hashes, source_matches_frozen_run=source_matches,
           runtime_drake=importlib.metadata.version('drake'), runtime_numpy=np.__version__))
    started = time.perf_counter()
    environments, constraints, records, missing, errors = {}, {}, [], [], []
    for case_index, case in enumerate(manifest['cases']):
        case_dir = run / 'cases' / case['id']
        identities[str((case_dir / 'case.json').relative_to(run))] = sha(case_dir / 'case.json')
        for method in ['NTO_distance', 'NTO_time']:
            folder = case_dir / method
            paths = [folder / name for name in ['optimization_00.json', 'candidate_00.npz', 'validation.json', 'result.json']]
            if not all(path.exists() for path in paths):
                missing.append(dict(case=case['id'], method=method, saved_case_status=read(case_dir / 'case.json')['status'],
                                    absent_files=[str(p.relative_to(run)) for p in paths if not p.exists()]))
                continue
            try:
                for path in paths:
                    identities[str(path.relative_to(run))] = sha(path)
                seed = case['environment_seed']
                if seed not in environments:
                    scene_path = run / 'scenes' / f'e{seed:02d}' / 'scene.json'
                    identities[str(scene_path.relative_to(run))] = sha(scene_path)
                    parts = drake_fr3(manifest['config'], read(scene_path)['obstacles'])
                    environments[seed] = parts
                    constraints[seed] = MinimumDistanceLowerBoundConstraint(
                        parts[2], manifest['config']['nto_collision_bound'], parts[3],
                        influence_distance_offset=manifest['config']['nto_influence_distance'])
                parts, constraint = environments[seed], constraints[seed]
                meta, valid, result = read(paths[0]), read(paths[2]), read(paths[3])
                with np.load(paths[1]) as saved:
                    curve = BsplineTrajectory(BsplineBasis(meta['spline_order'], saved['knots'].tolist()),
                                             [q[:, None] for q in saved['controls']])
                locations = np.array(meta['collocation_locations'])
                q = curve.vector_values(curve.start_time() + locations * meta['duration']).T
                evaluations = np.array([constraint.Eval(x)[0] for x in q])
                residual = evaluations - float(constraint.upper_bound()[0])
                collocation = negative_distances(parts, q)
                dense_q = np.array(valid['collision_points'], dtype=float).reshape(-1, 7)
                dense = negative_distances(parts, dense_q)
                coll_min = minimum_negative(collocation['min_negative_distance'])
                dense_min = minimum_negative(dense['min_negative_distance'])
                if coll_min is not None:
                    category = 'collocation_penetration'
                elif dense_min is not None:
                    category = 'between_collocation_penetration_at_saved_dense_witness'
                elif not valid['collision_free']:
                    category = 'saved_collision_flag_without_negative_drake_witness'
                else:
                    category = 'no_saved_or_collocation_collision'
                record = dict(case=case['id'], environment_seed=seed, method=method,
                              solver_success=meta['solver_success'], solver_status=meta['solver_status'],
                              strict_success=result['status'] == 'success_numerical', saved_status=result['status'],
                              saved_validation_passed=valid['passed'], saved_collision_free=valid['collision_free'],
                              saved_native_cuda_collision_free=valid['native_cuda_collision_free'],
                              saved_independent_collision_free=valid['independent_collision_free'],
                              collocation_points=len(q), collocation_negative_points=int((~collocation['nonnegative']).sum()),
                              collocation_min_negative_distance=coll_min, max_smooth_constraint_residual=float(residual.max()),
                              smooth_residual_diagnostic_above_1e_6=bool(residual.max() > 1e-6),
                              dense_witness_points=len(dense_q), dense_witness_negative_points=int((~dense['nonnegative']).sum()),
                              dense_witness_min_negative_distance=dense_min,
                              dense_witness_deeper_than_collocation=bool(dense_min is not None and (coll_min is None or dense_min < coll_min - 1e-12)),
                              geometry_category=category, control_points=meta['control_points'])
                records.append(record)
                dest = output / 'samples' / case['id']
                dest.mkdir(parents=True, exist_ok=True)
                np.savez_compressed(dest / (method + '.npz'), s=locations, q=q, evaluator=evaluations, residual=residual,
                                    **{'collocation_' + k: v for k, v in collocation.items()}, dense_witness_q=dense_q,
                                    **{'dense_witness_' + k: v for k, v in dense.items()})
            except Exception as error:
                errors.append(dict(case=case['id'], method=method, error=repr(error), traceback=traceback.format_exc()))
                atomic(output / 'errors.json', errors)
        if (case_index + 1) % 10 == 0:
            atomic(output / 'records.json', records)
            atomic(output / 'progress.json', dict(cases_processed=case_index + 1, candidates=len(records),
                                                 missing=len(missing), errors=len(errors), seconds=time.perf_counter() - started))
            print(f'{case_index + 1}/400 cases, {len(records)} candidates, {len(errors)} errors', flush=True)
    verified = all(sha(run / name) == expected for name, expected in identities.items())
    source_verified = all(sha(ROOT / name) == expected for name, expected in source_hashes.items())
    atomic(output / 'input_hashes.json', identities)
    atomic(output / 'records.json', records)
    atomic(output / 'missing.json', missing)
    atomic(output / 'errors.json', errors)
    summary = summarize(records, missing, errors, identities, verified, time.perf_counter() - started)
    summary['source_reverification_passed'] = source_verified
    atomic(output / 'summary.json', summary)
    make_report(output, summary)
    print(summary, flush=True)
    if errors or not verified or not source_verified:
        raise RuntimeError('Audit has unresolved errors or source/input changed; see saved records')


if __name__ == '__main__':
    main()
