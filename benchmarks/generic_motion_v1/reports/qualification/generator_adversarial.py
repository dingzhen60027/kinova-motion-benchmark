"""CPU-only adversarial qualification; never touches frozen benchmark inputs."""
import copy
import importlib
import json
from pathlib import Path
import tempfile

import numpy as np

from benchmarks.generic_motion_v1.generator.endpoints import generate_endpoints
from benchmarks.generic_motion_v1.generator.manifest import DEFAULT_CONFIG, file_sha256
from benchmarks.generic_motion_v1.generator.scene import digest
from benchmarks.generic_motion_v1.tests.test_generator import FakeModel

manifest = importlib.import_module('benchmarks.generic_motion_v1.generator.manifest')
audit_module = importlib.import_module('benchmarks.generic_motion_v1.generator.audit')


def run():
    manifest._require_model_audit = lambda model: {'workspace': {
        'moving_collision_geometry_min': [-2, -2, -2],
        'moving_collision_geometry_max': [2, 2, 2]}}
    audit_module._require_model_audit = lambda model: {}
    results = {}
    for name in ('clean', 'deleted_family', 'missing_raw_arrays', 'changed_seeds'):
        with tempfile.TemporaryDirectory(prefix='csdecomp-generator-adversarial-') as temp:
            root = Path(temp)
            generated = manifest.generate('smoke', 12, root, model=FakeModel(), source_code_commit='f' * 40)
            path = Path(generated['manifest_path'])
            rows = [json.loads(line) for line in path.read_text().splitlines()]
            receipt_path = root / 'receipts/smoke_generation.json'
            receipt = json.loads(receipt_path.read_text())
            if name == 'deleted_family':
                rows = [r for r in rows if r['family'] != 'route_choice']
            if name == 'missing_raw_arrays':
                relative = rows[0]['input_arrays_file']
                (root / relative).unlink()
                del receipt['files_sha256'][relative]
            if name == 'changed_seeds':
                rows[0]['endpoint_seed'] += 1
                rows[0]['paired_seeds'][0]['method_seed'] += 1
                row = dict(rows[0])
                row.pop('problem_sha256')
                rows[0]['problem_sha256'] = digest(row)
            path.write_text(''.join(json.dumps(r, sort_keys=True, separators=(',', ':')) + '\n' for r in rows))
            receipt['manifest_sha256'] = file_sha256(path)
            receipt['files_sha256']['manifests/smoke.jsonl'] = file_sha256(path)
            receipt_path.write_text(json.dumps(receipt))
            try:
                checked = audit_module.audit(path, model=FakeModel())
                results[name] = {'accepted': checked['passed'], 'cases': checked.get('problem_count'),
                                 'errors': checked['errors']}
            except (ValueError, RuntimeError, KeyError, OSError) as exc:
                results[name] = {'accepted': False, 'exception': f'{type(exc).__name__}: {exc}'}
    config = json.loads(DEFAULT_CONFIG.read_text())
    config['endpoint_joint_margin_rad'] = -0.5
    try:
        row, _ = generate_endpoints(FakeModel(), 0, 'medium', config)
        states = np.asarray([row['q_start'], row['q_goal']])
        results['negative_margin'] = {'accepted': row['validation_status'] == 'valid_input',
            'true_joint_limit_violations': int(np.sum((states < FakeModel.lower) | (states > FakeModel.upper)))}
    except (ValueError, RuntimeError) as exc:
        results['negative_margin'] = {'accepted': False, 'exception': f'{type(exc).__name__}: {exc}'}
    return results


if __name__ == '__main__':
    print(json.dumps(run(), indent=2))
