"""Build the specification's nine missing maps serially, without test queries."""
from datetime import datetime, timezone
import json
from pathlib import Path
import subprocess
import sys
import time

from ...common import ROOT, BASE, read, sha, write

OUT = Path(__file__).resolve().parent


def main():
    config = BASE / 'configs/rss2025_eizo.json'
    inventory = []
    for nodes in (3000, 9000, 12000):
        for seed in (0, 1, 2):
            target = BASE / 'roadmaps' / f'rss2025_n{nodes}_s{seed:03d}'
            command = [sys.executable, '-m', 'benchmarks.generic_motion_v1',
                       'build-roadmap', '--nodes', str(nodes), '--seed', str(seed),
                       '--config', str(config), '--output', str(target)]
            if target.exists():
                raise FileExistsError(f'Existing map must not be overwritten: {target}')
            started = datetime.now(timezone.utc).isoformat()
            tic = time.perf_counter()
            log = OUT / f'build_n{nodes}_s{seed:03d}.log'
            with log.open('w') as stream:
                result = subprocess.run(command, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT)
            metadata = read(target / 'metadata.json') if (target / 'metadata.json').exists() else {}
            record = dict(command=command, started_utc=started, returncode=result.returncode,
                          process_wall_seconds=time.perf_counter() - tic, metadata=metadata,
                          config_sha256=sha(config), log=str(log.relative_to(BASE)),
                          test_queries_executed=0, gpu_builds_serial=True)
            inventory.append(record)
            write(OUT / 'scaling_map_builds.json', dict(builds=inventory, test_queries_executed=0))
            print(json.dumps(dict(nodes=nodes, seed=seed, status=metadata.get('status'),
                                  returncode=result.returncode)), flush=True)
            if result.returncode or metadata.get('status') != 'ready':
                raise RuntimeError(f'Map build failed; evidence preserved: {log}')


if __name__ == '__main__':
    main()
