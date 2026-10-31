"""Run fixed dev probes in isolated processes, preserving native abort evidence."""
import argparse
from pathlib import Path
import json
import subprocess
import sys
import time
import numpy as np
from benchmarks.generic_motion_v1.common import ROOT, read, sha, write
from benchmarks.generic_motion_v1.runners.native_drm_query import native_library

HERE=Path(__file__).resolve().parent

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build-dir',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    out=args.output.resolve()
    if out.exists():raise FileExistsError(out)
    out.mkdir()
    original=sha(native_library())
    records=[]
    for case,seed in [('dev-sparse_clutter-000-p01',2),
                      ('dev-bottleneck-000-p00',0),('dev-route_choice-002-p00',0)]:
        pair=[]
        for mode in ('native','safe'):
            destination=out/f'{case}_s{seed}_{mode}'
            command=[sys.executable,'-m','benchmarks.generic_motion_v1.reports.baseline_repair_v2.drm_query_probe',
                '--case',case,'--seed',str(seed),'--mode',mode,'--output',str(destination),
                '--build-dir',str(args.build_dir.resolve())]
            record=dict(case=case,seed=seed,mode=mode,argv=command,output=str(destination))
            start=time.perf_counter()
            with (out/f'{case}_s{seed}_{mode}.stdout.log').open('w') as stdout, (out/f'{case}_s{seed}_{mode}.stderr.log').open('w') as stderr:
                result=subprocess.run(command,cwd=ROOT,stdout=stdout,stderr=stderr,timeout=60,check=False)
            record.update(returncode=result.returncode,wall_seconds=time.perf_counter()-start)
            if (destination/'result.json').exists():record['result']=read(destination/'result.json')
            records.append(record);pair.append(record)
            write(out/'processes.json',records)
        if case=='dev-sparse_clutter-000-p01':
            assert pair[0]['returncode']==-6 and 'closest_node >= 0' in (out/f'{case}_s{seed}_native.stderr.log').read_text()
            assert pair[1]['returncode']==0 and pair[1]['result']['status']=='frontend_failed'
            assert pair[1]['result']['layout_after']['collision_nodes']==pair[1]['result']['layout_after']['nodes']==6000
        else:
            assert all(r['returncode']==0 and r['result']['success'] for r in pair)
            native=np.load(Path(pair[0]['output'])/'path.npy',allow_pickle=False)
            safe=np.load(Path(pair[1]['output'])/'path.npy',allow_pickle=False)
            comparison=dict(case=case,seed=seed,shapes_match=native.shape==safe.shape,
                            bitwise_equal=np.array_equal(native,safe),maximum_absolute_error=float(np.max(np.abs(native-safe))))
            write(out/f'{case}_comparison.json',comparison)
            assert comparison['bitwise_equal']
    result=dict(passed=True,probe_cases=3,processes=6,native_abort_reproduced=1,
                safe_exhaustion_frontend_failure=1,normal_success_paths_bitwise_equal=2,
                installed_native_library_unchanged=sha(native_library())==original,
                installed_native_library_sha256=original,records=records)
    assert result['installed_native_library_unchanged']
    write(out/'audit.json',result)
    print(json.dumps({k:v for k,v in result.items() if k!='records'},indent=2))

if __name__=='__main__':main()
