"""Read-only reproducible address, package-linkage and owner-lifetime audit.

No CUDA calls, installations, source mutations, or historical result changes.
Fresh source downloads are identified by URL and SHA; the current installed
binary cannot retrospectively prove the original crashing binary identity.
"""
from __future__ import annotations
import base64,csv,datetime,gc,hashlib,importlib.metadata as md,io,json,os,re,subprocess,sys,urllib.request,weakref
from pathlib import Path

ROOT=Path(__file__).resolve().parents[5]
OUT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT))
LIB=ROOT/'.conda/envs/csdecomp/lib/python3.10/site-packages/pydrake/lib/libdrake.so'
OFFSET=0x279cc31

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def command(argv):
    result=subprocess.run(argv,cwd=ROOT,text=True,capture_output=True)
    return dict(argv=argv,exit_code=result.returncode,stdout=result.stdout,stderr=result.stderr)
def remote(url):
    with urllib.request.urlopen(url,timeout=30) as response:
        value=response.read()
        return dict(url=url,status=response.status,sha256=hashlib.sha256(value).hexdigest(),bytes=len(value),text=value.decode())

record=dict(scope='CPU-only read-only forensic localization and route verification; not a benchmark score',
    collected_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),git_commit=command(['git','rev-parse','HEAD'])['stdout'].strip(),
    original_kernel_line='Nov 03 15:26:21 eric-server kernel: python[251095]: segfault at 6135d35e8930 ip 00007dac1f19cc31 sp 00007ffdfe0beaf0 error 6 in libdrake.so[7dac1ca00000+4d0e000] likely on CPU 10 (core 20, socket 0)',
    current_library=str(LIB.relative_to(ROOT)),current_library_sha256=sha(LIB),relative_ip=hex(OFFSET),
    original_binary_identity_sealed=False,no_original_core_or_call_stack=True)
record['collection']=[command(['readelf','-n',str(LIB)]),
    command(['objdump','-d','--start-address=0x279b86b','--stop-address=0x279cdfe',str(LIB)]),
    command(['ldd',str(LIB)])]
process=subprocess.Popen(['readelf','--debug-dump=frames',str(LIB)],stdout=subprocess.PIPE,text=True)
containing=[]
for line in process.stdout:
    match=re.search(r'pc=([0-9a-f]+)\.\.([0-9a-f]+)',line)
    if match and int(match[1],16)<=OFFSET<int(match[2],16):
        containing.append(line.strip())
process.wait()
record['eh_frame_containing_range']=containing
binary=LIB.read_bytes()
record['within_containing_function_rodata_references']=[dict(address=hex(a),string=binary[a:a+150].split(b'\0')[0].decode()) for a in (0x44668f0,0x4466940)]
record['localization']=dict(function='SPRAL spral_match_order::mo_split',fde_entry='0x279b86b',fde_end='0x279cdfe',
    source='src/match_order.f90',fault_source_line=378,write_expression='iwork(j) = i; j = order(i)',
    confidence='Same current binary containing function established using FDE boundaries, embedded source filename and error string, exact source control flow and instruction accesses; not nearest exported symbol',
    unresolved='No original register values or source matrix, so the cause of the invalid effective write address cannot be determined.')
urls={
    'drake_spral_pin':'https://raw.githubusercontent.com/RobotLocomotion/drake/v1.51.1/tools/workspace/spral_internal/repository.bzl',
    'drake_ipopt_pin':'https://raw.githubusercontent.com/RobotLocomotion/drake/v1.51.1/tools/workspace/ipopt_internal/repository.bzl',
    'drake_metis_pin':'https://raw.githubusercontent.com/RobotLocomotion/drake/v1.51.1/tools/workspace/metis_internal/repository.bzl',
    'drake_metis_width':'https://raw.githubusercontent.com/RobotLocomotion/drake/v1.51.1/tools/workspace/metis_internal/patches/typewidth.patch',
    'spral_match_order':'https://raw.githubusercontent.com/ralna/spral/v2025.09.18/src/match_order.f90',
    'spral_ssids':'https://raw.githubusercontent.com/ralna/spral/v2025.09.18/src/ssids/ssids.f90',
    'spral_metis':'https://raw.githubusercontent.com/ralna/spral/v2025.09.18/src/metis5_wrapper.F90',
    'ipopt_interface':'https://raw.githubusercontent.com/coin-or/Ipopt/releases/3.14.19/src/Algorithm/LinearSolvers/IpSpralSolverInterface.cpp',
    'drake_minimum_distance_binding':'https://raw.githubusercontent.com/RobotLocomotion/drake/v1.51.1/bindings/pydrake/multibody/inverse_kinematics_py.cc',
    'drake_python_function_owner':'https://raw.githubusercontent.com/RobotLocomotion/drake/v1.51.1/bindings/pydrake/solvers/solvers_py_mathematicalprogram.cc',
}
record['official_sources']={}
texts={}
for name,url in urls.items():
    result=remote(url);texts[name]=result.pop('text');record['official_sources'][name]=result
record['official_dependency_versions']=dict(drake='1.51.1',ipopt='3.14.19',spral='v2025.09.18',metis='v5.2.1',metis_integer_bits=32)
record['source_route_review']=dict(
    spral_ssids_case1='options%ordering=1 -> metis_order directly; errors return before downstream permutation use.',
    spral_ssids_case2='options%ordering=2 -> match_order_metis -> mo_split.',
    ipopt_scaling_override='Ipopt 3.14.19 InitializeStructure overrides ordering to 2 when scaling_type==3 (matching); setting only spral_order=metis is insufficient.',
    mc64_route='spral_scaling=mc64 maps to scaling_type 1, leaving ordering=1 for explicit metis; uses Hungarian scaling, not matching-based ordering.',
    mathematical_constraints_changed=False,
    upstream_pr_848='https://github.com/coin-or/Ipopt/pull/848',
    pr848_scope='Official use-after-free fix concerns stale numeric fkeep_ after reanalysis of a singular matrix and subsequent solve; it is not established as the cause of this historical mo_split crash.',
    mitigation_scope='Explicit metis+mc64 bypasses the current binary function containing the historical fault. It is a configured workaround, not installation of the official PR848 patch or proof of zero native defects.')
import numpy,csdecomp,pydrake.solvers
paths=sorted({line.split()[-1] for line in Path('/proc/self/maps').read_text().splitlines()
    if '/' in line and any(token in Path(line.split()[-1]).name.lower() for token in ('drake','csdecomp','gfortran','gomp','stdc++','ipopt','metis','blas','spral','libgcc'))})
record['current_import_linkage']=dict(python=sys.executable,prefix=sys.prefix,numpy=numpy.__version__,csdecomp_file=csdecomp.__file__,
    drake_distributions=[dict(version=d.version,root=str(d.locate_file(''))) for d in md.distributions() if d.metadata['Name'].lower()=='drake'],
    library_paths=[dict(path=str(Path(p).relative_to(ROOT)) if Path(p).is_relative_to(ROOT) else p,sha256=sha(p)) for p in paths],
    observations=['One installed Drake distribution and one loaded libdrake.so from the existing Conda prefix.',
        'No dynamically loaded standalone libipopt, libspral or libmetis was observed; these solvers are vendored into libdrake.',
        'Wheel-private GNU OpenMP/Fortran runtimes from CSDecomp, Drake, NumPy and SciPy coexist under distinct hashed SONAMEs. This observation alone is not evidence of an ABI defect.',
        'No environment or shared library replacement was performed. Current linkage cannot establish the original crashing process mappings.'])
dist=md.distribution('drake')
rows=list(csv.reader(io.StringIO(dist.read_text('RECORD'))))
entry=next(row for row in rows if row[0]=='pydrake/lib/libdrake.so')
encoded=base64.urlsafe_b64encode(hashlib.sha256(LIB.read_bytes()).digest()).decode().rstrip('=')
record['wheel_integrity']=dict(record_entry=entry,sha256_record_matches=entry[1]=='sha256='+encoded)
from benchmarks.generic_motion_v1.model import Model
from benchmarks.generic_motion_v1.runners.nto_collision_constraint import RobustMinimumDistanceConstraint
case_path=ROOT/'benchmarks/generic_motion_v1/results/rigor_checks/dev_nto_incumbent/dev_seed0/jobs/dev-dense_clutter-001-p01-NTO_REFINED-minimum_time-4e4af77e/attempts/0001/input.json'
case=json.loads(case_path.read_text());model=Model(case['voxels_xyz'],case['voxel_radius_m'])
def forbidden(*args,**kwargs):raise AssertionError('CUDA forbidden in native localization')
model.cuda=forbidden
native=model.nto_plant();backend=RobustMinimumDistanceConstraint(native,0.,.01)
constraint=backend.constraint;reference=weakref.ref(backend)
expected=constraint.Eval(numpy.asarray(case['q_start'],float))
del backend,native,model
gc.collect()
actual=constraint.Eval(numpy.asarray(case['q_start'],float))
record['callback_owner_lifetime_probe']=dict(adapter_retained_after_caller_references_removed=reference() is not None,
    equal_after_forced_collection=bool(numpy.array_equal(expected,actual)),constraint_value=numpy.asarray(actual).tolist(),
    all_values_finite=bool(numpy.isfinite(actual).all()),gpu_calls=0,
    source_review='paper_nto local native -> RobustMinimumDistanceConstraint.native -> diagram+root context+plant context+scene context; NativeFirstCollisionSolve also owns backend through both solver invocations. Bound self.evaluate is captured by Drake PyFunctionConstraint. The public source wrapper retains func by value.',
    conclusion='No dangling owner demonstrated on this call path. Probe does not certify every native code path or absence of leaks.')
routing_path=OUT/'native_routes_final/summary.json'
record['actual_gdb_route_probe']=dict(path=str(routing_path.relative_to(ROOT)),sha256=sha(routing_path),
    summary=json.loads(routing_path.read_text()))
record['limitations']=['Original kernel record is one instruction pointer, not a call stack.',
    'Original binary was not hash-bound; localization is conditional on correspondence to the current installed wheel.',
    'Routing probe and lifetime check are CPU diagnostics, excluded from all benchmark success and timing tables.',
    'No root-cause claim for the original SIGSEGV is made, and its original infrastructure_error record is unchanged.']
record['artifact_sha256']={str(p.relative_to(ROOT)):sha(p) for p in sorted(OUT.glob('native_routes_final/**/*')) if p.is_file()}
record['collector_sha256']=sha(__file__)
(OUT/'native_localization.json').write_text(json.dumps(record,indent=2,sort_keys=True,allow_nan=False)+'\n')
print(json.dumps(dict(fde=containing,wheel_hash_matches=record['wheel_integrity']['sha256_record_matches'],callback=record['callback_owner_lifetime_probe'],route_counts=[x['route']['entry_count'] for x in record['actual_gdb_route_probe']['summary']['modes']]),indent=2))
