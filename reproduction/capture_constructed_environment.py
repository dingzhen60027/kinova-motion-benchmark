"""Capture the reused environment without installing or changing dependencies."""
import datetime
import hashlib
import os
import platform
import subprocess
import sys
from reproduction.common import ROOT, atomic
from reproduction.run import runtime_versions


def command(argv):
    result = subprocess.run(argv, cwd=ROOT, text=True, capture_output=True)
    return dict(command=argv, exit_code=result.returncode,
                stdout=result.stdout.strip(), stderr=result.stderr.strip())


def main():
    locks = ["reproduction/conda-explicit.txt", "reproduction/requirements.lock.txt",
             "local/python.sh"]
    result = dict(captured_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        executable=sys.executable, prefix=sys.prefix, python=sys.version,
        platform=platform.platform(), os_release=open("/etc/os-release").read(),
        runtime_versions=runtime_versions(),
        thread_environment={k: os.getenv(k) for k in
                            ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS")},
        gpu=command(["nvidia-smi", "--query-gpu=name,driver_version,memory.total,compute_cap", "--format=csv,noheader"]),
        cpu=command(["lscpu"]), nvcc=command(["nvcc", "--version"]),
        git_head=command(["git", "rev-parse", "HEAD"]),
        git_branch=command(["git", "branch", "--show-current"]),
        locks={p: hashlib.sha256((ROOT / p).read_bytes()).hexdigest() for p in locks},
        environment_reused=True, newly_installed_packages=[],
        fresh_environment_restore_tested=False)
    atomic(ROOT / "reproduction/reports/paper_constructed/environment.json", result)
    print(result)


if __name__ == "__main__":
    main()
