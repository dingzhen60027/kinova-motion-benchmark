"""Fresh public Git history retains exact old sources and sealed path bindings."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

import pytest

from benchmarks.generic_motion_v1 import batch, common


@pytest.fixture
def publication(tmp_path, monkeypatch):
    original = common.BASE / 'provenance'
    root = tmp_path / 'fresh-checkout'; root.mkdir()
    base = root / 'benchmarks/generic_motion_v1'
    shutil.copytree(original, base / 'provenance')
    subprocess.run(['git', 'init', '-q', '-b', 'main'], cwd=root, check=True)
    (root / 'README').write_text('Independent unit-test repository.\n')
    subprocess.run(['git', 'add', 'README'], cwd=root, check=True)
    subprocess.run(['git', '-c', 'user.name=Unit Test', '-c', 'user.email=test@example.invalid',
                    'commit', '-qm', 'New public history fixture'], cwd=root, check=True)
    monkeypatch.setattr(batch, 'ROOT', root)
    monkeypatch.setattr(batch, 'BASE', base)
    monkeypatch.setattr(common, 'ROOT', root)
    monkeypatch.setattr(common, 'BASE', base)
    return root, base, common.read(base / 'provenance/index.json')


def test_three_real_runtime_exports_work_without_any_old_git_objects(publication):
    root, base, index = publication
    assert len(index['commits']) == 3
    assert sum(map(len, index['commits'].values())) == 182
    for commit, sources in index['commits'].items():
        assert subprocess.run(['git', 'cat-file', '-e', commit], cwd=root,
                              capture_output=True).returncode != 0
        assert batch._git_source_errors(commit, sources) == []


@pytest.mark.parametrize('corruption', ['missing', 'bytes', 'index_hash', 'unknown_commit'])
def test_capsule_missing_or_corrupt_never_passes(publication, corruption):
    root, base, index = publication
    commit, sources = next(iter(index['commits'].items()))
    name, expected = next(iter(sources.items()))
    path = base / 'provenance/sources' / commit / name
    if corruption == 'missing': path.unlink()
    elif corruption == 'bytes': path.write_bytes(b'changed')
    elif corruption == 'index_hash':
        index['commits'][commit][name] = '0' * 64
        common.write(base / 'provenance/index.json', index)
    else: commit = '0' * 40
    assert batch._git_source_errors(commit, {name: expected}) == [name]


def test_existing_git_commit_mismatch_cannot_fall_back_to_capsule(publication):
    root, base, index = publication
    commit = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=root, text=True).strip()
    payload = b'A malicious capsule contradicting available Git.'
    expected = hashlib.sha256(payload).hexdigest()
    common.write(base / 'provenance/index.json', dict(index, commits={commit: {'README': expected}}))
    path = base / 'provenance/sources' / commit / 'README'; path.parent.mkdir()
    path.write_bytes(payload)
    assert batch._git_source_errors(commit, {'README': expected}) == ['README']


def test_relocation_exact_prefix_even_when_original_still_exists(publication, tmp_path):
    root, base, index = publication
    old = tmp_path / 'old-checkout'; old.mkdir()
    (old / 'input.json').write_text('original')
    (root / 'input.json').write_text('relocated')
    index['recorded_repository_roots'] = [str(old)]
    common.write(base / 'provenance/index.json', index)
    assert common.recorded_path(old / 'input.json') == root / 'input.json'
    assert common.recorded_path(str(old) + '-other/input.json') == Path(str(old) + '-other/input.json')
    assert common.recorded_path(tmp_path / 'elsewhere/input.json') == tmp_path / 'elsewhere/input.json'
    assert common.recorded_path('input.json') == root / 'input.json'
    with pytest.raises(ValueError, match='escapes'):
        common.recorded_path(old / '../elsewhere/input.json')


def test_relocation_rejects_symlink_escape(publication, tmp_path):
    root, base, index = publication
    old = Path(index['recorded_repository_roots'][0])
    (root / 'outside').symlink_to(tmp_path)
    with pytest.raises(ValueError, match='outside'):
        common.recorded_path(old / 'outside/file')
    with pytest.raises(ValueError, match='escapes'):
        common.recorded_path('outside/file')
    with pytest.raises(ValueError, match='escapes'):
        common.recorded_path('../file')


def test_capsule_directory_cannot_be_external_symlink(publication, tmp_path):
    root, base, index = publication
    commit, sources = next(iter(index['commits'].items()))
    capsule = base / 'provenance/sources' / commit
    outside = tmp_path / 'external-capsule'
    capsule.rename(outside); capsule.symlink_to(outside)
    assert batch._git_source_errors(commit, sources) == list(sources)


def test_public_cli_honors_launcher_existing_environment_override(tmp_path, monkeypatch):
    from benchmarks.generic_motion_v1 import cli
    import sys
    monkeypatch.setattr(cli, 'ROOT', tmp_path)
    monkeypatch.setenv('CSDECOMP_CONDA_PREFIX', sys.prefix)
    cli.require_environment()
    monkeypatch.setenv('CSDECOMP_CONDA_PREFIX', str(tmp_path / 'wrong-environment'))
    with pytest.raises(RuntimeError, match='Conda'):
        cli.require_environment()
