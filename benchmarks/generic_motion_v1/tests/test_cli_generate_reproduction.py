"""CPU dispatch/path safety tests; the generator is never executed."""
from pathlib import Path
import json
import pytest
from benchmarks.generic_motion_v1 import cli, generator


@pytest.fixture
def setup(tmp_path, monkeypatch):
    root=tmp_path/'repo';base=root/'benchmarks/generic_motion_v1'
    for folder in ('receipts','configs'):(base/folder).mkdir(parents=True,exist_ok=True)
    for i,split in enumerate(('smoke','dev','test')):
        (base/f'receipts/{split}_generation.json').write_text(json.dumps({'seed':42+i}))
        (base/f'configs/{split}_generator.json').write_text('{}')
    (root/'benchmark').symlink_to(base,target_is_directory=True)
    monkeypatch.setattr(cli,'ROOT',root)
    monkeypatch.setattr(cli,'BASE',base)
    monkeypatch.setattr(cli,'require_environment',lambda:None)
    calls=[]
    def fake_generate(split,seed,output,**kw):
        calls.append((split,seed,output,kw))
        return {'parameter_test_only':True}
    monkeypatch.setattr(generator,'generate',fake_generate)
    return root,base,calls


@pytest.mark.parametrize('split,seed',[('smoke',42),('dev',43),('test',44)])
def test_reconstruction_uses_original_split_seed_and_config(setup,split,seed,capsys):
    root,base,calls=setup
    cli.benchmark_main(['generate','--split',split,'--output','new-reproduction'])
    assert calls==[(split,seed,root/'new-reproduction',dict(config_path=base/f'configs/{split}_generator.json',unlock_test=split=='test'))]
    assert not (root/'new-reproduction').exists()


def test_explicit_seed_and_config_remain_supported_from_other_cwd(setup,tmp_path,monkeypatch):
    root,base,calls=setup
    monkeypatch.chdir(tmp_path)
    cli.benchmark_main(['generate','--split','test','--seed','99','--config','custom.json','--output',str(tmp_path/'fresh-test')])
    assert calls==[('test',99,tmp_path/'fresh-test',dict(config_path=root/'custom.json',unlock_test=True))]


@pytest.mark.parametrize('kind',['canonical','canonical_child','alias','alias_child','ancestor','existing_empty','existing_nonempty','existing_file','broken_symlink','parent_symlink_to_canonical'])
def test_reconstruction_never_overwrites_existing_or_canonical(setup,tmp_path,kind):
    root,base,calls=setup
    if kind=='canonical': output=base
    elif kind=='canonical_child':output=base/'new'
    elif kind=='alias':output=root/'benchmark'
    elif kind=='alias_child':output=root/'benchmark/new'
    elif kind=='ancestor':output=root
    elif kind=='existing_empty':output=tmp_path/'old';output.mkdir()
    elif kind=='existing_nonempty':
        output=tmp_path/'old';output.mkdir();(output/'marker').write_text('unique original data')
    elif kind=='existing_file':output=tmp_path/'old';output.write_text('unique original data')
    elif kind=='broken_symlink':
        output=tmp_path/'dangling';output.symlink_to(tmp_path/'missing',target_is_directory=True)
    elif kind=='parent_symlink_to_canonical':
        parent=tmp_path/'external-alias';parent.symlink_to(base,target_is_directory=True);output=parent/'new'
    before={str(p):p.read_bytes() for p in tmp_path.rglob('*') if p.is_file()}
    with pytest.raises(SystemExit) as e:
        cli.benchmark_main(['generate','--split','test','--output',str(output)])
    assert e.value.code==2 and calls==[]
    assert before=={str(p):p.read_bytes() for p in tmp_path.rglob('*') if p.is_file()}


def test_legacy_generator_test_gate_is_unchanged(tmp_path):
    from benchmarks.generic_motion_v1.generator.manifest import generate
    with pytest.raises(RuntimeError,match='locked'):
        generate('test',42,tmp_path/'fresh')
