"""Render saved validated trajectories and explicit failure views; never plan."""
from collections import defaultdict
from pathlib import Path
import html
import re
import traceback
from ...common import read,write,sha
from ...runners.replay import replay
from .run_extended_study import OUT,check_frozen

LABELS=('LSCS','RRTConnect','RRT*','CHOMP','STOMP','TrajOpt')
RULE=('Illustrations only: prefer the lexicographically first blocked dev input and repetition '
      'on which all six methods validated; otherwise select the first validated result per method '
      'under the same blocked-first order. Add the first failed legal result of each method. '
      'No cost or latency ranking; these selected views are not a performance sample.')

def order(row):
    return (row['straight_line_collision'] is not True,row['case_id'],row['roadmap_seed'],row['method_seed'])

def main():
    frozen=read(OUT/'execution_freeze.json');check_frozen(frozen)
    done=read(OUT/'completed_blocks.json');audit=read(OUT/'final_audit.json')
    if not done['complete'] or done['completed_jobs']!=1140 or audit.get('passed') is not True:
        raise ValueError('All frozen planning and independent audit must finish before rendering')
    metrics=read(OUT/'metrics/metrics.json')
    if not metrics['complete'] or not metrics['comparable'] or len(metrics['rows'])!=1140:
        raise ValueError('Incomplete comparison')
    rows=[r for r in metrics['rows'] if r['split']=='dev']
    groups=defaultdict(list)
    for row in rows:
        if row['success']:groups[(row['case_id'],row['roadmap_id'],row['method_seed'])].append(row)
    common=[g for g in groups.values() if len(g)==6 and {r['label'] for r in g}==set(LABELS)
            and all(r['straight_line_collision'] is True for r in g)]
    selected=min(common,key=lambda g:order(g[0])) if common else []
    if not common:
        selected=[min(pool,key=order) for label in LABELS if (pool:=[r for r in rows if r['label']==label and r['success']])]
    choices=[(r,'validated_final_trajectory') for r in sorted(selected,key=lambda r:LABELS.index(r['label']))]
    for label in LABELS:
        failed=[r for r in rows if r['label']==label and r['input_legal'] and not r['success']]
        if failed:choices.append((min(failed,key=order),'failed_attempt_saved_evidence'))
    destination=OUT/'replays'
    if destination.exists():raise FileExistsError('Preserve existing derived gallery')
    destination.mkdir()
    result=dict(complete=False,selection_rule=RULE,common_success_query=bool(common),
                metrics_sha256=sha(OUT/'metrics/metrics.json'),audit_sha256=sha(OUT/'final_audit.json'),
                entries=[],missing_success_labels=[label for label in LABELS if label not in {r['label'] for r in selected}])
    for row,role in choices:
        name=re.sub('[^a-z0-9]+','_',row['label'].lower()).strip('_')
        if row['label']=='RRT*':name='rrtstar'
        if role.startswith('failed'):name+='_failure'
        item=dict(name=name,label=row['label'],case_id=row['case_id'],roadmap_seed=row['roadmap_seed'],
                  role=role,status=row['status'],source_attempt=row['attempt'],
                  source_receipt_sha256=sha(Path(row['attempt'])/'receipt.json'),straight_line_collision=row['straight_line_collision'])
        try:
            item['render']=replay(row['attempt'],destination/name)
            item['rendered']=True
            item['files']={str(p.relative_to(destination)):sha(p) for p in sorted((destination/name).rglob('*')) if p.is_file()}
        except Exception as exc:
            item.update(rendered=False,error=repr(exc),traceback=traceback.format_exc())
        result['entries'].append(item);write(destination/'index.json',result)
    result['complete']=(all(item['rendered'] for item in result['entries'])
                        and {item['label'] for item in result['entries']}==set(LABELS))
    write(destination/'index.json',result)
    cards=[]
    for item in result['entries']:
        note='Saved validated output' if item['role']=='validated_final_trajectory' else 'FAILED attempt; saved samples or endpoints only, not a valid solution'
        cards.append('<article><h2>'+html.escape(item['label'])+'</h2><p>'+html.escape(note)+'</p><p>'+html.escape(item['case_id']+' · '+item['status'])+'</p><a href="'+item['name']+'/replay.html">Open interactive replay</a><br><img src="'+item['name']+'/scene.png" alt="Canonical collision geometry"></article>')
    (destination/'index.html').write_text('<!doctype html><html><meta charset="utf-8"><title>Extended Kinova baselines</title><style>body{font:16px system-ui;max-width:1080px;margin:40px auto;padding:0 24px}article{border-top:1px solid #ccc;padding:20px 0}img{max-width:720px;width:100%}p{line-height:1.6}</style><h1>Extended Kinova baselines</h1><p>'+html.escape(RULE)+'</p>'+''.join(cards)+'</html>')
    print(str(destination/'index.html'))
    if not result['complete']:raise RuntimeError('Replay failures retained in gallery index')

if __name__=='__main__':main()
