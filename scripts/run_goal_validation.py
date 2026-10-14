"""Exercise goal-directed arrival and honest nonarrival under sensor faults."""
import argparse
import json
from pathlib import Path
import subprocess
import sys

parser=argparse.ArgumentParser()
parser.add_argument('--output-dir',default='reports/goal_validation')
args=parser.parse_args()
base=Path(args.output_dir); base.mkdir(parents=True,exist_ok=True)
scenarios={
    'clean': ([],True),
    'noise': (['--noise-m','0.005'],True),
    'flow_dropout': (['--flow-dropout-start','0.5','--flow-dropout-end','1.0'],False),
    'low_quality': (['--low-quality-start','0.5','--low-quality-end','1.0'],False),
}
summary={}
for name,(extra,expected) in scenarios.items():
    directory=base/name; directory.mkdir(exist_ok=True)
    command=['ros2','run','gps_nav','demo','--mode','goal','--goal-x-m','3',
             '--goal-y-m','0','--speed-m-s','0.6','--max-duration-s','8',
             '--output-dir',str(directory),*extra]
    with (directory/'stdout.txt').open('w') as out, (directory/'stderr.txt').open('w') as err:
        proc=subprocess.run(command,stdout=out,stderr=err,timeout=35,check=False)
    result=json.loads((directory/'goal.json').read_text()) if proc.returncode==0 else {}
    summary[name]={'exit_code':proc.returncode,'expected_arrival':expected,**result}
    print(name,'exit',proc.returncode,'arrival',result.get('destination_reached'),
          'reason',result.get('completion_reason'),'ratio',result.get('path_length_ratio'),flush=True)
(base/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
if any(r['exit_code']!=0 or r.get('destination_reached') is not r['expected_arrival'] or
       not r.get('plot_history_matches_metrics',False) or
       not r.get('final_sample_processed',False) or
       (r['expected_arrival'] and r.get('completion_reason')!='destination reached') or
       (not r['expected_arrival'] and r.get('completion_reason')!='timeout')
       for r in summary.values()):
    sys.exit(1)
