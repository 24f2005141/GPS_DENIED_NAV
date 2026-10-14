"""Run finite ROS demos sequentially and summarize captured simulation metrics."""
import argparse
import json
from pathlib import Path
import subprocess
import sys

SCENARIOS = {
    'straight': ['--path','straight'],
    'turn': ['--path','turn'],
    'multi': ['--path','multi'],
    'noise': ['--path','straight','--noise-m','0.005'],
    'flow_dropout': ['--path','straight','--flow-dropout-start','0.5','--flow-dropout-end','1.0'],
    'imu_dropout': ['--path','straight','--imu-dropout-start','0.5','--imu-dropout-end','1.3'],
    'prolonged_imu_dropout': ['--path','straight','--imu-dropout-start','0.5','--imu-dropout-end','2.0'],
    'yaw_bias': ['--path','straight','--yaw-bias-rad-s','0.1'],
    'low_quality': ['--path','straight','--low-quality-start','0.5','--low-quality-end','1.0'],
    'mixed_rates': ['--path','turn','--flow-hz','10.0','--imu-hz','20.0'],
}
EXPECTED_ARRIVAL = {
    'straight': True, 'turn': True, 'multi': True, 'noise': True,
    'flow_dropout': False, 'imu_dropout': None, 'prolonged_imu_dropout': False,
    'yaw_bias': False,
    'low_quality': False, 'mixed_rates': True,
}
argument_parser=argparse.ArgumentParser()
argument_parser.add_argument('--output-dir',default='reports/validation')
options=argument_parser.parse_args()
base=Path(options.output_dir); base.mkdir(parents=True,exist_ok=True)
summary={}
for name,args in SCENARIOS.items():
    directory=base/name; directory.mkdir(exist_ok=True)
    with (directory/'stdout.txt').open('w') as out, (directory/'stderr.txt').open('w') as err:
        proc=subprocess.run(['ros2','run','gps_nav','demo','--mode','scripted',*args,
                             '--output-dir',str(directory)],
                            stdout=out,stderr=err,timeout=75,check=False)
    result=json.loads((directory/(args[1]+'.json')).read_text()) if proc.returncode==0 else {}
    summary[name]={'exit_code':proc.returncode,
                   'expected_arrival':EXPECTED_ARRIVAL[name],**result}
    print(name, 'exit',proc.returncode,'endpoint',result.get('endpoint_error_m'),
          'rmse',result.get('trajectory_rmse_m'),'arrival',result.get('destination_reached'),flush=True)
(base/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
if any(x['exit_code'] != 0 or
       (x['expected_arrival'] is not None and
        x.get('destination_reached') is not x['expected_arrival']) or
       not x.get('final_sample_processed', False)
       or not x.get('plot_history_matches_metrics', False)
       or (x['expected_arrival'] is None and x.get('destination_reached') and
           x.get('goal_error_m',float('inf')) > 0.25)
       for x in summary.values()):
    sys.exit(1)
