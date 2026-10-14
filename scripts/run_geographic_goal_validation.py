"""Exercise geographic goal input at several origins and initial headings."""
import argparse
import json
from pathlib import Path
import subprocess
import sys

from gps_nav.geo import nearby_geodetic


parser=argparse.ArgumentParser()
parser.add_argument('--output-dir',default='reports/geographic_goal_validation')
options=parser.parse_args()
base=Path(options.output_dir); base.mkdir(parents=True,exist_ok=True)
cases={
    'north_aligned': ((13.0827,80.2707),(0.0,3.0),90.0),
    'north_initial_east': ((13.0827,80.2707),(0.0,3.0),0.0),
    'northeast_initial_west': ((37.0,-122.0),(2.12,2.12),180.0),
    'west_initial_north': ((0.5,30.0),(-3.0,0.0),90.0),
}
results={}
for name,(origin,offset,heading) in cases.items():
    goal=nearby_geodetic(*origin,*offset)
    directory=base/name; directory.mkdir(exist_ok=True)
    command=['ros2','run','gps_nav','demo','--mode','goal',
             '--start-lat',str(origin[0]),'--start-lon',str(origin[1]),
             '--destination-lat',str(goal[0]),'--destination-lon',str(goal[1]),
             '--initial-heading-deg',str(heading),'--max-duration-s','12',
             '--output-dir',str(directory)]
    with (directory/'stdout.txt').open('w') as stdout, (directory/'stderr.txt').open('w') as stderr:
        process=subprocess.run(command,stdout=stdout,stderr=stderr,timeout=35,check=False)
    result=json.loads((directory/'goal.json').read_text()) if process.returncode==0 else {}
    results[name]={'exit_code':process.returncode,'requested_origin':origin,
                   'requested_offset_m':offset,'requested_heading_deg':heading,**result}
    print(name,'exit',process.returncode,'arrival',result.get('destination_reached'),
          'path',result.get('estimated_path_length_m'),
          'distance',result.get('straight_line_distance_m'),
          'endpoint_error',result.get('endpoint_error_m'),flush=True)
(base/'summary.json').write_text(json.dumps(results,indent=2)+'\n')
if any(r['exit_code'] or not r.get('destination_reached') or
       not r.get('final_sample_processed') or not r.get('plot_history_matches_metrics') or
       r.get('endpoint_error_m',float('inf')) > 0.05 or
       abs(r.get('initial_heading_deg',float('inf'))-r['requested_heading_deg']) > 1e-10 or
       r.get('completion_reason') != 'destination reached'
       for r in results.values()):
    sys.exit(1)
