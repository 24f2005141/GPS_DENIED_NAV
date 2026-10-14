"""Offscreen test that closes a running interactive graph and checks cleanup."""
import json
import os
from pathlib import Path
import sys
import time

os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
import matplotlib._c_internal_utils as display_check
display_check.display_is_valid=lambda: True
import matplotlib.pyplot as plt

from gps_nav.demo import main

out=Path(sys.argv[1] if len(sys.argv)>1 else 'reports/interactive_close_smoke').resolve()
out.mkdir(parents=True,exist_ok=True)
original_pause=plt.pause
started=None
closed=False

def close_after_motion(interval):
    global started,closed
    if started is None:
        started=time.monotonic()
    if not closed and time.monotonic()-started > 3.5:
        closed=True
        plt.close('all')
    if not closed:
        original_pause(interval)

plt.pause=close_after_motion
sys.argv=['demo','--mode','interactive','--show','--domain-id','212',
          '--goal-x-m','3','--goal-y-m','0','--output-dir',str(out)]
main()
report=json.loads((out/'interactive.json').read_text())
print(json.dumps({'mode':report['mode'],'completed':report['completed'],
                  'completion_reason':report['completion_reason'],
                  'trajectory_samples':report.get('trajectory_samples'),
                  'plot_history_matches_metrics':report['plot_history_matches_metrics']},indent=2))
if not closed or report['mode'] != 'interactive' or not report['plot_history_matches_metrics']:
    raise SystemExit(1)
