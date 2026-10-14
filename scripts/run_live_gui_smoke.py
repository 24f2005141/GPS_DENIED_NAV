"""Run demo against Qt's offscreen event loop and capture live graph frames."""
import os
import sys

os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
import matplotlib._c_internal_utils as display_check
display_check.display_is_valid=lambda: True  # Offscreen Qt is intentional here.

from gps_nav.demo import main

sys.argv=['demo',*sys.argv[1:]]
if '--show' not in sys.argv: sys.argv.append('--show')
if '--capture-live-frames' not in sys.argv: sys.argv.append('--capture-live-frames')
if '--gui-hold-seconds' not in sys.argv: sys.argv.extend(['--gui-hold-seconds','0.2'])
main()
