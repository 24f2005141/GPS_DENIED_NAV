"""Standalone UART inspection. Never starts ROS navigation or converts counts to metres."""
import argparse
import json
import os
from pathlib import Path
import stat
import time

from .hereflow_reader import HereFlowReader


def port_info(path):
    info = {'port': path, 'exists': False, 'character_device': False,
            'read_access_hint': False, 'stat_error': ''}
    try:
        mode = os.stat(path).st_mode
        info.update(exists=True, character_device=stat.S_ISCHR(mode),
                    read_access_hint=os.access(path, os.R_OK))
    except OSError as exc:
        info['stat_error'] = str(exc)
    return info


def diagnose(args):
    info = port_info(args.serial_port)
    reader = HereFlowReader(args.serial_port, args.baud_rate, args.serial_timeout,
                            args.reconnect_interval, args.byte_order, args.stale_after)
    capture = None
    below_threshold = 0
    try:
        if args.capture:
            target = Path(args.capture)
            target.parent.mkdir(parents=True, exist_ok=True)
            capture = target.open('w')
        end = time.monotonic() + args.duration
        while time.monotonic() < end:
            packets, data = reader.poll()
            if data and capture:
                capture.write(json.dumps({'receive_time_unix_ns': time.time_ns(),
                                          'bytes_hex': data.hex()}) + '\n')
                capture.flush()
            if args.quality_threshold is not None:
                below_threshold += sum(p.quality < args.quality_threshold for p in packets)
            time.sleep(0.01)
    finally:
        if capture:
            capture.close()
        status = reader.status()
        reader.close()
    report = {**info, **status,
              'duration_s': args.duration,
              'baud_rate': args.baud_rate,
              'byte_order_configured': args.byte_order,
              'quality_threshold': args.quality_threshold,
              'quality_below_threshold': below_threshold if args.quality_threshold is not None else None,
              'raw_capture': args.capture or None,
              'measurement_units': 'raw signed counts; physical meaning unverified',
              'metric_localization_tested': False,
              'interpretation': 'Accepted framing is a protocol hypothesis until checked against this device'}
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--serial-port', default='/dev/ttyUSB0')
    parser.add_argument('--baud-rate', type=int, default=19200)
    parser.add_argument('--serial-timeout', type=float, default=0.01)
    parser.add_argument('--reconnect-interval', type=float, default=2.0)
    parser.add_argument('--stale-after', type=float, default=0.6)
    parser.add_argument('--byte-order', choices=('little', 'big'), default='little')
    parser.add_argument('--duration', type=float, default=5.0,
                        help='Observe for this many seconds, including reconnect attempts')
    parser.add_argument('--quality-threshold', type=int, default=None,
                        help='Optional raw 0..255 quality threshold for a count only')
    parser.add_argument('--capture', default='', help='Optional timestamped raw hex JSONL file')
    parser.add_argument('--output', default='', help='Optional summary JSON file')
    args = parser.parse_args(argv)
    if args.duration <= 0 or args.serial_timeout < 0 or args.reconnect_interval < 0 or args.stale_after <= 0:
        parser.error('duration and stale-after must be positive; timeouts must be nonnegative')
    if args.quality_threshold is not None and not 0 <= args.quality_threshold <= 255:
        parser.error('quality threshold must be within 0..255')
    report = diagnose(args)
    rendered = json.dumps(report, indent=2)
    print(rendered)
    if args.output:
        target = Path(args.output)
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(rendered + '\n')
    return 0 if report['valid_packets'] else (2 if not report['connections'] else 3)
