"""STOP-only microcontroller serial diagnostic; never sends ARM or movement."""
import argparse
import json
from pathlib import Path
import time

from .hereflow_diagnostic import port_info
from .robot_serial import RobotSerialClient


def diagnose(port, baud, duration):
    client=RobotSerialClient(path=port,baud=baud,reconnect_s=0.5)
    acknowledgements=[]
    start=time.monotonic()
    try:
        while time.monotonic()-start<duration:
            now=time.monotonic()
            for ack in client.tick(now):
                acknowledgements.append({'sequence':ack.sequence,
                                         'accepted':ack.accepted,
                                         'command':ack.command,'reason':ack.reason})
            time.sleep(0.02)
        report={**port_info(port),'baud_rate':baud,'duration_s':duration,
                'connected_at_end':client.connected,
                'open_errors':client.open_errors,'disconnects':client.disconnects,
                'invalid_ack_frames':client.invalid_acks,
                'acknowledgements':acknowledgements,
                'reason':client.reason,'movement_commands_sent':0,
                'diagnostic_mode':'STOP only; no ARM, RESET, or movement'}
        return report
    finally:
        client.close(time.monotonic())


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--serial-port',default='/dev/ttyACM0')
    parser.add_argument('--baud-rate',type=int,default=115200)
    parser.add_argument('--duration',type=float,default=5.0)
    parser.add_argument('--output',default='')
    args=parser.parse_args(argv)
    if args.duration<=0: parser.error('duration must be positive')
    report=diagnose(args.serial_port,args.baud_rate,args.duration)
    rendered=json.dumps(report,indent=2)
    print(rendered)
    if args.output:
        target=Path(args.output); target.parent.mkdir(parents=True,exist_ok=True)
        target.write_text(rendered+'\n')
    return 0 if any(a['accepted'] and a['command']=='STOP'
                    for a in report['acknowledgements']) else 2


if __name__=='__main__': raise SystemExit(main())
