import os
import pty
import select
import threading

from gps_nav.firmware_model import MicrocontrollerModel
from gps_nav.robot_protocol import decode_command
from gps_nav.robot_serial_diagnostic import diagnose


def test_missing_port_reports_safe_failure(tmp_path):
    result=diagnose(str(tmp_path/'missing'),115200,0.06)
    assert not result['exists']
    assert result['open_errors']>=1
    assert result['acknowledgements']==[]
    assert result['movement_commands_sent']==0


def test_diagnostic_sends_only_stop_over_pseudoterminal():
    master,slave=pty.openpty()
    model=MicrocontrollerModel()
    received=[]
    def endpoint():
        while select.select([master],[],[],0.4)[0]:
            frame=os.read(master,1024)
            if not frame: break
            for line in frame.splitlines(keepends=True):
                if not line.endswith(b'\n'): continue
                command=decode_command(line)
                received.append(command.direction)
                os.write(master,model.process(line,0.0))
    worker=threading.Thread(target=endpoint)
    worker.start()
    try:
        result=diagnose(os.ttyname(slave),115200,0.12)
    finally:
        worker.join(timeout=1.0)
        os.close(master); os.close(slave)
    assert result['movement_commands_sent']==0
    assert received and set(received)=={'STOP'}
    assert any(a['accepted'] and a['command']=='STOP'
               for a in result['acknowledgements'])
