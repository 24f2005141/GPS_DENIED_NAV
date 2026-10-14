import os
import pty
import pytest
from gps_nav.serial_io import SerialPort
from gps_nav.parser import FlowParser
from gps_nav.sim_core import PathSimulator
from gps_nav.core import Tracker


def packet(x,y):
    b=x.to_bytes(2,'little',signed=True)+y.to_bytes(2,'little',signed=True)
    return b'\xfe\x04'+b+bytes([sum(b)&255,255,0xaa])


def test_serial_pseudoterminal_and_reconnect():
    master,slave=pty.openpty(); path=os.ttyname(slave)
    first=SerialPort(path)
    try:
        os.write(master,packet(-7,8))
        assert FlowParser().feed(first.read(timeout=0.5))[0].x == -7
    finally:
        first.close()
    second=SerialPort(path)
    try:
        os.write(master,packet(9,-10))
        assert FlowParser().feed(second.read(timeout=0.5))[0].y == -10
    finally:
        second.close(); os.close(master); os.close(slave)
    with pytest.raises(OSError): SerialPort(path)


def test_serial_eof_detection():
    master,slave=pty.openpty(); path=os.ttyname(slave)
    port=SerialPort(path)
    os.close(master)
    try:
        with pytest.raises(OSError): port.read(timeout=0.5)
    finally:
        port.close(); os.close(slave)


@pytest.mark.parametrize('scenario', [
    {'noise_m':0.005},
    {'dropout_start':0.5,'dropout_end':1.0},
])
def test_simulated_pipeline_reproducible(scenario):
    def run():
        s=PathSimulator('straight',seed=7,**scenario)
        t=Tracker(stale_s=0.6)
        while (sample:=s.step()) is not None:
            t.imu(sample.yaw,sample.t)
            if s.flow_available():
                dx,dy=s.noisy_flow(sample)
                t.flow(dx,dy,sample.t,s.quality(),True)
        return t.x,t.y,s.x,s.y
    assert run()==run()
    x,y,tx,ty=run()
    if 'dropout_start' in scenario:
        assert x == pytest.approx(2.55, abs=1e-9)
        assert tx-x == pytest.approx(0.45, abs=1e-9)
    else:
        assert abs(x-tx)<0.2
