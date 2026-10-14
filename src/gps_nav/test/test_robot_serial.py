import os
import pty
import select

from gps_nav.firmware_model import MicrocontrollerModel
from gps_nav.robot_protocol import Ack, Command, decode_ack, decode_command
from gps_nav.robot_serial import RobotSerialClient


def exchange(master,client,model,now):
    data=read_frame(master)
    command=decode_command(data)
    os.write(master,model.process(data,now))
    acknowledgements=client.tick(now+0.01)
    return command,acknowledgements


def read_frame(master):
    assert select.select([master],[],[],0.5)[0], 'serial frame was not transmitted'
    return os.read(master,1024)


def test_pseudoterminal_command_ack_stop_and_timeout():
    master,slave=pty.openpty(); path=os.ttyname(slave)
    client=RobotSerialClient(path,ack_timeout_s=0.2,command_timeout_s=0.2)
    model=MicrocontrollerModel(max_speed_permille=150)
    try:
        assert client.tick(1.0)==[]  # Port opens and sends startup STOP.
        command,acks=exchange(master,client,model,1.01)
        assert command.direction=='STOP' and acks[0].accepted
        assert client.ack_fresh(1.02)
        client.queue(Command(2,'ARM',0),1.02); client.tick(1.02)
        command,acks=exchange(master,client,model,1.03)
        assert command.direction=='ARM' and acks[0].command=='ARM'
        client.queue(Command(3,'FORWARD',80),1.04); client.tick(1.04)
        command,acks=exchange(master,client,model,1.05)
        assert command.direction=='FORWARD' and model.direction=='FORWARD'
        client.tick(1.3)  # Supervisor input ceased: bridge transmits STOP.
        stopped=decode_command(read_frame(master))
        assert stopped.direction=='STOP'
        os.write(master,model.process(encode_stopped(stopped),1.3))
        client.tick(1.31)
        assert model.direction=='STOP'
    finally:
        client.close(2.0); os.close(master); os.close(slave)


def encode_stopped(command):
    from gps_nav.robot_protocol import encode_command
    return encode_command(command)


def test_missing_port_reconnect_and_ack_timeout(tmp_path):
    link=tmp_path/'robot'; client=RobotSerialClient(str(link),reconnect_s=0.1,
                                                     ack_timeout_s=0.1)
    assert client.tick(1.0)==[] and client.open_errors==1 and not client.connected
    master,slave=pty.openpty(); os.symlink(os.ttyname(slave),link)
    try:
        client.tick(1.11)
        assert client.connected
        assert decode_command(read_frame(master)).direction=='STOP'
        client.tick(1.22)  # No ACK: fail closed and schedule reconnection.
        assert not client.connected
        assert 'acknowledgement timeout' in client.reason
        assert decode_command(read_frame(master)).direction=='STOP'
        client.tick(1.33)
        assert client.connected
        assert decode_command(read_frame(master)).direction=='STOP'
        os.close(master); master=None
        client.tick(1.34)
        assert not client.connected
    finally:
        client.close(2.0)
        if master is not None: os.close(master)
        os.close(slave)


def test_rejected_ack_never_qualifies_as_healthy_link():
    client=RobotSerialClient('/dev/does-not-exist')
    client.port=object()
    client.last_ack_at=1.0
    client.last_ack=Ack(1,False,'STOP','FAULT')
    assert not client.ack_fresh(1.01)
    client.last_ack=Ack(1,True,'STOP','STOPPED')
    assert client.ack_fresh(1.01)
