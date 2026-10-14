import pytest

from gps_nav.robot_protocol import (Ack, Command, LineBuffer, ProtocolError,
                                    decode_ack, decode_command, encode_ack,
                                    encode_command, frame, sequence_newer)
from gps_nav.firmware_model import MicrocontrollerModel


@pytest.mark.parametrize('name,speed', [('FORWARD',100),('BACKWARD',90),
                                         ('LEFT',75),('RIGHT',75),('STOP',0),
                                         ('ARM',0),('RESET',0),('DISARM',0)])
def test_command_and_ack_round_trip(name,speed):
    command=Command(123,name,speed)
    assert decode_command(encode_command(command))==command
    ack=Ack(123,True,name,'ACCEPTED')
    assert decode_ack(encode_ack(ack))==ack


@pytest.mark.parametrize('data,reason', [
    (b'V1,1,FORWARD,100,0000\n','BAD_CRC'),
    (frame('V1,1,FORWARD,100')[:-1],'BAD_FRAME'),
    (frame('V1,0,FORWARD,100'),'BAD_SEQUENCE'),
    (frame('V1,1,FLY,100'),'BAD_COMMAND'),
    (frame('V1,1,FORWARD,1001'),'BAD_SPEED'),
    (frame('V1,1,STOP,1'),'BAD_SPEED'),
    (b'V1,1,FORWARD,100,FFFF\n'+b'x'*100,'BAD_FRAME'),
])
def test_protocol_rejects_invalid_frames(data,reason):
    with pytest.raises(ProtocolError,match=reason):
        decode_command(data)


def test_streaming_frames_and_sequence_wrap():
    parser=LineBuffer(); one=encode_command(Command(1,'STOP',0))
    two=encode_command(Command(2,'ARM',0))
    assert parser.feed(b'x'*100+b'\n'+one[:5])==[]
    assert parser.discarded==1
    assert parser.feed(one[5:]+two)==[one,two]
    assert sequence_newer(1,0xffffffff)
    assert not sequence_newer(5,5)
    assert not sequence_newer(4,5)


def test_firmware_startup_arm_motion_watchdog_and_recovery():
    model=MicrocontrollerModel(watchdog_s=0.4,max_speed_permille=150)
    assert model.direction=='STOP' and not model.armed
    denied=decode_ack(model.process(encode_command(Command(1,'FORWARD',100)),0.0))
    assert not denied.accepted and denied.reason=='DISARMED'
    assert decode_ack(model.process(encode_command(Command(2,'ARM',0)),0.1)).reason=='ARMED'
    assert decode_ack(model.process(encode_command(Command(3,'FORWARD',100)),0.2)).accepted
    assert model.direction=='FORWARD' and model.speed==100
    assert model.watchdog(0.61)
    assert model.direction=='STOP' and model.fault and not model.armed
    rejected=decode_ack(model.process(encode_command(Command(4,'ARM',0)),0.62))
    assert rejected.reason=='FAULT'
    assert decode_ack(model.process(encode_command(Command(5,'RESET',0)),0.63)).accepted
    assert decode_ack(model.process(encode_command(Command(6,'ARM',0)),0.64)).accepted
    assert decode_ack(model.process(encode_command(Command(7,'LEFT',80)),0.65)).accepted


def test_stop_priority_stale_packet_and_speed_limit():
    model=MicrocontrollerModel(max_speed_permille=100)
    model.process(encode_command(Command(10,'ARM',0)),0)
    model.process(encode_command(Command(11,'FORWARD',80)),0.01)
    stop=decode_ack(model.process(encode_command(Command(1,'STOP',0)),0.02))
    assert stop.accepted and model.direction=='STOP' and not model.armed
    model.process(encode_command(Command(12,'ARM',0)),0.03)
    bad=decode_ack(model.process(encode_command(Command(13,'FORWARD',101)),0.04))
    assert not bad.accepted and bad.reason=='BAD_SPEED'
    assert model.direction=='STOP' and model.fault
    model.process(encode_command(Command(14,'RESET',0)),0.05)
    model.process(encode_command(Command(15,'ARM',0)),0.06)
    model.process(encode_command(Command(16,'FORWARD',80)),0.07)
    duplicate=decode_ack(model.process(encode_command(Command(16,'FORWARD',80)),0.08))
    assert duplicate.reason=='STALE' and model.direction=='STOP'


def test_bad_crc_stops_and_latches_fault():
    model=MicrocontrollerModel()
    model.process(encode_command(Command(1,'ARM',0)),0)
    model.process(encode_command(Command(2,'RIGHT',60)),0.1)
    bad=encode_command(Command(3,'RIGHT',60)).replace(b'RIGHT,60',b'RIGHT,61')
    ack=decode_ack(model.process(bad,0.2))
    assert not ack.accepted and model.direction=='STOP' and model.fault
