"""Versioned ASCII motor-command protocol; speed is per mille, never PWM."""
from dataclasses import dataclass
import binascii

COMMANDS={'FORWARD','BACKWARD','LEFT','RIGHT','STOP','ARM','RESET','DISARM'}
ACK_STATES={'OK','ERR'}
MAX_FRAME=96
MAX_SEQUENCE=0xffffffff


class ProtocolError(ValueError):
    def __init__(self, reason, sequence=0):
        super().__init__(reason)
        self.reason=reason
        self.sequence=sequence


@dataclass(frozen=True)
class Command:
    sequence: int
    direction: str
    speed_permille: int=0


@dataclass(frozen=True)
class Ack:
    sequence: int
    accepted: bool
    command: str
    reason: str


def checksum(payload):
    return binascii.crc_hqx(payload.encode('ascii'),0xffff)


def frame(payload):
    return f'{payload},{checksum(payload):04X}\n'.encode('ascii')


def _sequence(text, allow_zero=False):
    if not text.isdecimal() or len(text)>10:
        raise ProtocolError('BAD_SEQUENCE')
    value=int(text)
    if value>MAX_SEQUENCE or (value==0 and not allow_zero):
        raise ProtocolError('BAD_SEQUENCE')
    return value


def _body(data, field_count, prefix):
    if not isinstance(data,bytes) or len(data)>MAX_FRAME or not data.endswith(b'\n'):
        raise ProtocolError('BAD_FRAME')
    try:
        text=data[:-1].decode('ascii')
    except UnicodeDecodeError as exc:
        raise ProtocolError('BAD_FRAME') from exc
    if '\r' in text:
        text=text[:-1] if text.endswith('\r') else text
    fields=text.split(',')
    if len(fields)!=field_count or fields[0]!=prefix:
        raise ProtocolError('BAD_FRAME')
    try:
        given=int(fields[-1],16)
    except ValueError as exc:
        raise ProtocolError('BAD_CRC') from exc
    if len(fields[-1])!=4 or checksum(','.join(fields[:-1]))!=given:
        raise ProtocolError('BAD_CRC')
    return fields


def encode_command(command):
    if command.direction not in COMMANDS:
        raise ProtocolError('BAD_COMMAND')
    sequence=_sequence(str(command.sequence))
    speed=command.speed_permille
    if isinstance(speed,bool) or not isinstance(speed,int) or not 0<=speed<=1000:
        raise ProtocolError('BAD_SPEED',sequence)
    if (command.direction in ('STOP','ARM','RESET','DISARM')) != (speed==0):
        raise ProtocolError('BAD_SPEED',sequence)
    return frame(f'V1,{sequence},{command.direction},{speed}')


def decode_command(data):
    fields=_body(data,5,'V1')
    sequence=_sequence(fields[1])
    direction=fields[2]
    if direction not in COMMANDS:
        raise ProtocolError('BAD_COMMAND',sequence)
    if not fields[3].isdecimal() or len(fields[3])>4:
        raise ProtocolError('BAD_SPEED',sequence)
    speed=int(fields[3])
    if speed>1000 or ((direction in ('STOP','ARM','RESET','DISARM')) != (speed==0)):
        raise ProtocolError('BAD_SPEED',sequence)
    return Command(sequence,direction,speed)


def encode_ack(ack):
    sequence=_sequence(str(ack.sequence),allow_zero=True)
    if ack.command not in COMMANDS and ack.command!='UNKNOWN':
        raise ProtocolError('BAD_COMMAND',sequence)
    if not ack.reason.isupper() or not ack.reason.replace('_','').isalnum():
        raise ProtocolError('BAD_REASON',sequence)
    return frame(f'A1,{sequence},{"OK" if ack.accepted else "ERR"},{ack.command},{ack.reason}')


def decode_ack(data):
    fields=_body(data,6,'A1')
    sequence=_sequence(fields[1],allow_zero=True)
    if fields[2] not in ACK_STATES or (fields[3] not in COMMANDS and fields[3]!='UNKNOWN'):
        raise ProtocolError('BAD_ACK',sequence)
    if not fields[4].isupper() or not fields[4].replace('_','').isalnum():
        raise ProtocolError('BAD_ACK',sequence)
    return Ack(sequence,fields[2]=='OK',fields[3],fields[4])


class LineBuffer:
    """Bounded streaming framing; an overlong line is discarded through newline."""
    def __init__(self):
        self.buffer=bytearray(); self.discarding=False; self.discarded=0

    def feed(self,chunk):
        frames=[]
        for byte in chunk:
            if self.discarding:
                if byte==10:
                    self.discarding=False
                continue
            self.buffer.append(byte)
            if len(self.buffer)>MAX_FRAME:
                self.buffer.clear(); self.discarding=True; self.discarded+=1
            elif byte==10:
                frames.append(bytes(self.buffer)); self.buffer.clear()
        return frames


def sequence_newer(sequence,last):
    return last is None or (0 < ((sequence-last)&MAX_SEQUENCE) < 0x80000000)
