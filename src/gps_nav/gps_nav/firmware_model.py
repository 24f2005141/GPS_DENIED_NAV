"""Hardware-independent microcontroller behavior for protocol and watchdog tests."""
from .robot_protocol import Ack, ProtocolError, decode_command, encode_ack, sequence_newer


class MicrocontrollerModel:
    def __init__(self,watchdog_s=0.5,max_speed_permille=200,motor_sink=None):
        if watchdog_s<=0 or not 0<max_speed_permille<=1000:
            raise ValueError('invalid watchdog or speed limit')
        self.watchdog_s=watchdog_s; self.max_speed=max_speed_permille
        self.motor_sink=motor_sink
        self.armed=False; self.fault=False; self.direction='STOP'; self.speed=0
        self.last_sequence=None; self.last_motion_time=None
        self.stop_count=1

    def stop(self):
        self.direction='STOP'; self.speed=0; self.armed=False
        self.stop_count+=1
        if self.motor_sink is not None:
            self.motor_sink('STOP',0)

    def watchdog(self,now):
        if (self.armed and self.last_motion_time is not None and
                now-self.last_motion_time>self.watchdog_s):
            self.stop(); self.fault=True
            return True
        return False

    def process(self,data,now):
        self.watchdog(now)
        try:
            command=decode_command(data)
        except ProtocolError as exc:
            self.stop(); self.fault=True
            return encode_ack(Ack(exc.sequence,False,'UNKNOWN',exc.reason))
        seq=command.sequence; direction=command.direction
        # A well-framed STOP always wins, including duplicate or old sequences.
        if direction in ('STOP','DISARM'):
            self.stop()
            if sequence_newer(seq,self.last_sequence): self.last_sequence=seq
            return encode_ack(Ack(seq,True,direction,'STOPPED'))
        if not sequence_newer(seq,self.last_sequence):
            self.stop(); self.fault=True
            return encode_ack(Ack(seq,False,direction,'STALE'))
        self.last_sequence=seq
        if direction=='RESET':
            self.stop(); self.fault=False
            return encode_ack(Ack(seq,True,direction,'RESET'))
        if self.fault:
            self.stop()
            return encode_ack(Ack(seq,False,direction,'FAULT'))
        if direction=='ARM':
            self.armed=True; self.last_motion_time=now
            return encode_ack(Ack(seq,True,direction,'ARMED'))
        if not self.armed:
            self.stop()
            return encode_ack(Ack(seq,False,direction,'DISARMED'))
        if command.speed_permille>self.max_speed:
            self.stop(); self.fault=True
            return encode_ack(Ack(seq,False,direction,'BAD_SPEED'))
        if self.motor_sink is not None and not self.motor_sink(direction,command.speed_permille):
            self.stop(); self.fault=True
            return encode_ack(Ack(seq,False,direction,'NO_DRIVER'))
        self.direction=direction; self.speed=command.speed_permille
        self.last_motion_time=now
        return encode_ack(Ack(seq,True,direction,'ACCEPTED'))
