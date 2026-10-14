"""Pure high-level command gate; it never owns serial or motor pins."""
from dataclasses import dataclass
from .robot_protocol import Command, MAX_SEQUENCE


@dataclass
class Health:
    position_valid: bool
    heading_valid: bool
    flow_fresh: bool
    imu_fresh: bool
    metric_calibrated: bool
    received_at: float


@dataclass
class Guidance:
    recommendation: str
    distance_valid: bool
    reached: bool
    received_at: float


@dataclass
class Link:
    connected: bool
    ack_fresh: bool
    received_at: float


class ControlSupervisor:
    def __init__(self,allow_control=False,max_speed_permille=100,
                 forward_speed_permille=80,turn_speed_permille=60,
                 allow_reverse=False,input_stale_s=0.5,ack_timeout_s=0.5):
        if not (0<max_speed_permille<=1000 and
                0<forward_speed_permille<=max_speed_permille and
                0<turn_speed_permille<=max_speed_permille and
                input_stale_s>0 and ack_timeout_s>0):
            raise ValueError('invalid control limits')
        self.allow_control=allow_control; self.max_speed=max_speed_permille
        self.forward_speed=forward_speed_permille; self.turn_speed=turn_speed_permille
        self.allow_reverse=allow_reverse
        self.input_stale_s=input_stale_s; self.ack_timeout_s=ack_timeout_s
        self.health=None; self.guidance=None; self.link=None; self.goal=None
        self.enabled=False; self.arming=False; self.recovering=False
        self.fault_latched=False; self.arrival_latched=False
        self.reason='physical control disabled at startup'
        self.sequence=0; self.pending_sequence=None; self.pending_type=None
        self.pending_at=None; self.last_issued='STOP'; self.last_ack=None

    def set_goal(self,goal):
        if self.goal is not None and abs(self.goal[0]-goal[0])+abs(self.goal[1]-goal[1])>1e-8:
            self.arrival_latched=False
            self.enabled=False; self.arming=False
            self.pending_sequence=None; self.pending_type=None
            self.reason='destination changed; enable control again'
        self.goal=goal

    def reset_session(self):
        self.stop('session reset')
        self.arrival_latched=False

    def update_health(self,health): self.health=health

    def update_guidance(self,guidance):
        self.guidance=guidance
        if guidance.reached:
            self.arrival_latched=True
            self.stop('destination reached')

    def update_link(self,link): self.link=link

    def _next(self,direction,speed=0):
        self.sequence=(self.sequence%MAX_SEQUENCE)+1
        self.last_issued=direction
        return Command(self.sequence,direction,speed)

    def _ready(self,now):
        if self.health is None or now-self.health.received_at>self.input_stale_s:
            return False,'localization status stale'
        h=self.health
        if not (h.position_valid and h.heading_valid and h.flow_fresh and
                h.imu_fresh and h.metric_calibrated):
            return False,'localization or sensor invalid'
        if self.guidance is None or now-self.guidance.received_at>self.input_stale_s:
            return False,'navigation guidance stale'
        if not self.guidance.distance_valid or self.guidance.recommendation not in (
                'GO STRAIGHT','TURN LEFT','TURN RIGHT','GO BACK','DESTINATION REACHED'):
            return False,'navigation guidance invalid'
        if self.link is None or now-self.link.received_at>self.input_stale_s:
            return False,'microcontroller link status stale'
        if not self.link.connected or not self.link.ack_fresh:
            return False,'microcontroller link or acknowledgement invalid'
        return True,'ready'

    def stop(self,reason='STOP requested'):
        self.enabled=False; self.arming=False; self.recovering=False
        self.pending_sequence=None; self.pending_type=None; self.pending_at=None
        self.reason=reason; self.last_issued='STOP'

    def fault(self,reason):
        self.stop(reason)
        self.fault_latched=True

    def request_enable(self,now):
        if self.enabled or self.arming:
            return False,'control already enabled or arming'
        if not self.allow_control:
            return False,'physical control is disabled by configuration'
        if self.fault_latched:
            return False,'fault latched; request reset after correcting the fault'
        if self.arrival_latched:
            return False,'destination already reached'
        ready,reason=self._ready(now)
        if not ready: return False,reason
        self.arming=True; self.reason='arming microcontroller'
        return True,self.reason

    def request_reset(self,now):
        if not self.fault_latched:
            return False,'no latched fault'
        ready,reason=self._ready(now)
        if not ready: return False,reason
        self.recovering=True; self.reason='reset requested; awaiting microcontroller ACK'
        return True,self.reason

    def acknowledge(self,ack):
        self.last_ack=ack
        if self.pending_sequence==ack.sequence and self.pending_type==ack.command:
            if not ack.accepted:
                self.fault('microcontroller rejected '+ack.command+': '+ack.reason)
            elif ack.command=='ARM':
                self.arming=False; self.enabled=True; self.reason='control enabled'
            elif ack.command=='RESET':
                self.recovering=False; self.fault_latched=False
                self.reason='fault reset; explicit enable required'
            self.pending_sequence=None; self.pending_type=None; self.pending_at=None
        elif not ack.accepted and ack.command not in ('STOP','DISARM'):
            self.fault('microcontroller rejected '+ack.command+': '+ack.reason)

    def tick(self,now):
        if self.arrival_latched:
            self.stop('destination reached')
            return self._next('STOP')
        if self.enabled or self.arming:
            ready,reason=self._ready(now)
            if not ready:
                self.fault(reason)
                return self._next('STOP')
        if self.pending_sequence is not None:
            if now-self.pending_at>self.ack_timeout_s:
                self.fault(self.pending_type+' acknowledgement timeout')
                return self._next('STOP')
            return None
        if self.recovering:
            command=self._next('RESET')
            self.pending_sequence=command.sequence; self.pending_type='RESET'; self.pending_at=now
            return command
        if self.fault_latched:
            return self._next('STOP')
        if self.arming:
            command=self._next('ARM')
            self.pending_sequence=command.sequence; self.pending_type='ARM'; self.pending_at=now
            return command
        if not self.enabled:
            return self._next('STOP')
        mapping={'GO STRAIGHT':('FORWARD',self.forward_speed),
                 'TURN LEFT':('LEFT',self.turn_speed),
                 'TURN RIGHT':('RIGHT',self.turn_speed)}
        if self.allow_reverse:
            mapping['GO BACK']=('BACKWARD',self.forward_speed)
        elif self.guidance.recommendation=='GO BACK':
            self.stop('target behind robot; reverse motion not enabled')
            return self._next('STOP')
        direction,speed=mapping.get(self.guidance.recommendation,('STOP',0))
        return self._next(direction,min(speed,self.max_speed))
