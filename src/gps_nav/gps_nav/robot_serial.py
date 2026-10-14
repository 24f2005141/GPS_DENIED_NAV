"""Nonblocking POSIX USB serial transport for protocol V1."""
import errno
import os
import select
import termios
from .robot_protocol import (Command, LineBuffer, ProtocolError,
                             decode_ack, encode_command)

BAUD={9600:termios.B9600,19200:termios.B19200,57600:termios.B57600,
      115200:termios.B115200}


class DuplexPort:
    def __init__(self,path,baud):
        if baud not in BAUD: raise ValueError('unsupported robot serial baud rate')
        self.fd=os.open(path,os.O_RDWR|os.O_NOCTTY|os.O_NONBLOCK)
        try:
            flags=termios.tcgetattr(self.fd)
            flags[0]=0; flags[1]=0
            flags[2]=termios.CLOCAL|termios.CREAD|termios.CS8
            flags[3]=0; flags[4]=BAUD[baud]; flags[5]=BAUD[baud]
            flags[6][termios.VMIN]=0; flags[6][termios.VTIME]=0
            termios.tcsetattr(self.fd,termios.TCSANOW,flags)
        except Exception:
            self.close(); raise

    def read(self):
        if not select.select([self.fd],[],[],0)[0]:
            return b''
        try:
            data=os.read(self.fd,4096)
        except BlockingIOError:
            return b''
        if not data:
            raise OSError(errno.EIO,'robot serial disconnected')
        return data

    def write(self,data):
        while data:
            try:
                sent=os.write(self.fd,data)
            except BlockingIOError:
                if not select.select([], [self.fd], [], 0.01)[1]:
                    raise OSError(errno.EAGAIN,'robot serial write stalled')
                continue
            if sent<=0: raise OSError(errno.EIO,'robot serial write failed')
            data=data[sent:]

    def close(self):
        if self.fd is not None:
            os.close(self.fd); self.fd=None


class RobotSerialClient:
    def __init__(self,path='/dev/ttyACM0',baud=115200,reconnect_s=1.0,
                 ack_timeout_s=0.4,command_timeout_s=0.35,port_factory=DuplexPort):
        if reconnect_s<=0 or ack_timeout_s<=0 or command_timeout_s<=0:
            raise ValueError('invalid robot serial timeouts')
        self.path=path; self.baud=baud; self.reconnect_s=reconnect_s
        self.ack_timeout_s=ack_timeout_s; self.command_timeout_s=command_timeout_s
        self.port_factory=port_factory; self.port=None
        self.next_connect_at=0.0; self.open_errors=0; self.disconnects=0
        self.reason='not connected'; self.frames=LineBuffer(); self.invalid_acks=0
        self.pending=None; self.pending_at=None; self.queued=None
        self.last_command_at=None; self.last_sequence=1
        self.last_ack=None; self.last_ack_at=None
        self.idle_stopped=False

    @property
    def connected(self): return self.port is not None

    def ack_fresh(self,now):
        return self.connected and self.last_ack is not None and self.last_ack.accepted and self.last_ack_at is not None and (
            now-self.last_ack_at<=self.ack_timeout_s)

    def queue(self,command,now):
        encode_command(command)  # Reject malformed ROS messages before serial output.
        self.last_command_at=now; self.last_sequence=command.sequence
        self.idle_stopped=command.direction=='STOP'
        self.queued=command
        if command.direction=='STOP' and self.pending is not None:
            self.pending=None; self.pending_at=None

    def _send(self,command,now):
        self.port.write(encode_command(command))
        self.pending=command; self.pending_at=now

    def _close(self,now,reason):
        if self.port is not None:
            try: self.port.close()
            except OSError: pass
            self.disconnects+=1
        self.port=None; self.reason=reason; self.next_connect_at=now+self.reconnect_s
        self.pending=None; self.pending_at=None; self.queued=None
        self.last_ack_at=None; self.frames=LineBuffer(); self.idle_stopped=True

    def close(self,now=0.0):
        if self.port is not None:
            try: self.port.write(encode_command(Command(self.last_sequence,'STOP',0)))
            except OSError: pass
        self._close(now,'closed')

    def tick(self,now):
        acknowledgements=[]
        if self.port is None:
            if now<self.next_connect_at: return acknowledgements
            try:
                self.port=self.port_factory(self.path,self.baud)
            except (OSError,ValueError) as exc:
                self.open_errors+=1; self.reason=str(exc)
                self.next_connect_at=now+self.reconnect_s
                return acknowledgements
            self.reason='connected; awaiting STOP acknowledgement'
            self.frames=LineBuffer(); self.pending=None; self.queued=None
            self.last_ack_at=None; self.idle_stopped=True
            try: self._send(Command(self.last_sequence,'STOP',0),now)
            except OSError as exc:
                self._close(now,str(exc))
            return acknowledgements
        try:
            chunk=self.port.read()
            for line in self.frames.feed(chunk):
                try: ack=decode_ack(line)
                except ProtocolError:
                    self.invalid_acks+=1; continue
                acknowledgements.append(ack)
                if self.pending and ack.sequence==self.pending.sequence:
                    self.pending=None; self.pending_at=None
                    self.last_ack=ack; self.last_ack_at=now
                    self.reason='acknowledged' if ack.accepted else 'command rejected: '+ack.reason
            if self.pending and now-self.pending_at>self.ack_timeout_s:
                try: self.port.write(encode_command(Command(self.last_sequence,'STOP',0)))
                except OSError: pass
                self._close(now,'acknowledgement timeout; STOP attempted')
                return acknowledgements
            if (self.last_command_at is None or
                    now-self.last_command_at>self.command_timeout_s) and not self.idle_stopped:
                self.queued=Command(self.last_sequence,'STOP',0)
                self.pending=None; self.pending_at=None; self.idle_stopped=True
                self.reason='supervisor command stale; STOP sent'
            if self.queued is not None and self.pending is None:
                command=self.queued; self.queued=None
                self._send(command,now)
        except OSError as exc:
            self._close(now,str(exc))
        return acknowledgements
