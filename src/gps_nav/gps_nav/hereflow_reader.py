"""Hardware-only UART session management; measurements remain uncalibrated counts."""
import time

from .parser import FlowParser
from .serial_io import SerialPort


class HereFlowReader:
    def __init__(self, path='/dev/ttyUSB0', baud=19200, timeout=0.01,
                 reconnect_interval=2.0, byte_order='little', stale_after=0.6,
                 port_factory=SerialPort, clock=time.monotonic):
        if timeout < 0 or reconnect_interval < 0 or stale_after <= 0:
            raise ValueError('serial timeouts must be nonnegative and stale_after positive')
        self.path = path
        self.baud = baud
        self.timeout = timeout
        self.reconnect_interval = reconnect_interval
        self.stale_after = stale_after
        self.port_factory = port_factory
        self.clock = clock
        self.parser = FlowParser(byte_order)
        self.port = None
        self.last_attempt = None
        self.last_valid = None
        self.last_error = ''
        self.connection_attempts = 0
        self.connections = 0
        self.reconnects = 0
        self.disconnects = 0
        self.open_errors = 0
        self.read_errors = 0
        self.bytes_received = 0
        self.quality_sum = 0
        self.quality_min = None
        self.quality_max = None
        self.last_packet = None

    def poll(self):
        """Return (validated packets, raw bytes); raw bytes are for diagnostics only."""
        now = self.clock()
        if self.port is None:
            if self.last_attempt is not None and now - self.last_attempt < self.reconnect_interval:
                return [], b''
            self.last_attempt = now
            self.connection_attempts += 1
            try:
                self.port = self.port_factory(self.path, self.baud)
            except (OSError, ValueError) as exc:
                self.open_errors += 1
                self.last_error = f'open: {exc}'
                return [], b''
            self.parser.discarded_bytes += len(self.parser.buffer)
            self.parser.buffer.clear()  # never join a prior port's partial frame
            if self.connections:
                self.reconnects += 1
            self.connections += 1
            self.last_error = ''
        try:
            data = self.port.read(timeout=self.timeout)
        except OSError as exc:
            self.read_errors += 1
            self.last_error = f'read: {exc}'
            self.disconnects += 1
            self.close()
            self.parser.discarded_bytes += len(self.parser.buffer)
            self.parser.buffer.clear()
            self.last_attempt = now
            return [], b''
        self.bytes_received += len(data)
        packets = self.parser.feed(data)
        if packets:
            self.last_valid = now
            self.last_packet = packets[-1]
            for packet in packets:
                self.quality_sum += packet.quality
                self.quality_min = packet.quality if self.quality_min is None else min(self.quality_min, packet.quality)
                self.quality_max = packet.quality if self.quality_max is None else max(self.quality_max, packet.quality)
        return packets, data

    def status(self):
        now = self.clock()
        age = None if self.last_valid is None else max(0.0, now - self.last_valid)
        return {
            'connected': self.port is not None,
            'data_stale': age is None or age > self.stale_after,
            'connection_attempts': self.connection_attempts,
            'connections': self.connections,
            'reconnects': self.reconnects,
            'disconnects': self.disconnects,
            'open_errors': self.open_errors,
            'read_errors': self.read_errors,
            'bytes_received': self.bytes_received,
            'valid_packets': self.parser.valid,
            'invalid_frames': self.parser.invalid,
            'checksum_failures': self.parser.checksum_failures,
            'discarded_bytes': self.parser.discarded_bytes,
            'last_valid_age_s': age,
            'quality_min': self.quality_min,
            'quality_max': self.quality_max,
            'quality_mean': None if not self.parser.valid else self.quality_sum / self.parser.valid,
            'last_raw_x': None if self.last_packet is None else self.last_packet.x,
            'last_raw_y': None if self.last_packet is None else self.last_packet.y,
            'last_error': self.last_error,
        }

    def close(self):
        if self.port is not None:
            port, self.port = self.port, None
            try:
                port.close()
            except OSError as exc:
                self.last_error = f'close: {exc}'
