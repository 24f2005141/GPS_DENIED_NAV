"""Streaming parser for the nine-byte FE 04 optical-flow UART frame."""
from dataclasses import dataclass


@dataclass(frozen=True)
class Packet:
    x: int
    y: int
    quality: int


class FlowParser:
    def __init__(self, byte_order='little'):
        if byte_order not in ('little', 'big'):
            raise ValueError('byte_order must be little or big')
        self.byte_order = byte_order
        self.buffer = bytearray()
        self.valid = 0
        self.invalid = 0
        self.checksum_failures = 0
        self.discarded_bytes = 0

    def feed(self, data):
        self.buffer.extend(data)
        packets = []
        while len(self.buffer) >= 2:
            start = self.buffer.find(b'\xfe\x04')
            if start < 0:
                # Keep a possible partial header.
                self.discarded_bytes += len(self.buffer) - (1 if self.buffer[-1] == 0xfe else 0)
                self.buffer[:] = self.buffer[-1:] if self.buffer[-1] == 0xfe else b''
                break
            if start:
                self.discarded_bytes += start
                del self.buffer[:start]
            if len(self.buffer) < 9:
                break
            frame = self.buffer[:9]
            if frame[8] != 0xaa:
                self.invalid += 1
                self.discarded_bytes += 1
                del self.buffer[0]
                continue
            if (sum(frame[2:6]) & 0xff) != frame[6]:
                self.invalid += 1
                self.checksum_failures += 1
                self.discarded_bytes += 1
                del self.buffer[0]
                continue
            x = int.from_bytes(frame[2:4], self.byte_order, signed=True)
            y = int.from_bytes(frame[4:6], self.byte_order, signed=True)
            packets.append(Packet(x, y, frame[7]))
            self.valid += 1
            del self.buffer[:9]
        return packets
