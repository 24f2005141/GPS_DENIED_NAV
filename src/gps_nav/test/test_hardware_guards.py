import json
import math
import errno
import os
import pty
import threading
import time

from gps_nav.core import Navigator, Tracker
from gps_nav.hereflow_diagnostic import main as diagnostic_main
from gps_nav.hereflow_reader import HereFlowReader


def frame(x, y, quality=255):
    data = x.to_bytes(2, 'little', signed=True) + y.to_bytes(2, 'little', signed=True)
    return b'\xfe\x04' + data + bytes([sum(data) & 255, quality, 0xaa])


class Clock:
    now = 0.0

    def __call__(self):
        return self.now


class FakePort:
    def __init__(self, reads):
        self.reads = iter(reads)
        self.closed = False

    def read(self, timeout):
        result = next(self.reads)
        if isinstance(result, OSError):
            raise result
        return result

    def close(self):
        self.closed = True


def test_reader_disconnect_reconnect_clears_partial_packet():
    clock = Clock()
    first = FakePort([b'\xfe\x04\x7f', OSError('unplugged')])
    second = FakePort([frame(-7, 8, 0), frame(9, -10, 255)])
    ports = iter([first, second])

    def factory(path, baud):
        assert path == '/dev/fake' and baud == 19200
        return next(ports)

    reader = HereFlowReader('/dev/fake', reconnect_interval=1, port_factory=factory, clock=clock)
    assert reader.poll()[0] == []
    assert reader.parser.buffer
    assert reader.poll()[0] == []
    assert first.closed and not reader.parser.buffer
    assert reader.status()['disconnects'] == 1
    clock.now = 0.5
    assert reader.poll()[0] == []
    assert reader.status()['connection_attempts'] == 1
    clock.now = 1.0
    packets, raw = reader.poll()
    assert raw == frame(-7, 8, 0)
    assert [(p.x, p.y, p.quality) for p in packets] == [(-7, 8, 0)]
    reader.poll()
    status = reader.status()
    assert status['connections'] == 2 and status['reconnects'] == 1
    assert status['valid_packets'] == 2 and status['quality_min'] == 0
    assert status['quality_max'] == 255 and status['quality_mean'] == 127.5
    clock.now = 1.7
    assert reader.status()['data_stale']
    reader.close()
    assert second.closed


def test_reader_missing_port_and_invalid_frame():
    clock = Clock()
    def unavailable(path, baud):
        raise FileNotFoundError(path)
    reader = HereFlowReader('/dev/absent', reconnect_interval=1,
                            port_factory=unavailable, clock=clock)
    assert reader.poll()[0] == []
    assert reader.status()['open_errors'] == 1
    clock.now = 0.1
    reader.poll()
    assert reader.status()['connection_attempts'] == 1

    bad = bytearray(frame(1, 2))
    bad[6] ^= 1
    reader = HereFlowReader('/dev/fake', port_factory=lambda path, baud: FakePort([bytes(bad)]),
                            clock=clock)
    assert reader.poll()[0] == []
    assert reader.status()['checksum_failures'] == 1
    assert reader.status()['valid_packets'] == 0


def test_reader_reports_permission_denial():
    def denied(path, baud):
        raise PermissionError(errno.EACCES, 'permission denied', path)
    reader = HereFlowReader('/dev/restricted', port_factory=denied)
    assert reader.poll()[0] == []
    assert reader.status()['open_errors'] == 1
    assert 'permission denied' in reader.status()['last_error'].lower()


def test_heading_invalidates_recent_metric_position_and_arrival():
    tracker = Tracker(stale_s=0.5)
    assert tracker.imu(0, 1.0)
    assert tracker.flow(1, 0, 1.1, 255, True)
    assert tracker.status(1.2)[0]
    tracker.heading_valid = False  # equivalent to rejected IMU covariance
    valid, imu_fresh, flow_fresh, reason = tracker.status(1.2)
    assert not valid and imu_fresh and flow_fresh
    assert reason == 'IMU heading unavailable'
    assert not tracker.flow(1, 0, 1.3, 255, True)
    assert tracker.x == 1

    navigator = Navigator(tolerance=0.25, stable_updates=2)
    for _ in range(3):
        guidance = navigator.evaluate(0, 0, math.nan, 0.1, 0, True, False)
        assert guidance.label == 'HEADING UNAVAILABLE'
        assert not guidance.reached


def test_missing_calibration_and_stale_flow_reject_position():
    tracker = Tracker(stale_s=0.5)
    tracker.imu(0, 1.0)
    assert not tracker.flow(5, 0, 1.1, 255, False)
    assert tracker.x == 0 and not tracker.status(1.2)[0]
    assert tracker.flow(1, 0, 1.2, 255, True)
    assert tracker.status(1.3)[0]
    assert not tracker.status(1.8)[0]
    assert tracker.x == 1
    assert not tracker.flow(float('nan'), 0, 1.9, 255, True)
    assert not tracker.status(1.9)[0]


def test_diagnostic_missing_port_is_explicit(capsys, tmp_path):
    missing = str(tmp_path / 'not-a-device')
    code = diagnostic_main(['--serial-port', missing, '--duration', '0.03'])
    report = json.loads(capsys.readouterr().out)
    assert code == 2
    assert not report['exists'] and not report['connected']
    assert report['open_errors'] >= 1 and report['valid_packets'] == 0
    assert report['metric_localization_tested'] is False


def test_diagnostic_receives_raw_packets_without_navigation(capsys, tmp_path):
    master, slave = pty.openpty()
    path = os.ttyname(slave)
    capture = tmp_path / 'uart.jsonl'
    def writer():
        time.sleep(0.04)
        for _ in range(5):
            os.write(master, frame(-3, 4, 127))
            time.sleep(0.015)
    thread = threading.Thread(target=writer)
    thread.start()
    try:
        code = diagnostic_main(['--serial-port', path, '--duration', '0.18',
                                '--capture', str(capture)])
        report = json.loads(capsys.readouterr().out)
        assert code == 0 and report['valid_packets'] >= 1
        assert report['quality_min'] == 127
        assert report['metric_localization_tested'] is False
        assert capture.read_text().strip()
    finally:
        thread.join()
        os.close(master); os.close(slave)
