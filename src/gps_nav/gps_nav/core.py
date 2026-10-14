"""Pure estimation and guidance logic. ENU x East/y North; yaw CCW from East."""
from dataclasses import dataclass
import math


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def rotate(x, y, angle):
    c, s = math.cos(angle), math.sin(angle)
    return c * x - s * y, s * x + c * y


@dataclass
class Tracker:
    stale_s: float = 0.6
    min_quality: int = 30
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    heading_valid: bool = False
    position_valid: bool = False
    metric_calibrated: bool = False
    last_imu: float | None = None
    last_flow: float | None = None
    last_accepted_flow: float | None = None
    flow_quality: int = 0
    flow_healthy: bool = False
    reason: str = 'awaiting sensors'

    def reset(self):
        stale, quality = self.stale_s, self.min_quality
        self.__dict__.update(Tracker(stale, quality).__dict__)

    def imu(self, yaw, stamp):
        if not (math.isfinite(yaw) and math.isfinite(stamp)):
            return False
        if self.last_imu is not None and stamp <= self.last_imu:
            return False
        self.yaw = wrap(yaw)
        self.last_imu = stamp
        self.heading_valid = True
        return True

    def flow(self, dx, dy, stamp, quality, metric_valid, mounting_rad=0.0):
        if not all(math.isfinite(v) for v in (dx, dy, stamp, mounting_rad)):
            self.flow_healthy = False; self.reason = 'nonfinite flow'; return False
        if self.last_flow is not None and stamp <= self.last_flow:
            self.flow_healthy = False; self.reason = 'duplicate/out-of-order flow'; return False
        self.last_flow = stamp
        self.flow_quality = quality
        if quality < self.min_quality:
            self.flow_healthy = False; self.reason = 'low flow quality'; return False
        if not metric_valid:
            self.flow_healthy = False; self.metric_calibrated = False
            self.reason = 'uncalibrated flow units'; return False
        if (not self.heading_valid or self.last_imu is None or
                stamp - self.last_imu > self.stale_s or stamp < self.last_imu - self.stale_s):
            self.flow_healthy = False; self.reason = 'IMU unavailable/stale'; return False
        bx, by = rotate(dx, dy, mounting_rad)
        ex, ny = rotate(bx, by, self.yaw)
        self.x += ex; self.y += ny
        self.position_valid = True
        self.metric_calibrated = True
        self.flow_healthy = True
        self.last_accepted_flow = stamp
        self.reason = 'ok'
        return True

    def status(self, now):
        imu_fresh = self.last_imu is not None and 0 <= now - self.last_imu <= self.stale_s
        flow_fresh = (self.flow_healthy and self.last_accepted_flow is not None and
                      0 <= now - self.last_accepted_flow <= self.stale_s)
        valid = self.position_valid and self.heading_valid and imu_fresh and flow_fresh
        reason = ('ok' if valid else
                  ('metric calibration unavailable' if not self.metric_calibrated else
                   'IMU heading unavailable' if not self.heading_valid else
                   'IMU stale' if not imu_fresh else
                   self.reason if not self.flow_healthy else 'flow stale'))
        return valid, imu_fresh, flow_fresh, reason


@dataclass
class Guidance:
    distance: float = math.nan
    bearing: float = math.nan
    error: float = math.nan
    label: str = 'POSITION UNRELIABLE'
    reached: bool = False


class Navigator:
    def __init__(self, tolerance=0.25, stable_updates=3, straight_deg=15, back_deg=135):
        if tolerance <= 0 or stable_updates < 1 or not (0 < straight_deg < back_deg < 180):
            raise ValueError('invalid navigation thresholds')
        self.tolerance = tolerance
        self.stable_updates = stable_updates
        self.straight = math.radians(straight_deg)
        self.back = math.radians(back_deg)
        self.arrival_count = 0

    def reset(self):
        self.arrival_count = 0

    def evaluate(self, x, y, yaw, goal_x, goal_y, position_valid, heading_valid, degraded=False):
        if not position_valid:
            self.arrival_count = 0
            return Guidance(label='POSITION UNRELIABLE')
        dx, dy = goal_x - x, goal_y - y
        distance = math.hypot(dx, dy)
        bearing = math.atan2(dy, dx)
        error = wrap(bearing - yaw) if heading_valid else math.nan
        self.arrival_count = self.arrival_count + 1 if heading_valid and distance <= self.tolerance else 0
        reached = self.arrival_count >= self.stable_updates
        if reached:
            label = 'DESTINATION REACHED'
        elif degraded:
            label = 'SENSOR DEGRADED'
        elif not heading_valid:
            label = 'HEADING UNAVAILABLE'
        elif abs(error) >= self.back:
            label = 'GO BACK'
        elif abs(error) <= self.straight:
            label = 'GO STRAIGHT'
        else:
            label = 'TURN LEFT' if error > 0 else 'TURN RIGHT'
        return Guidance(distance, bearing, error, label, reached)
