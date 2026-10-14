"""Deterministic planar path generator; truth is never fed to the estimator."""
from dataclasses import dataclass
import math
import random
from .core import rotate, wrap

PATHS = {
    'straight': [(3.0, 1.0, 0.0)],
    'turn': [(1.0, 1.0, 0.0), (1.0, 0.0, math.pi/2), (1.0, 1.0, 0.0)],
    'multi': [(1.0, 1.0, 0.0), (1.0, 0.0, math.pi/2), (1.0, 1.0, 0.0),
              (1.0, 0.0, math.pi/2), (1.0, 1.0, 0.0), (1.0, 0.0, -math.pi/2),
              (1.0, 1.0, 0.0), (1.0, 0.0, -math.pi/2), (3.0, 1.0, 0.0),
              (1.0, 0.0, -math.pi/2), (2.0, 1.0, 0.0)],
}
GOALS = {'straight': (3.0, 0.0), 'turn': (1.0, 1.0), 'multi': (3.0, 0.0)}


@dataclass
class Sample:
    t: float
    x: float
    y: float
    yaw: float
    dx_body: float
    dy_body: float
    yaw_rate: float
    acceleration_x: float


class PathSimulator:
    def __init__(self, path='straight', dt=0.05, seed=42, noise_m=0.0,
                 yaw_bias_rad_s=0.0, dropout_start=-1.0, dropout_end=-1.0,
                 low_quality_start=-1.0, low_quality_end=-1.0):
        if path not in PATHS or dt <= 0 or noise_m < 0:
            raise ValueError('invalid simulation configuration')
        self.path, self.dt = path, dt
        self.rng = random.Random(seed)
        self.noise_m, self.yaw_bias_rad_s = noise_m, yaw_bias_rad_s
        self.dropout_start, self.dropout_end = dropout_start, dropout_end
        self.low_quality_start, self.low_quality_end = low_quality_start, low_quality_end
        self.t = self.x = self.y = self.yaw = 0.0
        self.segment = self.elapsed = 0
        self.last_speed = 0.0

    def step(self):
        if self.segment >= len(PATHS[self.path]):
            return None
        duration, speed, rate = PATHS[self.path][self.segment]
        dt = min(self.dt, duration - self.elapsed)
        midpoint_yaw = self.yaw + rate * dt / 2
        dx, dy = rotate(speed * dt, 0.0, midpoint_yaw)
        self.x += dx; self.y += dy
        self.yaw = wrap(self.yaw + rate * dt)
        self.t += dt; self.elapsed += dt
        sample = Sample(self.t, self.x, self.y, self.yaw, speed * dt, 0.0,
                        rate + self.yaw_bias_rad_s, (speed - self.last_speed) / dt)
        self.last_speed = speed
        if self.elapsed >= duration - 1e-9:
            self.segment += 1; self.elapsed = 0.0
        return sample

    def manual_step(self, speed, rate):
        if not (math.isfinite(speed) and math.isfinite(rate)):
            raise ValueError('manual command must be finite')
        dt = self.dt
        midpoint_yaw = self.yaw + rate * dt / 2
        dx, dy = rotate(speed * dt, 0.0, midpoint_yaw)
        self.x += dx; self.y += dy
        self.yaw = wrap(self.yaw + rate * dt)
        self.t += dt
        sample = Sample(self.t, self.x, self.y, self.yaw, speed * dt, 0.0,
                        rate + self.yaw_bias_rad_s, (speed - self.last_speed) / dt)
        self.last_speed = speed
        return sample

    def flow_available(self):
        return not self.dropout_start <= self.t < self.dropout_end

    def quality(self):
        return 1 if self.low_quality_start <= self.t < self.low_quality_end else 255

    def noisy_flow(self, sample):
        return (sample.dx_body + self.rng.gauss(0, self.noise_m),
                sample.dy_body + self.rng.gauss(0, self.noise_m))


class GoalSimulator(PathSimulator):
    """Obstacle-free point-goal motion: rotate in place, then travel on the line."""

    def __init__(self, goal_x, goal_y, speed=0.6, turn_rate=math.pi/2,
                 initial_heading=None, **kwargs):
        if not all(math.isfinite(v) for v in (goal_x, goal_y, speed, turn_rate)):
            raise ValueError('goal and motion parameters must be finite')
        if speed <= 0 or turn_rate <= 0:
            raise ValueError('speed and turn rate must be positive')
        super().__init__(path='straight', **kwargs)
        self.goal_x, self.goal_y = goal_x, goal_y
        self.speed, self.turn_rate = speed, turn_rate
        self.yaw = (math.atan2(goal_y, goal_x) if initial_heading is None and
                    math.hypot(goal_x, goal_y) > 0 else
                    (0.0 if initial_heading is None else wrap(initial_heading)))

    def step(self):
        dx, dy = self.goal_x - self.x, self.goal_y - self.y
        distance = math.hypot(dx, dy)
        if distance <= 1e-10:
            return self.manual_step(0.0, 0.0)
        error = wrap(math.atan2(dy, dx) - self.yaw)
        if abs(error) > 1e-8:
            rate = math.copysign(min(self.turn_rate, abs(error) / self.dt), error)
            return self.manual_step(0.0, rate)
        return self.manual_step(min(self.speed, distance / self.dt), 0.0)
