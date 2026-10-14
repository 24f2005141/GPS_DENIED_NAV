"""Ordered history of accepted estimated poses for metrics and visualization."""
import math
from threading import RLock


def stamp_key(stamp):
    return stamp.sec, stamp.nanosec


class TrajectoryHistory:
    def __init__(self):
        self.samples = {}
        self.cutoff = None
        self._lock = RLock()

    def clear(self, cutoff=None):
        with self._lock:
            self.samples.clear()
            self.cutoff = cutoff

    def add_pose(self, pose):
        stamp = stamp_key(pose.header.stamp)
        point = pose.pose.position
        if not (math.isfinite(point.x) and math.isfinite(point.y)):
            return False
        with self._lock:
            if self.cutoff is not None and stamp <= self.cutoff:
                return False
            self.samples[stamp] = point.x, point.y
            return True

    def merge_path(self, path):
        for pose in path.poses:
            self.add_pose(pose)

    def ordered_samples(self):
        with self._lock:
            return [(stamp, self.samples[stamp]) for stamp in sorted(self.samples)]

    def snapshot(self):
        with self._lock:
            return dict(self.samples)

    def plot_coordinates(self, max_gap_s=0.6):
        """Use NaN separators so missing measurements are not drawn as motion."""
        xs, ys, previous = [], [], None
        for stamp, (x, y) in self.ordered_samples():
            current = stamp[0] * 1_000_000_000 + stamp[1]
            if previous is not None and current - previous > max_gap_s * 1_000_000_000:
                xs.append(math.nan); ys.append(math.nan)
            xs.append(x); ys.append(y)
            previous = current
        return xs, ys

    def length_from_start(self, start=(0.0, 0.0), max_gap_s=0.6):
        """Sum observed pose steps; omit unknown motion across sensor gaps."""
        length, gaps = 0.0, 0
        previous_point, previous_stamp = start, None
        for stamp, point in self.ordered_samples():
            current_stamp = stamp[0] * 1_000_000_000 + stamp[1]
            if (previous_stamp is not None and
                    current_stamp - previous_stamp > max_gap_s * 1_000_000_000):
                gaps += 1
            else:
                length += math.dist(previous_point, point)
            previous_point, previous_stamp = point, current_stamp
        return length, gaps
