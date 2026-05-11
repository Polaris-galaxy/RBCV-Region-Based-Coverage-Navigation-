"""rosbag 时间戳对齐：平面位姿线性插值。"""

from __future__ import annotations

import bisect
from dataclasses import dataclass


@dataclass
class PoseTimeline:
    """``samples`` 按时间升序，每项 ``(t_s, x, y, yaw)``。"""

    t: list[float]
    x: list[float]
    y: list[float]
    yaw: list[float]

    @classmethod
    def from_samples(cls, samples: list[tuple[float, float, float, float]]) -> PoseTimeline:
        s = sorted(samples, key=lambda u: u[0])
        return cls(
            t=[a[0] for a in s],
            x=[a[1] for a in s],
            y=[a[2] for a in s],
            yaw=[a[3] for a in s],
        )

    def xy_at(self, t_query: float) -> tuple[float, float]:
        if not self.t:
            raise ValueError("empty PoseTimeline")
        if t_query <= self.t[0]:
            return (self.x[0], self.y[0])
        if t_query >= self.t[-1]:
            return (self.x[-1], self.y[-1])
        i = bisect.bisect_right(self.t, t_query)
        t0, t1 = self.t[i - 1], self.t[i]
        u = (t_query - t0) / (t1 - t0) if t1 > t0 else 0.0
        x = self.x[i - 1] + u * (self.x[i] - self.x[i - 1])
        y = self.y[i - 1] + u * (self.y[i] - self.y[i - 1])
        return (float(x), float(y))
